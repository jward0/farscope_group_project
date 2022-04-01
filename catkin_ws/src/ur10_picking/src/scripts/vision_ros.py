#!/usr/bin/env python

from ur10_picking.srv import detect_markers
from ur10_picking.srv import detect_object
from ur10_picking.msg import arucoMarker
from ur10_picking.msg import arucoMarkerArray
from geometry_msgs.msg import Point

import rospy

import cv2
import numpy as np
import pyrealsense2 as rs

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Vision_Core(object):
    def __init__(self):
        """Init function for the main Vision Core system that holds the realsense variables 
        """
        self.hlow = 15
        self.hhigh = 180
        self.slow = 35
        self.shigh = 255
        self.vlow = 100
        self.vhigh = 255

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.profile = self.pipeline.start(self.config)

        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Using depth scale of ", self.depth_scale)

        self.clipping_distance_in_meters = 2
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParmas = cv2.aruco.DetectorParameters_create()

    
    def start(self):
        """Function to start a ROS topics from the vision system (Currently no topics are being published)
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            

            rate.sleep()  

    

def handle_detect_markers(req):
    """Handler function on service request to detect ARUCO 3D coordinates and their ID

    Args:
        req (Boolean): Not used boolean but needed for ROS service reqest

    Returns:
        arucoMarkerArray: An array that holds Marker objects, which in turn holds a geometry_msgs/Point and a marker ID
    """
    markers = []

    frames = vision_core.pipeline.wait_for_frames()
    aligned_frames = vision_core.align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        return

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    grey_color = 153
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    bg_removed = np.where((depth_image_3d > vision_core.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    #ARCO DETECTION
    (corners, ids, rejected) = cv2.aruco.detectMarkers(bg_removed, vision_core.arucoDict, parameters=vision_core.arucoParmas)
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
                
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            depth = round(aligned_depth_frame.get_distance(cX, cY),3)

            depth_intrinsic = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            result = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [cX, cY], depth)
            result_string = "X: %f, Y: %f, Z: %f" % (result[0], result[1], result[2]) 
            
            print(result_string)
            mark = arucoMarker()
            mark.markerID = markerID
            # mark.point = Point(result[0],result[2],-result[1])
            mark.point = Point(result[2], result[0], -result[1])
            markers.append(mark)

    toReturn = arucoMarkerArray()
    toReturn.markers = markers

    return toReturn

def handle_detect_objects(req):
    """Detects an object in a shelf 

    Args:
        req (Boolean): Not used boolean but needed for ROS service reqest

    Returns:
        geometry_msgs/Point: The 3D point of the object relative to the camera
    """
    object = Point(0,0,0)
    frames = vision_core.pipeline.wait_for_frames()
    aligned_frames = vision_core.align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    for x in range(5):
        vision_core.pipeline.wait_for_frames()

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, vision_core.arucoDict, parameters=vision_core.arucoParmas)
    shelf_image, shelf_image_depth = get_shelf(color_image, depth_image,corners, ids)
    maskedShelf, maskedShelf_mask = maskShelf(shelf_image)

    contours = findContours(maskedShelf_mask, 1)

    for c in contours:
            rect = cv2.boundingRect(c)
            x,y,w,h = rect
            depthLevel = getDepthOfRect(depth_image,rect)
            coords = getDepth([x+w/2, w+h/2], depthLevel, aligned_depth_frame)
            print("Detected Objects")
            print(coords)
            # object = Point(coords[0], coords[2], -coords[1])
            object = Point(coords[2], coords[0], -coords[1]) 
            
    return object

def get_shelf(color, depth, corners, ids):
    """Gets just the pixels of the shelf between the markers

    Args:
        color (int8 3*w*h array): Color image array from the real sense camera, w and h can be what ever
        depth (int16 1*w*h array): Depth image array from the real sense camera
        corners (list): A list containing the corners of the aruco markers 
        ids (list): A list containing the id of the aruco markers
    
    Returns:
        color (int8 3*w*h array): A aruco cropped image array 
        depth (int16 1*w*h array): A aruco cropped depth image array
    """
    mask = np.zeros(color.shape, np.uint8)
    if len(corners) > 0:
		# flatten the ArUco IDs list
        ids = ids.flatten()
        markerCenters = {}
		# loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

			# compute and draw the center (x, y)-coordinates of the
			# ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            if markerID == 0:
                markerCenters[markerID] = [bottomRight[0], bottomRight[1]]
            if markerID == 1:
                markerCenters[markerID] = [bottomLeft[0], bottomLeft[1]]
            if markerID == 2:
                markerCenters[markerID] = [topRight[0], topRight[1]]
            if markerID == 3:
                markerCenters[markerID] = [topLeft[0], topLeft[1]]

            cv2.circle(color, (cX, cY), 4, (0, 0, 255), -1)
            
        if(len(markerCenters) >= 4):
            pts = np.array([markerCenters[0],markerCenters[1],markerCenters[3],markerCenters[2]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(mask,[pts],(255,255,255))
            mask_gray=cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            color = cv2.bitwise_and(color,color,mask = mask_gray)
            depth = np.bitwise_and(depth, mask_gray)
        
    return color, depth

def maskShelf(color_img):
    """Function to mask the shelf image to contain only the object and remove the background colours

    Args:
        color_img (int8 3*w*h array): The aruco cropped image of the shelf

    Returns:
        color_img (int8 3*w*h array): The masked cropped image of the shelf containing only the object
        mask_background (int8 1*w*h array): The binary image mask that was used.
    """
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

    mask_background = cv2.inRange(hsv,(vision_core.hlow,vision_core.slow,vision_core.vlow),(vision_core.hhigh,vision_core.shigh,vision_core.vhigh))

    kernel = np.ones((10,10),np.uint8)          
    mask_background = cv2.morphologyEx(mask_background, cv2.MORPH_CLOSE, kernel)

    color_img = cv2.bitwise_and(color_img,color_img,mask =  mask_background)

    return color_img, mask_background

def findContours(img, amount):
    """A function to find the contours of any image it is given, sorted by the biggest area of the contour

    Args:
        img (int8 3*w*h array): The input image to detect contours on
        amount (int): How many contours to return

    Returns:
        Contour: a list of contours based on area
    """
    objects = []

    contours, hierarchy = cv2.findContours(image=img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contours = contours[:amount]
    toDraw = []
    #cv2.drawContours(image=shelf_image, contours=contours[:1], contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    return contours

def getDepthOfRect(depth, rect):
    """Returns the depth of a bounding box rect

    Args:
        depth (_type_): The full depth image from the camera
        rect (_type_): A bounding box rect

    Returns:
        depth (depth array): A filted out depth array based on the rect
    """
    x,y,w,h = rect
    depth = depth[x+w/2,y+h/2].astype(float)
    depth = depth * vision_core.depth_scale
    return depth

def getDepth(point, depth, aligned_depth_frame):
    """_summary_

    Args:
        point (x,y Point): The point the depth information is needed of
        depth (_type_): NOT USED ATM
        aligned_depth_frame (_type_): NOT USED ATM

    Returns:
        int: the depth of the requested point using camera intrinsics
    """
    depth_intrinsic = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    result = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [point[0], point[1]], depth)
    #result_string = "X: %f, Y: %f, Z: %f" % (result[0], result[1], result[2]) 

    return result

def init_ros():
    """Will start the ros node 'vision_server' and sets up the service handlers
    """
    rospy.init_node('vision_server')
    markerDetection = rospy.Service('detect_markers', detect_markers, handle_detect_markers)
    detectObject = rospy.Service('detect_object', detect_object, handle_detect_objects)
    print("Vision Server Ready")


if __name__ == "__main__":
    global vision_core
    init_ros()

    vision_core = Vision_Core()
    vision_core.start()
