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

import os


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

def getPoly(img1, img2):
    MIN_MATCH_COUNT = 10

    # Initiate SIFT detector
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.4*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        print(pts)
        try:
            dst = cv2.perspectiveTransform(pts,M)
        except:
            return None;
        return [np.int32(dst)]
    else:
        return None
    
if __name__ == "__main__":
    global vision_core
    print(cv2.__version__)

    vision_core = Vision_Core()

    object_name = "eraser"

    for x in range(60):
        vision_core.pipeline.wait_for_frames()

    frames = vision_core.pipeline.wait_for_frames()
    aligned_frames = vision_core.align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, vision_core.arucoDict, parameters=vision_core.arucoParmas)
    shelf_image, shelf_image_depth = get_shelf(color_image, depth_image,corners, ids)
    maskedShelf, maskedShelf_mask = maskShelf(shelf_image)

    img2 = cv2.cvtColor(maskedShelf, cv2.COLOR_BGR2GRAY)
    directory = "/home/farscope/farscope_group_project/catkin_ws/src/ur10_picking/src/scripts/data/" + object_name

    polys = []
    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f):
            print("Testing image")
            img1 = cv2.imread(f,0)
            polyPoints = getPoly(img1, img2)
            color_image = cv2.polylines(color_image, polyPoints, True, (0,0,255),3, cv2.LINE_AA)
            polys.append(polyPoints)

    for p in polys:
        if p:
            center = np.mean(np.squeeze(p), axis=0)
            print(center)
            cv2.circle(color_image, (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)


    cv2.namedWindow('Aligned', cv2.WINDOW_NORMAL)
    cv2.imshow('Aligned', color_image)
    key = cv2.waitKey()
    if key == 27:
        cv2.destroyAllWindows()




