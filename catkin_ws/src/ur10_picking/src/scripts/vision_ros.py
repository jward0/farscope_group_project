from ur10_picking.srv import detect_markers
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
        self.enable_camera = False

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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable_camera:
                pass
            else:
                pass

            rate.sleep()  

    

def handle_detect_markers(req):
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
            mark.point = Point(result[0],result[1],result[2])
            markers.append(mark)

    toReturn = arucoMarkerArray()
    toReturn.markers = markers

    return toReturn


def init_ros():
    rospy.init_node('camera_server')
    s = rospy.Service('detect_markers', detect_markers, handle_detect_markers)
    print("Ready for camera")


if __name__ == "__main__":
    global vision_core
    init_ros()

    vision_core = Vision_Core()
    vision_core.start()