import profile
import cv2
import numpy as np
import pyrealsense2 as rs

def draw_depth_markers(frame, corners, ids):
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

            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

            depth = round(aligned_depth_frame.get_distance(cX, cY),3)

            depth_intrinsic = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            result = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [cX, cY], depth)
            result_string = "X: %f, Y: %f, Z: %f" % (result[0], result[1], result[2]) 

            cv2.putText(frame, result_string,
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
            
    return frame

def draw_markers(frame, corners, ids):
    if len(corners) > 0:
		# flatten the ArUco IDs list
        ids = ids.flatten()

		# loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

			# convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

			# compute and draw the center (x, y)-coordinates of the
			# ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

			# draw the ArUco marker ID on the frame
            cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
    return frame

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Using depth scale of ", depth_scale)

clipping_distance_in_meters = 5
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParmas = cv2.aruco.DetectorParameters_create()


try:
    while True:
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()

        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        #ARCO DETECTION
        (corners, ids, rejected) = cv2.aruco.detectMarkers(bg_removed, arucoDict, parameters=arucoParmas)
        aruco_image = draw_markers(bg_removed,corners, ids)
        arcu_depth_image = draw_depth_markers(depth_colormap, corners, ids)


        images = np.hstack((aruco_image, arcu_depth_image))
        cv2.namedWindow('Aligned', cv2.WINDOW_NORMAL)
        cv2.imshow('Aligned', images)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
