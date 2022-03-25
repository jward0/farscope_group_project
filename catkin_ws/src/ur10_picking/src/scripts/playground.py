from mimetypes import init
import cv2
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt


def get_shelf(color, depth, corners, ids):
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


def findDepthObjects(img, amount):
    objects = []
    img = cv2.convertScaleAbs(img)
    dst = cv2.GaussianBlur(img,(5,5),cv2.BORDER_DEFAULT)
    edges = cv2.Canny(image=dst, threshold1=70, threshold2=200) # Canny
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
    dilated = cv2.dilate(edges, kernel)
    
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    #cv2.drawContours(color, contours, -1, (0, 255, 0), 1) #---set the last parameter to -1
    for b in range(1,amount+1):
        box = contours[b]
        (x_min, y_min, box_width, box_height) = cv2.boundingRect(box)
        objects.append([x_min, y_min, box_width, box_height])
    return objects


def findContours(img, amount):
    objects = []
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(image=img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contours = contours[:amount]
    toDraw = []

    return contours

def maskShelf(color_img):
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

    mask_background = cv2.inRange(hsv,(15,35,100),(180,255,255))

    kernel = np.ones((5,5),np.uint8)          
    mask_background = cv2.morphologyEx(mask_background, cv2.MORPH_CLOSE, kernel)

    color_img = cv2.bitwise_and(color_img,color_img,mask =  mask_background)

    return color_img

def getDepthOfRect(depth, rect):
    x,y,w,h = rect
    depth = depth[int(x+w/2),int(y+h/2)].astype(float)
    depth = depth * depth_scale
    return depth

def getDepth(point, depth, aligned_depth_frame):
    depth_intrinsic = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    result = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [point[0], point[1]], depth)
    result_string = "X: %f, Y: %f, Z: %f" % (result[0], result[2], result[1]) 

    return result_string


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
            exit()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_image = cv2.flip(depth_image, -1)
        color_image = cv2.flip(color_image, -1)

        depth_image[depth_image > 3000] = 0
        #returnedImage = segmentateImage(depth_image, depth_scale)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_HSV)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParmas)
        shelf_image, shelf_image_depth = get_shelf(color_image, depth_image,corners, ids)
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(shelf_image_depth, alpha=0.1), cv2.COLORMAP_HSV)

        maskedImage = maskShelf(shelf_image)

        contours = findContours(maskedImage,1)
        cv2.drawContours(image=maskedImage, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)


        for c in contours:
            rect = cv2.boundingRect(c)
            x,y,w,h = rect
            depthLevel = getDepthOfRect(depth_image,rect)
            comx, comy = c.mean(axis=0)[0]

            coords = getDepth([comx, comy], depthLevel, aligned_depth_frame)
            cv2.rectangle(color_image, rect, (0,0,255),4)
            cv2.putText(color_image, coords, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.circle(color_image, (int(comx),int(comy)), 4, (0,255,0), -1)
            #cv2.circle(color_image, (int(x+w/2), int(y+h/2)), 4, (0, 255, 0), -1)

            
        com = np.hstack((color_image, maskedImage))
        com = np.hstack((com, depth_colormap))
        cv2.imshow('Aligned', com)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
