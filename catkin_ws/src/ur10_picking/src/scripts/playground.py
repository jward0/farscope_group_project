from mimetypes import init
import cv2
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans

from treelib import Node, Tree

class Quad(object):
    def __init__(self, indexX, indexY, data):
        self.indexX = indexX
        self.indexY = indexY
        self.data = data
        self.colour = None

def check_homogeneity(image):
    max_value = np.amax(image)
    min_value = np.amin(image)
    #print("max_dif: %f" % (max_value-min_value))
    if max_value-min_value >= 1:
        return False
    else:
        return True 

def process_nodes(tree):
    for node in tree.all_nodes():
        if node.is_leaf():
            if not check_homogeneity(node.data.data):
                image_data = node.data.data
                hsplit = np.array_split(image_data, 2, 1)
                for x in range(len(hsplit)):
                    vsplit = np.array_split(hsplit[x], 2, 0)
                    for y in range(len(vsplit)):
                        quad = vsplit[y]
                        if(quad.shape[0] > 1 and quad.shape[1] > 1):
                            tree.create_node(parent=node, data=Quad(x,y, quad))
                            tree = process_nodes(tree)
            else:
                return tree
    
    return tree
            
            
            

def segmentateImage(depth_image, depth_scale):
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    depth_image_3d.fill(0)
    print(depth_image)
    tree = Tree()
    tree.create_node("0",0, data=Quad(0,0,depth_image))
    tree = process_nodes(tree)
    tree.show()

    for node in tree.leaves():
        print("%f %f" % (node.data.indexX, node.data.indexY))



    return depth_image_3d

def kmean(depth_colormap, color_image):
    twoDiamge = depth_colormap.reshape((-1,3))
    twoDiamge = np.float32(twoDiamge)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,10,1.0)
    K = 3
    attempts = 10
    ret, label, center= cv2.kmeans(twoDiamge, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    result_image = res.reshape((depth_colormap.shape))

    low_white = np.array([80,101,110])
    high_white = np.array([184, 180, 172])

    #mask = cv2.inRange(color_image, low_white, low_white)
    #mask = 255-mask
    
    for c in center:
        mask = cv2.inRange(result_image, c, c)
        contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        print("Found %f " %len(contours))
        if contours:
            (x_min, y_min, box_width, box_height) = cv2.boundingRect(contours[0])
            cv2.rectangle(color_image, (x_min - 15, y_min -15),
                    (x_min + box_width + 15, y_min + box_height + 15),
                        (0,int(c[0]),0), 4)


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
            markerCenters[markerID] = [cX, cY]
            cv2.circle(color, (cX, cY), 4, (0, 0, 255), -1)
            
        if(len(markerCenters) >= 4):
            pts = np.array([markerCenters[0],markerCenters[1],markerCenters[3],markerCenters[2]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(mask,[pts],(255,255,255))
            mask_gray=cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            color = cv2.bitwise_and(color,color,mask = mask_gray)
            depth = np.bitwise_and(depth, mask_gray)
        
    return color, depth

def findObjects(img, amount):
    objects = []
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    #cv2.drawContours(image=shelf_image, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    if len(contours) > amount:
        for box in range(1, amount+1):
            (x_min, y_min, box_width, box_height) = cv2.boundingRect(contours[box])
            objects.append([x_min, y_min, box_width, box_height])

    
    return objects

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

def checkORB(input, amount):
    points = []
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(input, None)
    pts = cv2.KeyPoint_convert(kp)

    if(len(pts) > 10):
        kmeans = KMeans(n_clusters=(amount+4), init='k-means++', max_iter=300, n_init=10, random_state=0)
        pred_y = kmeans.fit_predict(pts)
        points = kmeans.cluster_centers_

    return points


def findBlob(input, color_image):
    params = cv2.SimpleBlobDetector_Params()

    params.filterByColor = True
    params.blobColor = 0

    params.filterByArea = True
    params.minArea = 100

    detector = cv2.SimpleBlobDetector_create(params)
    input_gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    keypoints = detector.detect(input_gray)
    print(keypoints)
    color_image = cv2.drawKeypoints(color_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return color_image






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

        depth_image[depth_image > 3000] = 0
        #returnedImage = segmentateImage(depth_image, depth_scale)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_HSV)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParmas)
        shelf_image, shelf_image_depth = get_shelf(color_image, depth_image,corners, ids)
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(shelf_image_depth, alpha=0.1), cv2.COLORMAP_HSV)

        objects_depth = findDepthObjects(shelf_image_depth, 2)
        for ob in objects_depth:
            cv2.rectangle(color_image, (ob[0] - 15, ob[1] -15),
                (ob[0] + ob[2] + 15, ob[1] + ob[3] + 15),
                (0,255,0), 4)

        objects = findObjects(shelf_image,2)
        for ob in objects:
            cv2.rectangle(color_image, (ob[0] - 15, ob[1] -15),
                (ob[0] + ob[2] + 15, ob[1] + ob[3] + 15),
                (0,0,255), 4)
        
        orb_points = checkORB(shelf_image, 2)
        for point in orb_points:
            cv2.circle(color_image,tuple(point),4,(255,0,0), -1)


        cv2.imshow('Aligned', color_image)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
