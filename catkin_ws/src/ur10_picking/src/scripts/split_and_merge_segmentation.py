import cv2
import numpy as np
import pyrealsense2 as rs

from sklearn.cluster import KMeans

from treelib import Tree


class Quad(object):
    """
    Contains information of homogeneous regions in image segmentation- designed to be used in a tree with
    below functions

    :attribute x_index: int
        0 or 1, quadrant x index of parent Quad in tree
    :attribute y_index: int
        0 or 1, quadrant y index of parent Quad in tree
    :attribute owned_space: [int, int, int, int]
        [x0, x1, y0, y1] (probably). Pixel coordinates that this Quad owns. Subset of owned_space
        of parent Quad in a tree structure.
    :attribute data: np.array of ints
        Greyscale image data located in owned space
    :attribute color: float
        Mean greyscale value of owned space
    """
    def __init__(self, x_index, y_index, owned_space, data):

        self.x_index = x_index
        self.y_index = y_index
        self.owned_space = owned_space
        self.data = data
        self.color = np.mean(data)


def check_homogeneity(node_data):
    """
    Checks image data of Quad object is within some homogeneity tolerance
    Currently just works on standard deviation

    :param node_data: Quad
        Quad containing image data to examine
    :return: bool
        True if homogeneity criteria satisfied, otherwise false
    """
    image = node_data.data

    if np.std(image.flatten()) > 40:
        return False
    else:
        return True


def explode_tree(tree):
    """
    Iteratively expands a tree of quads to locate homogeneous regions
    Every iteration, all leaves on the tree that do not satisfy homogeneity criteria will explode
    When all leaves satisfy criteria, iteration ends
    :param tree: treelib.Tree of Quads
    :return: treelib.Tree of Quads
    """

    finished_exploding = True

    for node in tree.leaves():
        if not check_homogeneity(node.data):
            # Split leaf 4 ways if not homogeneous

            finished_exploding = False
            image_data = node.data.data

            node_dimensions = [node.data.owned_space[1] - node.data.owned_space[0] + 1,
                               node.data.owned_space[3] - node.data.owned_space[2] + 1]

            hsplit = np.array_split(image_data, 2, 1)
            for x in range(len(hsplit)):
                vsplit = np.array_split(hsplit[x], 2, 0)
                for y in range(len(vsplit)):
                    split_image = vsplit[y]

                    new_owned_space = np.array(
                                        [node.data.owned_space[0] + x*node_dimensions[0]/2,
                                         node.data.owned_space[0] + (1+x)*node_dimensions[0]/2 - 1,
                                         node.data.owned_space[2] + y*node_dimensions[1]/2,
                                         node.data.owned_space[2] + (1+y)*node_dimensions[1]/2 - 1]
                                               ).astype(int)

                    if split_image.shape[0] > 1 and split_image.shape[1] > 1:

                        new_node_data = Quad(x, y, new_owned_space, split_image)
                        tree.create_node(parent=node, data=new_node_data)

    if not finished_exploding:
        tree = explode_tree(tree)

    return tree


def generate_image_from_tree(tree):
    """
    Takes populated quad tree and stitches leaves together to form an array that can be read
    as a greyscale image. There's a transpose involved for reasons I'm not going to sort right now.

    :param tree: treelib.Tree of Quads
    :return: np.array(shape=(640, 480))
        Greyscale image formed from tree
    """

    combined_image = np.zeros(shape=(480, 640))

    for node in tree.leaves():
        combined_image[node.data.owned_space[0]:node.data.owned_space[1]+1,
                       node.data.owned_space[2]:node.data.owned_space[3]+1] = node.data.color
        
    return np.transpose(np.array(combined_image, dtype=np.uint8))


def segment_greyscale_image(img):
    """
    Generates tree of Quads from greyscale image. Pixel dimensions currently hardcoded for no good reason.
    :param img: np.array(shape=(480, 640) or shape=(640, 480) (not sure, see comment about transpose above)) of ints
        greyscale image
    :return: np.array(shape=(640, 480)) of ints
        greyscale image post-split and merge segmentation
    """

    tree = Tree()
    tree.create_node("0", 0, data=Quad(0, 0, [0, 479, 0, 639], img))
    tree = explode_tree(tree)
    tree.show()

    return generate_image_from_tree(tree)


if __name__ == "__main__":

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

            depth_image[depth_image > 750] = 0

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_HSV)[:, :, 2]
            dst = cv2.GaussianBlur(depth_colormap, (25, 25), cv2.BORDER_DEFAULT)

            segmented_image = segment_greyscale_image(dst)

            cv2.imshow('Depth', segmented_image)
            key = cv2.waitKey(1)
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
