#!/usr/bin/env python

import copy
import rospy
from geometry_msgs.msg import Pose, PoseArray
from ur10_picking.msg import PoseMessage


class State:
    """
    Define statemachine class
    Each state will be initiated using this class.
    The class includes state name, on_event, next_state
    """

    def __init__(self):
        print("Initiating state:", str(self))

    def run(self):
        assert 0, "run not implemented"

    def on_event(self, event):
        assert 0, "on_event not implemented"

    def next_state(self, input):
        assert 0, "next state not implemented"


class Initialise(State):
    """
    Class defintion for the initialisation state
    Run launch file
    Turn on and check nodes are on and topics are publishing
    Prioritisation - list of lists output
    """

    def run(self):
        print("Initialising the system and prioritising items")
        # Do item initiation program here
        # Do item prioritisation program here

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, input):
        if input == "Initialisation complete":
            return True
        else:
             return False


class Calibration(State):
    """
    Class definition for the calibration state
    Output - binary status of calibration = 1
    Output - Home position - hard coded - move to home
    output - Bin position - xyz
    Output - Centroids of each shelf - list or dict of coordinates / Pose for shelf home
    """

    def run(self):
        print("Beginning calibration process")
        # Do vision system calibration
        # Do vacuum calibration
        Pipeline.vacuumcalibration.call("begin vacuum calibration")
        # Do robot arm calibration - position relative to shelf
        # Do any other calibration
        print("Calibration complete")

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, input):
        if input == "Calibration complete":

            return True
        else:
            return False


class FindShelf(State):
    """
    Class definition for the FindShelf state
    """

    def run(self):
        print("Moving to shelf")
        # Read item from prioritised list
        # Identify shelf reference
        self.shelf_centre = 0
        # Move to shelf centre

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, input):
        coordinates = input
        if coordinates == self.shelf_centre:
            return True
        else:
            return False


class AssessShelf(State):
    """
    Class definition for assesing the shelf using the vision system
    """

    def run(self):
        # Assess shelf - use vision node topic to extract the centroid of the item

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, input):
        # Move to next state when there is a confirmed centroid for the item


class ServiceCaller:
    """
    Define service class
    This class is defined for each ROS service interaction
    The class can send a request to a service and process a response from a service
    :attribute servicename: string
        string name for the service as defined in the service definition
    :attribute service_class: class
        service class as defined in the service definition and imported from the relevent package
    """

    def __init__(self, servicename, service_class):
        self.servicename = servicename
        self.service_class = service_class
        rospy.wait_for_service(servicename)
        # rospy.loginfo("Connected to service:", servicename)

    def call(self, req):
        """
        Sends a service request to the service attributed with this class
        :param req: request to be sent to the class. The data type for this varies based on the service definition
        :return: returns the response from the service
        """
        rospy.wait_for_service(self.servicename)
        client = rospy.ServiceProxy(self.servicename, self.service_class)
        response = client(req)
        return response


class TopicReader:
    def __init__(self, topic_name, data_class):
        self.var = None
        self.topic_name = topic_name
        self.data_class = data_class
        rospy.Subscriber(self.topic_name, self.data_class, self.callback)
        rospy.sleep(1)
        rospy.loginfo("Reading topic {}...the following data has been read: {}".format(topic_name, self.var))

    def callback(self, data):
        """
        Callback function - this is called automatically from the readtopic() function below
        Sets the self.var variable to the data from the topic
        :param data: input autofilled through the rospy.Subscriber function in the readtopic() function below
        """
        self.var = data.data

    def read_topic(self):
        """
        :return: returns the data read from the topic
        """
        rospy.Subscriber(self.topic_name, self.data_class, self.callback)
        rospy.sleep(1)

        return self.var


class TopicWriter:
    """
    Define topicreader class to read data from published ROS topics
    This class processes data from a ROS topic and returns it
    :attribute: readname: name of the topic to subscribe and read from. Default to "none" if publishing only
    :attribute: writename: name of the topic to publish and write to. Default to "none" if subscribing only
    :attribute: read_data_class: data
    """

    def __init__(self, topic_name, data_class):
        self.var = None
        self.topic_name = topic_name
        self.data_class = data_class
        self.pub = rospy.Publisher(topic_name, self.data_class, queue_size=10)

    def write_topic(self, message):
        """
        Write to the topic that has been created
        :param message: Input data to be published to this topic
        """
        self.pub.publish(message)


class PipelineCore:

    def __init__(self):
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)


def run_pipeline():

    pipeline_core = PipelineCore


if __name__ == "__main__":
    run_pipeline()
