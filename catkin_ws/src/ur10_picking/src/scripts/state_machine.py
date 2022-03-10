#!/usr/bin/env python
"""
State machine for the FARSCOPE group project - Amazon Picking Challenge

This script acts as the master controller for the whole system.

Version: v2
Date: 8 March 2022
Updates:
    - Added a Pipeline() class which describes the main loop through which the code will move
    - Added to the Pipeline() class: initiate topics, services, pose functions and the main pipeline function
    - Rearranged to code to incorporate Jamees' UR10 control code
    - Updated descriptions and comments for clearer reading
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from gemoetry_msgs.msg import Pose, PoseArray
from ur10_picking.srv import *
from ur10_picking.msg import PoseMessage


"""
-----------DEFINE TOPICS AND SERVICES----------
"""


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


class TopicCaller:
    """
    Define topicreader class to read data from published ROS topics
    This class processes data from a ROS topic and returns it

    :attribute: readname: name of the topic to subscribe and read from. Default to "none" if publishing only
    :attribute: writename: name of the topic to publish and write to. Default to "none" if subscribing only
    :attribute: read_data_class: data
    """

    def __init__(self, type, read_data_type=String, write_data_type=String, readname="none", writename="none"):
        self.var = None
        self.readname = readname
        self.writename = writename
        self.read_data_class = read_data_type
        self.write_data_class = write_data_type
        if type == 'read' or type == 'readwrite':
            rospy.Subscriber(self.readname, self.read_data_class, self.callback)
            rospy.sleep(1)
            rospy.loginfo("Reading topic {}...the following data has been read: {}".format(topicname, self.var))
        if type == 'write' or type == 'readwrite':
            self.pub = rospy.Publisher(writename, self.write_data_class, queue_size=10)

    def callback(self, data):
        """
        Callback function - this is called automatically from the readtopic() function below
        Sets the self.var variable to the data from the topic

        :param data: input autofilled through the rospy.Subscriber function in the readtopic() function below
        """
        self.var = data.data

    def readtopic(self):
        """
        :return: returns the data read from the topic
        """
        rospy.Subscriber(self.topicname, self.data_class, self.callback)
        rospy.sleep(1)

        return self.var

    def writetopic(self, message):
        """
        Write to the topic that has been created
        :param message: Input data to be published to this topic
        """
        self.pub.publish(message)


"""
----------DEFINE STATES FOR STATE MACHINE----------
"""


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


"""
----------DEFINE PIPELINE CLASS----------
"""


class Pipeline():
    def __init__(self):
        # Initiate Pipeline Node:
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)

        # Initiate topics and services:
        # UR10 control:=
        self.pose_talker = TopicCaller(type="write", write_date_type=PoseMessage, writename='/pipeline/next_cartesian_pose')
        self.trajectory = TopicCaller(type="write", write_data_type=PoseArray, writename='/pipeline/cartesian_trajectory')
        # Vision topics and services:

        # Vacuum control:
        self.vacuumonoff = ServiceCaller("vacuum_switch", vacuum_switch)
        self.vacuumcalibration = ServiceCaller("vacuum_calibration", vacuum_calibration)
        self.vacuumsucking = TopicCaller(type="read", read_data_type=Bool, readname="vacuum_pressure")

        # Initiate states
        initialisation_state = Initialise()
        calibration_state = Calibration()
        findshelf_state = FindShelf()

    # UR10 Control Functions:
    def log_pose(self, pose):
        self.current_pose = pose
        print_callback(pose)

    # State Machine Functions:
    def run_state_machine(self, state_condition):
        # Update the state condition as per the state machine documentation to move through the different states
        ### TEST TEST TEST ###
        self.state_condition = state_condition
        self.vacuumcalibration.call(1)

    def state_machine_status(self):
        # Provide a status reading for the state machine based on state_condition
        ### TEST TEST TEST ###
        return self.state_condition


"""
-----------MAIN LOOP----------
"""


if __name__ == "__main__":
    pipeline = Pipeline()
    rospy.sleep(30.0)  # To allow robot to home before sending start pose

    # SOME TESTING SOME TESTING SOME TESTING

    # Test service caller with vacuum node
    for i in range(5):
        pipeline.vacuumsucking.call(1)
        pipeline.vacuumsucking.call(0)

    # Test vacuum sucking topic with vacuum node

    # Test vacuum calibration with vacuum node

    # Test Gazebo arm movement




