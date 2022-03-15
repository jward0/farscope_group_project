#!/usr/bin/env python

import copy
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray
from ur10_picking.msg import PoseMessage
from ur10_picking.srv import *

# Define a global variable for the state machine status:
state_machine_status = 0

class State:
    """
    Define statemachine class
    Each state will be initiated using this class.
    The class includes state name, on_event, next_state
    """

    def __init__(self, pipeline_core):
        self.pipeline_core = pipeline_core
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
        Check nodes are on and topics are publishing
        Prioritisation - list of lists output
    """

    def run(self):
        print("Initialising the system and prioritising items")
        # TODO: Test node communications

        # Do item prioritisation program here (NOT FOR DEMO)

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
        Output - Bin position - xyz
        Output - Centroids of each shelf - list or dict of coordinates / Pose for shelf home
    """

    def run(self):
        print("Beginning calibration process")
        # NOTE FOR DEMO - no calibration has been setup
        # TODO: do we need vision calibration or UR10 calibration?
        # TODO: Do vision system calibration

        # (NOT FOR DEMO) Do vacuum calibration
        # (NOT FOR DEMO) Do robot arm calibration - position relative to shelf
        # (NOT FOR DEMO) Do any other calibration
        print("Calibration complete")

        # TODO: Move UR10 to home position (HARDCODED FOR DEMO) & confirm position has been reached

        self.next_state()

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self):
        state_machine_status = 2


class FindShelf(State):
    """
    Class definition for the FindShelf state
        Read item from the prioritised list
        Identify shelf reference
        Identify retrieval mechanism
        Move UR10 to shelf centre and conform position has been reached
    """

    def run(self):
        print("Moving to shelf")
        # (NOT FOR DEMO) Read item from prioritised list
        # (NOT FOR DEMO) Identify shelf reference
        # TODO: Move UR10 to shelf centre (HARDCODED FOR DEMO) & confirm position has been reached

        self.next_state()

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self):
        state_machine_status = 3


class AssessShelf(State):
    """
    Class definition for assesing the shelf using the vision system
        Use vision node to extract the centroid of the item
        Create UR10 trajectory for target extraction of item
    """

    def run(self):
        # TODO: Assess shelf - use vision node topic to extract the centroid of the item
        # TODO: Print centroid of the item to terminal

        self.next_state()

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self):
        state_machine_status = 4


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
    """
        Define topicreaderclass to read data from published ROS topics
        This class processes data from a ROS topic and returns it
        :attribute: topic_name - string name of the topic to which data will be published
        :attribute: data_class - data type / ROS data class for the published data
    """

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
    Define topicwriterclass to read data from published ROS topics
    This class processes data from a ROS topic and returns it
    :attribute: topic_name - string name of the topic to which data will be published
    :attribute: data_class - data type / ROS data class for the published data
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
        # Initiate Pipeline Node:
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)

        # Initiate other nodes using the launch file
        # TODO: add the launch file here OR just use ROSLAUNCH in the command line prior to running this script
        # TODO: make sure launch file dependencies are specified as required

        # Initiate topics and services:
        # UR10 control:
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)

        # Vision topics and services:
        # TODO: add the vision topics and services

        # Vacuum control:
        self.vacuumonoff = ServiceCaller("vacuum_switch", vacuum_switch)
        self.vacuumcalibration = ServiceCaller("vacuum_calibration", vacuum_calibration)
        self.vacuumsucking = TopicReader("vacuum_pressure", Bool)

        # Gripper topics and services:
        # (NOT FOR DEMO) Add the gripper topics and services (limit switches arduino node)


class RunStateSupervisor():
    """
    Define state machine supervisor class to manage the transfer from one state from another and output the state machine status
    :attribute: pipeline_core - input the pipeline_core class to define all ROS communications
    """
    def __init__(self):
        # Initiate states
        self.initialisation_state = Initialise()
        self.calibration_state = Calibration()
        self.findshelf_state = FindShelf()
        self.assessshelf_state = AssessShelf()

        # Confirm state machine has started
        print("State machine has started. State machine is no in statea number: ", state_machine_status)

    def run(self):
        """
        Basic run of state machine for the purpose of the Demo
        Note that the actual state machine for the final solution will use a binary state_machine_status
        """
        if state_machine_status == 0:
            self.initialisation_state.run()
        if state_machine_status == 1:
            self.calibration_state.run()
        if state_machine_status == 2:
            self.findshelf_state.run()
        if state_machine_status == 3:
            self.assessshelf_state.run()
        if state_machine_status == 4:
            print("Demo completed :)")

    def report_status(self):
        # TO DO: add code here to decode the state_machine_status and report on the status of the state machine
        # This will be integrated as based on the binary state_machine_status system that is yet to be implemented
        print("The state machine is now in state: ", state_machine_status)


def run_pipeline():
    """
    Function to initiate and run the pipeline including state machine supervisor
    """
    pipeline_core = PipelineCore
    state_machine = RunStateSupervisor
    while True:
        state_machine.run()
        rospy.spin()


if __name__ == "__main__":
    run_pipeline()

