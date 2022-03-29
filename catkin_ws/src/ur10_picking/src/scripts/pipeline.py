#!/usr/bin/env python

import copy
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
from ur10_picking.msg import PoseMessage
from ur10_picking.srv import *

import tf2_ros
import tf2_geometry_msgs


class State:
    """
    Define statemachine class
    Each state will be initiated using this class.
    The class includes state name, on_event, next_state
    """

    def __init__(self):
        print("Initiating state:", str(self))

    def run(self, pipeline_core):
        assert 0, "run not implemented"

    def on_event(self, event):
        assert 0, "on_event not implemented"

    def next_state(self, state_complete):
        assert 0, "next state not implemented"


class Initialise(State):
    """
    Class definition for the initialisation state
        Check nodes are on and topics are publishing
        Prioritisation - list of lists output
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Initialising the system and prioritising items")
        # TODO: Test node communications
        # Do item prioritisation program here (NOT FOR DEMO)

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Initialisation complete")
            return 2  # Move to Calibration
        else:
            return 1  # Stay in Initialise


class Calibrate(State):
    """
    Class definition for the calibration state
        Output - binary status of calibration = 1
        Output - Home position - hard coded - move to home
        Output - Bin position - xyz
        Output - Centroids of each shelf - list or dict of coordinates / Pose for shelf home
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Beginning calibration process")
        # TODO: do we need vision calibration or UR10 calibration?
        # TODO: Do vision system calibration

        # (NOT FOR DEMO) Do vacuum calibration
        # (NOT FOR DEMO) Do robot arm calibration - position relative to shelf
        # (NOT FOR DEMO) Do any other calibration

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Calibration complete")
            return 3  # Move to FindShelf
        else:
            return 2  # Stay in Initialise


class FindShelf(State):
    """
    Class definition for the FindShelf state
        Read item from the prioritised list
        Identify shelf reference
        Identify retrieval mechanism
        Move UR10 to shelf centre and conform position has been reached
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Moving to shelf")
        
        try:
            # Transform from camera pose to EE pose
            transform = pipeline_core.tf_buffer.lookup_transform('camera', 'ee_link',  rospy.Time())
        
            # Desired camera pose in world frame
            shelf_assess_pose_stamped = PoseStamped()
            shelf_assess_pose_stamped.stamp = rospy.Time.now()
            shelf_assess_pose_stamped.position.x = 0.05
            shelf_assess_pose_stamped.position.y = 0.5
            shelf_assess_pose_stamped.position.z = 0.85
            shelf.assess_pose_stamped.orientation.x = 0.7071
            shelf_assess_pose_stamped.orientation.y = 0.7071
            shelf_assess_pose_stamped.orientation.z = 0
            shelf_assess_pose_stamped.orientation.w = 0

            pose_transformed = tf2_geometry_msgs.do_transform_pose(shelf_assess_pose_stamped, transform)
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Failed!")
            rospy.sleep(1.0)


        '''
        # Shelf E home
        pose_msg = PoseMessage()
        shelf_centre_pose = Pose()
        shelf_centre_pose.position.x = 0.05
        shelf_centre_pose.position.y = 0.5
        shelf_centre_pose.position.z = 0.42
        shelf_centre_pose.orientation.x = 0.7071
        shelf_centre_pose.orientation.y = 0.7071
        shelf_centre_pose.orientation.z = 0
        shelf_centre_pose.orientation.w = 0

        pose_msg.pose = shelf_centre_pose
        pose_msg.incremental = False

        # Turn on vacuum
	pipeline_core.vacuumonoff.call(1)

        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(10.0)
        
        # Shelf E pick
        pose_msg = PoseMessage()
        shelf_centre_pose = Pose()
        shelf_centre_pose.position.x = 0.05
        shelf_centre_pose.position.y = 0.65
        shelf_centre_pose.position.z = 0.42
        shelf_centre_pose.orientation.x = 0.7071
        shelf_centre_pose.orientation.y = 0.7071
        shelf_centre_pose.orientation.z = 0
        shelf_centre_pose.orientation.w = 0

        pose_msg.pose = shelf_centre_pose
        pose_msg.incremental = False

        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(10.0)	

        # Shelf E home
        pose_msg = PoseMessage()
        shelf_centre_pose = Pose()
        shelf_centre_pose.position.x = 0.05
        shelf_centre_pose.position.y = 0.5
        shelf_centre_pose.position.z = 0.42
        shelf_centre_pose.orientation.x = 0.7071
        shelf_centre_pose.orientation.y = 0.7071
        shelf_centre_pose.orientation.z = 0
        shelf_centre_pose.orientation.w = 0
        
        pose_msg.pose = shelf_centre_pose
        pose_msg.incremental = False

        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(10.0)
        '''
	# Turn off vacuum
	pipeline_core.vacuumonoff.call(0)

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Shelf found")
            return 4  # Move to AssessShelf
        else:
            return 3  # Stay in FindShelf


class AssessShelf(State):
    """
    Class definition for assesing the shelf using the vision system
        Use vision node to extract the centroid of the item
        Create UR10 trajectory for target extraction of item
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Assessing shelf")
        # TODO: Assess shelf - use vision node topic to extract the centroid of the item
        # TODO: Print centroid of the item to terminal

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):
        # Move to next state when there is a confirmed centroid for the item
        if state_complete:
            print("Assessment complete")
            return 5  # End
        else:
            return 4  # Stay in AssessShelf


class SpinState(State):

    def run(self, pipeline_core):

        print("Listening for transform...")

        try:
            transform = pipeline_core.tf_buffer.lookup_transform('world', 'camera', rospy.Time())
            
            sample_pose_stamped = PoseStamped()
            sample_pose_stamped.header.stamp = rospy.Time.now()
            sample_pose_stamped.pose.orientation.w = 1.0
            pose_transformed = tf2_geometry_msgs.do_transform_pose(sample_pose_stamped, transform)
            print(pose_transformed.pose)
            print("***************************")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityExecption, tf2_ros.ExtrapolationException):
            print("Not found!")
            rospy.sleep(1.0)

        return self.next_state(False)
            

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Finished spinning")
            return 5
        else:
            return 999  # Stay in SpinState


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
        self.var = data
        print(data)

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


class StateSupervisor:
    """
    Governs transitions between states and acts as scope for all state objects
    """

    def __init__(self):

        self.status = 999
        self.state_initialise = Initialise()
        self.state_calibrate = Calibrate()
        self.state_find_shelf = FindShelf()
        self.state_assess_shelf = AssessShelf()
        self.state_spin = SpinState()

        print("State machine started. Current status:", self.status)

    def run(self, pipeline_core):
        """
        Monitors status and runs relevant states
        :param pipeline_core: PipelineCore object
        :return: None
        """

        if self.status == 0:
            self.status = self.state_initialise.run(pipeline_core)
        if self.status == 1:
            self.status = self.state_calibrate.run(pipeline_core)
        if self.status == 2:
            self.status = self.state_find_shelf.run(pipeline_core)
        if self.status == 3:
            self.status = self.state_assess_shelf.run(pipeline_core)
        if self.status == 4:
            print("Demo completed")
        if self.status == 999:
            self.status = self.state_spin.run(pipeline_core)

    def report_status(self):
        print("Current state: ", self.status)


class PipelineCore:
    """
    Handles all persistent data and ROS interfaces
    """

    def __init__(self):
        # Initiate Pipeline Node:
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)

        # Initiate topics and services:
        # UR10 control:
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Vision topics and services:
        # TODO: add the vision topics and services

        # Vacuum control (commented out for UR10 testing purposes):
        self.vacuumonoff = ServiceCaller("vacuum_switch", vacuum_switch)
        self.vacuumcalibration = ServiceCaller("vacuum_calibration", vacuum_calibration)
        self.vacuumsucking = TopicReader("vacuum_pressure", Bool)

        # Gripper topics and services:
        # (NOT FOR DEMO) Add the gripper topics and services (limit switches arduino node)


def run_pipeline():
    """
    Instantiates pipeline_core and state_machine objects, and then just spins the state machine
    :return: None
    """
    pipeline_core = PipelineCore()
    print("Waiting for moveit interface to start...")
    while not pipeline_core.pose_feedback_subscriber.var:
        rospy.sleep(0.1)
    print("Moveit interface started. Starting state machine...")
    state_machine = StateSupervisor()
    while True:
        state_machine.run(pipeline_core)
        state_machine.report_status()
        rospy.sleep(5.0)


if __name__ == "__main__":
    run_pipeline()
