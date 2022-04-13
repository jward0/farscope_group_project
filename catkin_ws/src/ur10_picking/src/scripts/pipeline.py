#!/usr/bin/env python

import copy
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Vector3
from roscomm import ServiceCaller, TopicReader, TopicWriter
from std_msgs.msg import String
from std_msgs.msg import Bool
from ur10_picking.msg import PoseMessage
from ur10_picking.srv import *
from prioritise import *

import tf2_ros
import tf2_geometry_msgs
import tf
from tf import transformations as ts


class State:
    """
    Define statemachine class
    Each state will be initiated using this class.
    The class includes state name, on_event, next_state
    :param: pipeline_core: pipeline_core class containing all pipeline data needed for states
    :param: event: event causing change to the state
    :param: state_status: binary status number to used to update state machine status
    """

    def __init__(self):
        print("Initiating state:", str(self))
        self.state_status = 0b00000000000000

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

        # TODO: Check arm has iniitiated and is in shelf home

        # TODO: Check prioritised item list has been populated
        # Function imported from priortise.py
        json_object = import_json('apc_pick_test.json')
        pipeline_core.bin_contents = json_object["bin_contents"]
        pipeline_core.work_order = json_object["work_order"]
        pipeline_core.work_order_prioritised = prioritise_items(bin_contents, work_order)

        # TODO: Test node communications

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Initialisation complete")
            return 0b10010000000000  # Move to Calibration
        else:
            return 0b00000000000000  # Stay in Initialise


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
            return 0b01011000000000  # Move to FindShelf
        else:
            return 0b10010000000000  # Stay in Calibration


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

        pipeline_core.target_shelf = pipeline_core.work_order_prioritised[0]["bin"]

        print("Moving to shelf")
        
        # Shelf E assess home
        pose_msg = PoseMessage()
        shelf_centre_pose = Pose()
        shelf_centre_pose.position.x = 0
        shelf_centre_pose.position.y = 0.4
        shelf_centre_pose.position.z = 0.65
        shelf_centre_pose.orientation.x = 0.6964
        shelf_centre_pose.orientation.y = 0.6964
        shelf_centre_pose.orientation.z = -0.1227
        shelf_centre_pose.orientation.w = 0.1227

        # Go to shelf
        pose_msg.pose = shelf_centre_pose
        pose_msg.incremental = False
        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(10.0)

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Shelf found")
            return 0b01011000000000  # Move to AssessShelf
        else:
            return 0b01011000000000  # Stay in FindShelf


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
        object_xyz = pipeline_core.get_object_centroid.call("bin_E")

        transform = pipeline_core.tf_buffer.lookup_transform('world', 'camera', rospy.Time())
        object_pose = PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.pose.position = object_xyz.object

        target_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)
        target_pose.pose.orientation.x = 0.7071
        target_pose.pose.orientation.y = 0.7071
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0

        print("Grip pose in world frame:")
        print(target_pose)

        pipeline_core.stored_pose = target_pose.pose

        state_complete = "Item found"
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):
        # Move to next state when there is a confirmed centroid for the item
        if state_complete == "Item found":
            print("Assessment complete - item found")
            return 0b01011110000000  # End
        if state_complete == "Item not found":
            print("Assessment complete - item not found")
            return 0b01011100000000  # Go to work order management to update priority list
        if state_complete == "Shelf not found":
            print("Shelf not found")
            return 0b01010000000000  # Rerun calibration


class DoGrip(State):
    """
    Do grip
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """

        print("Attempting grip")
        pick_home_pose_msg = PoseMessage()
        pick_home = Pose()
        pick_home.position.x = 0.05
        pick_home.position.y = 0.5
        pick_home.position.z = 0.42
        pick_home.orientation.x = 0.7071
        pick_home.orientation.y = 0.7071
        pick_home.orientation.z = 0
        pick_home.orientation.w = 0
        
        pick_home_pose_msg.pose = pick_home
        pick_home_pose_msg.incremental = False

        pipeline_core.pose_publisher.write_topic(pick_home_pose_msg)
        rospy.sleep(10.0)


        # Attempt grip

        pose_msg = PoseMessage()
        pose_msg.pose = pipeline_core.stored_pose
        # Translate the pose from periscope suction cup to end effector
        # Suction cup is 0.65m from the end of the end effector
        # Suction cup has 0.05m sag from the end effector due to pipe flex
        pose_msg.pose.position.y -= 0.65
        pose_msg.pose.position.z += 0.05
        pose_msg.incremental = False

        pipeline_core.vacuumonoff.call(1)
        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(10.0)

        # Lift 2cm

        pose_msg = PoseMessage()
        pose_msg.pose.position.z = 0.02
        pose_msg.incremental = True

        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(2.0)

        # Return to pick home

        pipeline_core.pose_publisher.write_topic(pick_home_pose_msg)
        rospy.sleep(10.0)

        pipeline_core.vacuumonoff.call(0)

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Grip attempt finished")
            return 5 # Done
        else:
            return 4 # Stay in DoGrip


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

        self.status = 0b00000000000000
        self.state_initialise = Initialise()
        self.state_calibrate = Calibrate()
        self.state_find_shelf = FindShelf()
        self.state_assess_shelf = AssessShelf()
        self.state_do_grip = DoGrip()
        self.state_spin = SpinState()

        print("State machine started. Current status:", self.status)

    def run(self, pipeline_core):
        """
        Monitors status and runs relevant states
        :param pipeline_core: PipelineCore object
        :return: None
        """

        if self.status == 0b00000000000000:
            self.status = self.state_initialise.run(pipeline_core)
        if self.status in (0b10010000000000, 0b01010000000000):
            self.status = self.state_calibrate.run(pipeline_core)
        if self.status in (0b10011000000000, 0b10011000000000):
            self.status = self.state_find_shelf.run(pipeline_core)
        if self.status in (0b01011000000000, 0b01111110110000, 0b01011111010000):
            self.status = self.state_assess_shelf.run(pipeline_core)
        if self.status == 0b01011110000000:
            self.status = self.state_do_grip.run(pipeline_core)
        else:
            print("End of pipeline code so far")

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

        # TODO: create the prioritised item list of dictionaries (from Tj's program)
        # Imported from prioritise.py
        self.all_items = all_items
        self.item_profile = item_profiles
        self.bin_profiles = bin_profiles
        self.bin_contents = None # Might not need to be shared to all states
        self.work_order = None # Might not need to be shared to all states.
        self.work_order_prioritised = None
        self.target_shelf = None

        # Initiate topics and services:
        # UR10 control:
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Vision topics and services:
        self.get_object_centroid = ServiceCaller("detect_object", detect_object)

        # Vacuum control (commented out for UR10 testing purposes):
        self.vacuumonoff = ServiceCaller("vacuum_switch", vacuum_switch)
        self.vacuumcalibration = ServiceCaller("vacuum_calibration", vacuum_calibration)
        self.vacuumsucking = TopicReader("vacuum_pressure", Bool)

        # Scratch space for pose logging between states:
        self.stored_pose = Pose()

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
