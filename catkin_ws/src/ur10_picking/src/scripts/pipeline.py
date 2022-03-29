#!/usr/bin/env python

import rospy
from roscomm import ServiceCaller, TopicReader, TopicWriter
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray
from ur10_picking.msg import PoseMessage
from ur10_picking.srv import *


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
        print("Moving to shelf")

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

        # Turn off vacuum
        pipeline_core.vacuumonoff.call(0)

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
        # TODO: Assess shelf - use vision node topic to extract the centroid of the item
        # TODO: Print centroid of the item to terminal

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

        # TODO: create the priortised item list of dictionaries (from Tj's program)

        # Initiate topics and services:
        # UR10 control:
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)

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
        rospy.spin()


if __name__ == "__main__":
    run_pipeline()
