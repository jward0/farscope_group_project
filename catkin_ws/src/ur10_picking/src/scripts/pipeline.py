#!/usr/bin/env python

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

    def run(self, pipeline_core):
        assert 0, "run not implemented"

    def on_event(self, event):
        assert 0, "on_event not implemented"

    def next_state(self, state_complete):
        assert 0, "next state not implemented"


class Initialise(State):
    """
    Class defintion for the initialisation state
    Run launch file
    Turn on and check nodes are on and topics are publishing
    Prioritisation - list of lists output
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Initialising the system and prioritising items")
        # Do item initiation program here
        # Do item prioritisation program here
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
    output - Bin position - xyz
    Output - Centroids of each shelf - list or dict of coordinates / Pose for shelf home
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Beginning calibration process")
        # Do vision system calibration
        # Do vacuum calibration
        
        # Do robot arm calibration - position relative to shelf
        # Do any other calibration
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
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Moving to shelf")
       
        pose_msg = PoseMessage()
        shelf_centre_pose = Pose()
        shelf_centre_pose.position.x = 0.1
        shelf_centre_pose.position.y = -0.5
        shelf_centre_pose.position.z = 0.3
        shelf_centre_pose.orientation.x = 0
        shelf_centre_pose.orientation.y = 0
        shelf_centre_pose.orientation.z = 0

        pose_msg.pose = shelf_centre_pose
        pose_msg.incremental = False

        pipeline_core.pose_publisher.write_topic(pose_msg)
        rospy.sleep(5.0)

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
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Assessing shelf")
        # Assess shelf - use vision node topic to extract the centroid of the item

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
        self.var = data

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

        self.status = 0
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

    def report_status(self):
        print("Current state: ", self.status)


class PipelineCore:
    """
    Handles all persistent data and ROS interfaces
    """

    def __init__(self):
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)


def run_pipeline():
    """
    Instantiates pipeline_core and state_machine objects, and then just spins the state machine
    :return: None
    """
    pipeline_core = PipelineCore()
    while not pipeline_core.pose_feedback_subscriber.var:
        rospy.sleep(0.1)
    state_machine = StateSupervisor()
    while True:
        state_machine.run(pipeline_core)
        rospy.spin()


if __name__ == "__main__":
    run_pipeline()
