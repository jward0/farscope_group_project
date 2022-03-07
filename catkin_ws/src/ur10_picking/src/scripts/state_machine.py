#!/usr/bin/env python
"""
State machine for the FARSCOPE group project - Amazon Picking Challenge

This script acts as the master controller for the whole system.

Version: v1
2Date: 1 March 2022
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from ur10_picking.srv import *

"""
Define statemachine class
Each state will be initiated using this class.
The class includes state name, on_event, next_state
"""


class state:
    def __init__(self):
        print("Initiating state:", str(self))

    def run(self):
        assert 0, "run not implemented"

    def on_event(self, event):
        assert 0, "on_event not implemented"

    def next_state(self, input):
        assert 0, "next state not implemented"


"""
Define service class
This class is defined for each ROS service interaction
The class can send a request to a service and process a response from a service
"""


class servicecaller:
    def __init__(self, servicename, service_class):
        self.servicename = servicename
        self.service_class = service_class
        rospy.wait_for_service(servicename)
        # rospy.loginfo("Connected to service:", servicename)

    def call(self, req):
        rospy.wait_for_service(self.servicename)
        client = rospy.ServiceProxy(self.servicename, self.service_class)
        response = client(req)
        return response


"""
Define topic class
This class processes data from a ROS topic and returns it
"""


class topicreader:
    def __init__(self, topicname, data_class):
        self.var = None
        self.topicname = topicname
        self.data_class = data_class
        rospy.Subscriber(self.topicname, self.data_class, self.callback)
        rospy.sleep(1)
        rospy.loginfo("Reading topic {}...the following data has been read: {}".format(topicname, self.var))

    def callback(self, data):
        self.var = data.data

    def readtopic(self):
        rospy.Subscriber(self.topicname, self.data_class, self.callback)
        rospy.sleep(1)

        return self.var


"""
MAIN LOOP:
"""
if __name__ == "__main__":
    """
    DEFINE TOPIC AND SERVICES FUNCTIONS
    """

    # UR10 topics and services

    # Vision system topics and services

    # Vacuum 1 topics and services
    vacuum1onoff = servicecaller("vacuum_switch", vacuum_switch)
    vacuum1calibration = servicecaller("vacuum_calibration", vacuum_calibration)
    vacuum1sucking = topicreader("vacuum_pressure", Bool)


    """
    DEFINE TOPIC AND SERVICES FUNCTIONS
    """
    class initialisation(state):
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


    class calibration(state):
        def run(self):
            print("Beginning calibration process")
            # Do vision system calibration
            # Do vacuum calibration
            vacuum1calibration.call("begin vacuum calibration")
            print("Vacuum sucking status: ", vacuum1sucking.readtopic())
            # Do robot arm calibration
            # Do any other calibration
            print("Calibration complete")

        def on_event(self, event):
            print("No on-event function for this state")

        def next_state(self, input):
            if input == "Calibration complete":
                return True
            else:
                return False


    class findshelf(state):
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

    """
    INITITIALISE STATES
    """
    initialisation_state = initialisation()
    calibration_state = calibration()
    findshelf_state = findshelf()

    """
    BEGIN THE STATE MACHINE
    """
    # TEST TEST TEST
    rospy.init_node('state_machine')
    initialisation_state.run()
    if initialisation_state.next_state("Initialisation complete"):
        calibration_state.run()
    if calibration_state.next_state("Calibration complete"):
        findshelf_state.run()



