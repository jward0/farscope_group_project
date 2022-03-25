#!/usr/bin/env python
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from ur10_picking.msg import PoseMessage

import tf_conversions
import tf2_ros

class PoseTalker:

    def __init__(self, publisher_name):

        self.pub = rospy.Publisher(publisher_name, PoseMessage, queue_size=10)

    def send(self, pose, incremental=False):
        pose_msg = PoseMessage()
        pose_msg.pose = pose
        pose_msg.incremental = incremental
        self.pub.publish(pose_msg)


class MoveitInterface:
    
    def __init__(self):
        """
        Initialises all the various MoveIt things, sets up ROS interfaces, and moves through two initial poses-
        first, straight upwards to ensure joints aren't twisted, and then to a provisional home pose
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_interface', anonymous=False)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')
        
        self.pose_talker = PoseTalker('/moveit_interface/cartesian_pose_feedback')
        self.pose_sub = rospy.Subscriber('/pipeline/next_cartesian_pose', PoseMessage, self.move_to_pose)
        self.trajectory_sub = rospy.Subscriber('/pipeline/cartesian_trajectory', PoseArray, self.move_trajectory)

        self.br = tf2_ros.TransformBroadcaster()

        self.move_group.set_joint_value_target([1.571, -1.571, 0.0, 0.0, 0.0, 0.0])
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(10.0)
        self.move_group.set_joint_value_target([1.407, -1.812, 2.184, -0.384, 1.408, 0.0])
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(10.0)

    def move_to_pose(self, pose_message):
        """
        Sends command to MoveIt to move to a specified Cartesian pose. If pose_message.incremental,
        determines incremental change to current pose and sends that

        :param pose_message: PoseMessage object (incremental, pose)
        :return:
        """

        if pose_message.incremental:
            pose = self.move_group.get_current_pose().pose
            pose.position.x += pose_message.pose.position.x
            pose.position.y += pose_message.pose.position.y
            pose.position.z += pose_message.pose.position.z
        else:
            pose = pose_message.pose

        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def move_trajectory(self, cartesian_trajectory):
        """
        Takes a PoseArray input and sends a cartesian path request to MoveIt based on the PoseArray

        :param cartesian_trajectory: PoseArray object containing waypoints for trajectory
        :return: None
        """

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                                cartesian_trajectory.poses,
                                                0.01,
                                                0.0)
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def feedback(self):
        """
        Monitors robot pose and sends back on pose_talker
        Also publishes to tf2 as a frame
        :return: None
        """
    
        # joint_states_temp = self.move_group.get_current_joint_values()
        # print("++++++++++++++++++++++++++++++++++++")
        # print(joint_states_temp)
        # print("++++++++++++++++++++++++++++++++++++")
        pose = self.move_group.get_current_pose().pose
        self.pose_talker.send(pose)
        print(pose)     
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "wrist_centre"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.br.sendTransform(t)


if __name__ == "__main__":

    interface = MoveitInterface()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        interface.feedback()
        rate.sleep()
