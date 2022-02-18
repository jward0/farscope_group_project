#!/usr/bin/env python
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose


class PoseTalker():

    def __init__(self, publisher_name):

        self.pub = rospy.Publisher(publisher_name, Pose, queue_size=10)

    def send(self, pose):
        self.pub.publish(pose)


class MoveitInterface():
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_interface', anonymous=False)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')
        
        self.pose_talker = PoseTalker('/moveit_interface/cartesian_pose_feedback')
        self.sub = rospy.Subscriber('/controller/next_cartesian_pose', Pose, self.move_to_pose)

    def move_to_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def feedback(self):
        pose = self.move_group.get_current_pose().pose
        self.pose_talker.send(pose)


if __name__ == "__main__":

    interface = MoveitInterface()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        interface.feedback()
        rate.sleep()

