#!/usr/bin/env python
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose


class MoveitInterface():
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_interface', anonymous=False)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')

    def callback(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def listen(self):
        rospy.Subscriber('/controller/next_cartesian_pose', Pose, self.callback)
        rospy.spin()

if __name__ == "__main__":

    interface = MoveitInterface()

    while not rospy.is_shutdown():
        interface.listen()

