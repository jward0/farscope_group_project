#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


class PoseTalker():

    def __init__(self):

        self.pub = rospy.Publisher("/controller/next_cartesian_pose", Pose, queue_size=10)

    def send(self, pose):
        self.pub.publish(pose)


if __name__ == "__main__":

    rospy.init_node("controller", anonymous=False)

    rate = rospy.Rate(10)

    zero_pose = Pose()
    zero_pose.position.x = 0.5
    zero_pose.position.y = 0.5
    zero_pose.position.z = 0.5

    pose_talker = PoseTalker()
    pose_talker.send(zero_pose)

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        pose_talker.send(zero_pose)        
        rate.sleep()   

