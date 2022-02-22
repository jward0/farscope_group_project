#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


class PoseTalker():

    def __init__(self, publisher_name):

        self.pub = rospy.Publisher(publisher_name, Pose, queue_size=10)

    def send(self, pose):
        self.pub.publish(pose)


def print_callback(arg):
    print("----------------")
    print(rospy.get_time())
    print(arg)


if __name__ == "__main__":

    rospy.init_node("controller", anonymous=False)

    rate = rospy.Rate(10)

    zero_pose = Pose()
    zero_pose.position.x = -0.1
    zero_pose.position.y = 0.5
    zero_pose.position.z = 0.3
    zero_pose.orientation.x = 0
    zero_pose.orientation.y = 0
    zero_pose.orientation.z = 0.7071081
    zero_pose.orientation.w = 0.7071055

    pose_talker = PoseTalker('/controller/next_cartesian_pose')
    pose_talker.send(zero_pose)
    rospy.Subscriber('/moveit_interface/cartesian_pose_feedback', Pose, print_callback)

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        pose_talker.send(zero_pose)        
        rate.sleep()   

