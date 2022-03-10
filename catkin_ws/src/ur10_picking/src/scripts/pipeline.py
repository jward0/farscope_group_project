#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from ur10_picking.msg import PoseMessage

class PoseTalker():

    def __init__(self, publisher_name):

        self.pub = rospy.Publisher(publisher_name, PoseMessage, queue_size=10)

    def send(self, pose, incremental=False):
        pose_msg = PoseMessage()
        pose_msg.pose = pose
        pose_msg.incremental = incremental
        self.pub.publish(pose_msg)


class TrajectoryTalker():

    def __init__(self, publisher_name):

        self.pub = rospy.Publisher(publisher_name, PoseArray, queue_size=10)

    def send(self, trajectory):
        self.pub.publish(trajectory)


def print_callback(arg):
    print("----------------")
    print(rospy.get_time())
    print(arg)


class Pipeline():

    
    def __init__(self):
    
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)
        self.pose_talker = PoseTalker('/pipeline/next_cartesian_pose')
        self.trajectory_talker = TrajectoryTalker('/pipeline/cartesian_trajectory')
        rospy.Subscriber('/moveit_interface/cartesian_pose_feedback', PoseMessage, self.log_pose)
    
    def log_pose(self, pose):
        self.current_pose = pose
        print_callback(pose)
    
    def pose_start(self):

        start_pose = Pose()
        start_pose.position.x = 0.1
        start_pose.position.y = -0.5
        start_pose.position.z = 0.3
        start_pose.orientation.x = 0
        start_pose.orientation.y = 0
        start_pose.orientation.z = 0.707
        start_pose.orientation.w = 0.707
    
        self.pose_talker.send(start_pose)

        rospy.sleep(10.0)
 

if __name__ == "__main__":

    pipeline = Pipeline()
    rospy.sleep(30.0) # To allow robot to home before sending start pose
    
    pose_adjustment = Pose()
    pose_adjustment.position.x = 0.01
    
    while not pipeline.current_pose
        rospy.sleep(0.5)
       
    next_pose = pipeline.current_pose.pose
    waypoints = []
    next_pose.position.x += 0.02
    waypoints.append(copy.deepcopy(next_pose))
    next_pose.position.z += 0.04
    waypoints.append(copy.deepcopy(next_pose))
    next_pose.position.x -= 0.04
    waypoints.append(copy.deepcopy(next_pose))
    next_pose.position.z -= 0.04
    waypoints.append(copy.deepcopy(next_pose))
    next_pose.position.x += 0.02
    waypoints.append(copy.deepcopy(next_pose))
    
    trajectory_message = PoseArray()
    trajectory_message.poses = waypoints
    pipeline.trajectory_talker.send(trajectory_message)
    rospy.sleep(30.0)

    for _ in range(10):
        pipeline.pose_talker.send(pose_adjustment, incremental=True)        
        pipeline.rate.sleep()   

