#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from vortex_msgs.msg import ObjectPosition
from geometry_msgs.msg import Point
import time


class Perciever:
    def __init__(self):
        self.state = "da"
        rospy.Subscriber("/fsm/state", String, self.state_cb)

        self.landmarks_pub = rospy.Publisher(
            "/fsm/object_positions_in", ObjectPosition, queue_size=1
        )

        self.gate = ObjectPosition()
        self.gate.objectID = "gate"
        self.gate.objectPose.pose.position = Point(10, 0, -1.4)
        self.gate.objectPose.pose.orientation.x = 0
        self.gate.objectPose.pose.orientation.y = 0
        self.gate.objectPose.pose.orientation.z = 1
        self.gate.objectPose.pose.orientation.w = 0
        self.gate.isDetected = True
        self.gate.estimateConverged = True

        self.pole = ObjectPosition()
        self.pole.objectID = "pole"
        self.pole.objectPose.pose.position = Point(16.7, 0, -1.4)
        self.pole.objectPose.pose.orientation.x = 0
        self.pole.objectPose.pose.orientation.y = 0
        self.pole.objectPose.pose.orientation.z = 1
        self.pole.objectPose.pose.orientation.w = 0
        self.pole.isDetected = True
        self.pole.estimateConverged = True

    def state_cb(self, msg):
        self.state = msg.data

    def execute(self):
        rate = rospy.Rate(10)
        rospy.sleep(rospy.get_param("/fsm/time_to_launch"))
        while self.state != "gate_search" and not rospy.is_shutdown():
            rate.sleep()
        time.sleep(5)
        self.landmarks_pub.publish(self.gate)
        time.sleep(10)
        self.gate.estimateFucked = True
        self.landmarks_pub.publish(self.gate)
        while self.state != "pole_search" and not rospy.is_shutdown():
            rate.sleep()
        self.landmarks_pub.publish(self.pole)
        while self.state != "gate_search" and not rospy.is_shutdown():
            rate.sleep()
        self.gate.objectPose.pose.position.x = 3
        self.gate.objectPose.pose.orientation.z = 0
        self.gate.objectPose.pose.orientation.w = 1
        self.landmarks_pub.publish(self.gate)


if __name__ == "__main__":
    rospy.init_node("perception_simulator")
    perceptron = Perciever()
    perceptron.execute()
