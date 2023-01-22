#!/usr/bin/env python3
import rospy

class MyNode2:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/task_1")

    def my_node_2(self, msg):
        if self.enabled:
            rospy.loginfo("test 2; Inside callback")
            

if __name__ == "__main__":
    rospy.init_node("test_2_node")
    my_node_2 = MyNode2()
    rospy.spin()
