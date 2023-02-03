#!/usr/bin/python3
import rospy

class MyNode1:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/task_1")

    def my_node_1(self, msg):
        if self.enabled:
            rospy.loginfo("test 1; Inside callback")
        

if __name__ == "__main__":
    rospy.init_node("test_1_node")
    my_node_1 = MyNode1()
    rospy.spin()

    