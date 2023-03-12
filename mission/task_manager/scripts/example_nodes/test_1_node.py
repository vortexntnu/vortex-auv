#!/usr/bin/python3
import rospy


class MyNode1:

    def __init__(self):
        self.enabled = rospy.get_param("/tasks/task_1")

    def spin(self):
        while not rospy.is_shutdown():
            self.enabled = rospy.get_param("/tasks/task_1")

            if self.enabled:
                rospy.loginfo("test 1; Inside callback")

            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("test_1_node")
    my_node_1 = MyNode1()
    my_node_1.spin()
