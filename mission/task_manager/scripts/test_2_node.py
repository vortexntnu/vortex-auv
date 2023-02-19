#!/usr/bin/python3
import rospy


class MyNode2:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/task_2")

    def spin(self):
        while not rospy.is_shutdown():
            self.enabled = rospy.get_param("/tasks/task_2")

            if self.enabled:
                rospy.loginfo("test 2; Inside callback")

            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("test_2_node")
    my_node_2 = MyNode2()
    my_node_2.spin()
