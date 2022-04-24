
import rospy
from std_msgs.msg import String

class Perciever:
    def __init__(self):
        self.state = ""
        rospy.Subscriber("/fsm/state",String,self.state_cb)

    def state_cb(self, msg):
        self.state = msg

    def execute():
        rate = rospy.Rate(1)
        while not rospy.is_shutdown:
            
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('perception_simulator')
    perceptron = Perciever()
    perceptron.execute()
    