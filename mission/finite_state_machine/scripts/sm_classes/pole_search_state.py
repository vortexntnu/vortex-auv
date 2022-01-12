import rospy
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String
from landmarks.srv import request_position

class PoleSearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['pole_search_output'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        self.pole_position = self.landmarks_client("pole").pos

        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("POLE_SEARCH")
        
    def execute(self, userdata):

        while self.pole_position.x == 0:
            print("SEARCHING FOR POLE ...")
            rospy.wait_for_service('send_positions')   
            self.pole_position = self.landmarks_client("pole").pos
        
        print("POLE POSITION DETECTED: "+ str(self.pole_position.x) + ", "+ str(self.pole_position.y))    
                        
        userdata.pole_search_output = self.pole_position       
        return 'succeeded'
    
