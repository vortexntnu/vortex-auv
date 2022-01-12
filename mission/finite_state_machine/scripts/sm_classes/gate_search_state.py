import rospy
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String

class GateSearchState(smach.State):
    def __init__(self,client):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output'])           
        self.get_position_client = client
        self.gate_position = self.get_position_client("gate").pos

        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("GATE_SEARCH")
        
    def execute(self, userdata):
        
        while self.gate_position.x == 0: #should not be 0, but c++ NULL translates to 0.0 ...
            print("SEARCHING FOR GATE ...")
            rospy.wait_for_service('send_positions')   
            self.gate_position = self.get_position_client("gate").pos
        
        print("GATE POSITION DETECTED: "+ str(self.gate_position.x) + ", "+ str(self.gate_position.y))                
        userdata.gate_search_output = self.gate_position       
        return 'succeeded'
    
