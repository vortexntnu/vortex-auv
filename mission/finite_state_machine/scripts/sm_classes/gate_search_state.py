import rospy
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String

class GateSearchState(smach.State):
    def __init__(self,client):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output'])           
        self.gate_position = None 
        self.get_position_client = client

        #testing
        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("GATE_SEARCH")
        
    def execute(self, userdata):

        #possibly rotate the drone until it finds the gate first?
        while (self.gate_position is None):
            print("IN WHILE")
            rospy.wait_for_service('send_positions')   
            position = self.get_position_client("gate")
            self.gate_position = position.pos
               
                        
        userdata.gate_search_output = self.gate_position       
        return 'succeeded'
    
