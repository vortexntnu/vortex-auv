import rospy
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String

class PoleSearchState(smach.State):
    def __init__(self,client):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['pole_search_output'])           
        self.pole_position = None 
        self.get_position_client = client

        #testing
        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("POLE_SEARCH")
        
    def execute(self, userdata):

        #possibly move the drone around until it finds the pole first?
        while (self.pole_position is None):
            print("IN WHILE")
            rospy.wait_for_service('send_positions')   
            position = self.get_position_client("pole")
            self.pole_position = position.pos
               
                        
        userdata.pole_search_output = self.pole_position       
        return 'succeeded'
    
