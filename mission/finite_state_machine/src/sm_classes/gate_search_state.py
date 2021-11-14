import rospy
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String

class GateSearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output'])

        rospy.Subscriber("goal_position", Point, self.callback) #possibly change message type (for gate, maybee 3 points?)
                    
        self.goal_position = None 

        #testing
        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("GATE_SEARCH")
        
    def execute(self, userdata):

        #possibly rotate the drone until it finds the gate first?
               
        rospy.wait_for_message('goal_position', Point)
                        
        userdata.gate_search_output = self.goal_position       
        
        return 'succeeded'
    
    def callback(self, goal_position):
        
        self.goal_position = goal_position
        
        