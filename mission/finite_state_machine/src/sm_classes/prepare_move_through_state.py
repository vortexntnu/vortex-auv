import rospy
import smach
from nav_msgs.msg import Odometry

move_action_server = '/guidance/move'

class PrepareMoveThroughState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['goal_pos_input'])

        rospy.Subscriber("odometry/filtered", Odometry, self.callback)                  
       
        self.pos = None    
        
    def execute(self, userdata):
                
        rospy.loginfo("current position: %f,%f,%f", self.pos.x,self.pos.y,self.pos.z)

        rospy.loginfo("goal position: %f,%f,%f", userdata.goal_pos_input.x,userdata.goal_pos_input.y,userdata.goal_pos_input.z)

        # if self.pos.x < userdata.goal_pos_input.x:
        #     dp_move(userdata.goal_pos_input.x-1,userdata.goal_pos_input.y)
        # else :
        #     dp_move(userdata.goal_pos_input.x+1,userdata.goal_pos_input.y)
        return 'succeeded'
    
    def callback(self, odom):
        
        self.pos = odom.pose.pose.position


        
        