import rospy
from smach import Sequence
import smach
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler
import time


class GateSearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output_cntrl_name','gate_search_output_target_pose'])
        
        
        self.controller_name_LOS = 'LOS' #2410
        self.target_pose = Pose(None,Quaternion(*quaternion_from_euler(0, 0, 0))) #2410
        
    
    

    def execute(self, userdata):
        rospy.loginfo('executing GateSearchState')
        sub = rospy.Subscriber("goal_position", Point, self.callback)
        rospy.loginfo('waiting for goal position message')
        rospy.wait_for_message('goal_position', Point)
        #2410 rospy.loginfo('goal position message received %f,%f,%f', self.goal_position.x,self.goal_position.y,self.goal_position.z)
        sub.unregister()
        #24.10 userdata.gate_search_output = self.goal_position
        userdata.gate_search_output_cntrl_name = self.controller_name_LOS #2410
        userdata.gate_search_output_target_pose = self.target_pose #2410
        
        rospy.loginfo('sleeping')
        time.sleep(10) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        return 'succeeded'
    
    def callback(self, goal_position):
        rospy.loginfo(rospy.get_caller_id() + "I heard %f,%f,%f", goal_position.x,goal_position.y,goal_position.z)
        #24.10 self.goal_position = goal_position
        self.target_pose.position = goal_position #2410