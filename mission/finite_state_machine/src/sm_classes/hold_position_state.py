import rospy
import smach
import math
import actionlib
from vortex_msgs.msg import MoveAction, MoveGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

move_action_server = '/guidance/move'
drift_tolerance = 0.1

class HoldPositionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])

        rospy.Subscriber("odometry/filtered", Odometry, self.odom_cb)
        
        #Next goal to move to using LOS guidance, a message on this topic breaks the position hold loop
        rospy.Subscriber("new_target",Point,self.event_cb) 

        self.client = actionlib.SimpleActionClient(move_action_server, MoveAction)                   
        
        self.continue_waiting = True
        self.hold_position_acquired = False
        self.hold_position = None   
        self.current_position = None   
        
    def execute(self, userdata):

        rospy.wait_for_message('odometry/filtered', Odometry)
        self.hold_position_acquired = True

        while self.continue_waiting:
            if (self.check_drift() > drift_tolerance):
                goal = MoveGoal()
                goal.target_pose.position = self.hold_position                
                goal.guidance_type = "PositionHold"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                result = self.client.get_result()
                #if (result == "aborted"):
                    #to do 
                #elif (result == "preempted"):
                    #to do
            rospy.sleep(1)        
        
        self.client.cancel_all_goals()                        
        return 'succeeded'
    
    def odom_cb(self, odom):
        if self.hold_position_acquired == True:
            self.hold_position = odom.pose.pose.position
        else:
            self.current_position = odom.pose.pose.position

    def event_cb(self,pos):
        self.continue_waiting = False


    def check_drift(self):
        drift_dist = math.sqrt(pow(self.current_position.x,2)+pow(self.current_position.y,2)+pow(self.current_position.z,2))
        return drift_dist
        
        