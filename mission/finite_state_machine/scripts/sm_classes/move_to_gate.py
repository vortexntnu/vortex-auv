import rospy
from smach import StateMachine
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String
from fsm_helper import dp_move, los_move
from nav_msgs.msg import Odometry

class MoveToGate(smach.State):
    
    def __init__(self, odom):
        self.odom = odom
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position'])    
        

    def execute(self, userdata):
        print("should perform the dp move to "+str(userdata.gate_position.x)+" , " +str(userdata.gate_position.y))
        gate_move_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with gate_move_sm:
            print("odom: " + str(self.odom.pose.pose.position.x))
            x_dist = userdata.gate_position.x - self.odom.pose.pose.position.x
            print(str(x_dist))
            distance_from_gate = 1
            
            StateMachine.add('MOVE_TO_GATE',
                            dp_move(x_dist - distance_from_gate,userdata.gate_position.y))
            print("odom: " + str(self.odom.pose.pose.position.x))
        try:
            gate_move_sm.execute()

        except Exception as e:
            rospy.loginfo("gate_move_sm failed: %s" % e)

        return 'succeeded'