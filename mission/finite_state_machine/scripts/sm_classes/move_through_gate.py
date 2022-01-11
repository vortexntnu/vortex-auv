import rospy
from smach import StateMachine
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String
from fsm_helper import dp_move, los_move

class MoveThroughGate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position'])           

    def execute(self, userdata):
        print("should perform the dp move to "+str(userdata.gate_position.x + 1)+" , " +str(userdata.gate_position.y))
        gate_move_through_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with gate_move_through_sm:
            StateMachine.add('MOVE_THROUGH_GATE_X',
                        dp_move(userdata.gate_position.x + 1,userdata.gate_position.y))
        try:
            gate_move_through_sm.execute()

        except Exception as e:
            rospy.loginfo("gate_move_sm failed: %s" % e)

        return 'succeeded'