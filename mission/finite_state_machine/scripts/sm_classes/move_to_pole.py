import rospy
from smach import StateMachine
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String
from fsm_helper import dp_move, los_move

class MoveToPole(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole_position'])           

    def execute(self, userdata):
        print("should perform the dp move to "+str(userdata.pole_position.x)+" , " +str(userdata.pole_position.y))
        pole_move_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with pole_move_sm:
            StateMachine.add('MOVE_TO_POLE',
                        dp_move(userdata.pole_position.x,userdata.pole_position.y))
        
        try:
            pole_move_sm.execute()
        except Exception as e:
            rospy.loginfo("pole_move_sm failed: %s" % e)

        return 'succeeded'