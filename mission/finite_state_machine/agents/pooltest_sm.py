#!/usr/bin/env	python
import	rospy
from    time import sleep
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal

# Imported help functions from src/finite_state_machine
from    finite_state_machine import ControllerMode, WaypointClient, PathFollowingClient

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3 
DEPTH_HOLD          = 4
STAY_LEVEL          = 5
CONTROL_MODE_END    = 6


class Mission():

    def __init__(self):
        
        # ros init
        rospy.init_node('mission_fsm', anonymous=True)

        # rate
        self.rate = rospy.Rate(100) #Hz

class Vehicle():

    def __init__(self):

        # Subscriber
        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)

    def positionCallback(self, msg):
        
        self.vehicle = msg
        
class ControlMode(State):

    def __init__(self, mode):
        State.__init__(self, outcomes=['success'])
        self.mode = mode
        self.control_mode = ControllerMode()

    def execute(self, userdata):

        # change control mode
        self.control_mode.change_control_mode_client(self.mode)
        return 'success'

class Drive(State):
    def __init__(self, distance):
        State.__init__(self,outcomes=['succeeded','aborted','preempted'])
        self.distance = distance

    def execute(self, userdata):
        print 'Driving', self.distance
        sleep(1)
        return 'succeeded'

class Turn(State):
    def __init__(self, angle):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.angle = angle

    def execute(self, userdata):
        print 'Turning', self.angle
        sleep(1)
        return 'succeeded'


if __name__ == '__main__':

    try:
        mission = Mission()
        vehicle = Vehicle()
        wpc     = WaypointClient()
        los     = PathFollowingClient()

    except rospy.ROSInterruptException:
        rospy.loginfo("Unable to run constructor")

    # possible transitions
    patrol = StateMachine(['succeeded','aborted','preempted'])

    with patrol:

        waypoints   =   [['one',    (20.0,   -2.0, -0.75),    (0.0,   0.0,    0.0)],
                         ['two',    (24.0,  -2.0, -0.75),    (0.0,   0.0,    1.57)]]

        # Adding the states and transitions
        StateMachine.add('POSE_HOLD',     ControlMode(POSE_HOLD), transitions={'success':'POSE_HOLD - ' + waypoints[0][0]})
        StateMachine.add('POSE_HOLD - ' + waypoints[0][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[0])),transitions={'succeeded':'POSE_HOLD - ' + waypoints[1][0]})
        StateMachine.add('POSE_HOLD - ' + waypoints[1][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[1])),transitions={'succeeded':'DEPTH_HOLD'})
        StateMachine.add('DEPTH_HOLD',    ControlMode(DEPTH_HEADING_HOLD), transitions={'success':'DEPTH_HOLD - ' + waypoints[1][0]})
        StateMachine.add('DEPTH_HOLD - ' + waypoints[1][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[1])),transitions={'succeeded':'OPEN_LOOP'})
        StateMachine.add('OPEN_LOOP',     ControlMode(OPEN_LOOP), transitions={'success':'succeeded'})

    # Define outcomes
    square = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
   
    # State machine square
    with square:
        StateMachine.add('SIDE1', Drive(1), transitions={'succeeded':'TURN1'})
        StateMachine.add('TURN1', Turn(90), transitions={'succeeded':'SIDE2', 'preempted':'TURN2'})
        StateMachine.add('SIDE2', Drive(1), transitions={'succeeded':'TURN2'})
        StateMachine.add('TURN2', Turn(90), transitions={'succeeded':'SIDE3'})
        StateMachine.add('SIDE3', Drive(1), transitions={'succeeded':'TURN3'})
        StateMachine.add('TURN3', Turn(90), transitions={'succeeded':'SIDE4'})
        StateMachine.add('SIDE4', Drive(1), transitions={'succeeded':'succeeded'})


    # Define transit state
    transit = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    _goal = los.path_client(0, 0, 16, -2, 0.2, 0.5)

    # State machine 
    with transit:
        StateMachine.add('transit', SimpleActionState('los_path',LosPathFollowingAction, goal=_goal), transitions={'succeeded':'succeeded'})
    
    # Define outcomes
    shapes = StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'completed'])
    
    # Creating a hierarchical state machine nesting several sm's
    with shapes:
        StateMachine.add('transit', transit, transitions={'succeeded':'square'})
        StateMachine.add('square', square, transitions={'succeeded':'terminal'})
        StateMachine.add('terminal', patrol, transitions={'succeeded':'completed'})
    
    sis = IntrospectionServer(str(rospy.get_name()), shapes, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    shapes.execute()
    rospy.spin()
    sis.stop()
