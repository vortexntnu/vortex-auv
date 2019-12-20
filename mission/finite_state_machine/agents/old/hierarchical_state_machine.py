#!/usr/bin/env	python
import	rospy
from    time import sleep
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal

# import object detection
from	vortex_msgs.msg import CameraObjectInfo

# Imported help functions from src/finite_state_machine/
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

class Navigation():

    def __init__(self):

        self.x  = 0
        self.y  = 0
        self.z  = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 0

        # pole detection states
    
        self.pole_px = -1
        self.pole_py = -1
        self.pole_fx = 0
        self.pole_fy = 0
        self.pole_confidence = 0
        self.distance_to_pole = 0

        # gate detection states
        self.gate_px = -1
        self.gate_py = -1
        self.gate_fx = 0
        self.gate_fy = 0
        self.gate_confidence = 0
        self.distance_to_gate = 0  

        # subscriber
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
        self.sub_pole = rospy.Subscriber('/pole_midpoint', CameraObjectInfo, self.poleDetectionCallback, queue_size=1)
        self.sub_gate = rospy.Subscriber('/gate_midpoint', CameraObjectInfo, self.gateDetectionCallback, queue_size=1)

    def positionCallback(self, msg):
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        self.z  = msg.pose.pose.position.z
        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w
    
    def poleDetectionCallback(self, msg):
        
        self.pole_px = msg.pos_x
        self.pole_py = msg.pos_y
        self.pole_fx = msg.frame_width
        self.pole_fy = msg.frame_height
        self.pole_confidence = msg.confidence
        self.pole_distance = msg.distance_to_pole

    def gateDetectionCallback(self, msg):

        self.gate_px = msg.pos_x
        self.gate_py = msg.pos_y
        self.gate_fx = msg.frame_width
        self.gate_fy = msg.frame_height
        self.gate_confidence = msg.confidence
        self.gate_distance = msg.distance_to_pole


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
        mission    = Mission()
        navigation = Navigation()
        wpc        = WaypointClient()
        los        = PathFollowingClient()

    except rospy.ROSInterruptException:
        rospy.loginfo("Unable to run constructor")

    # possible transitions
    patrol = StateMachine(['succeeded','aborted','preempted'])

    with patrol:

        waypoints   =   [['one',    (4.0,  -2.0, -0.75),    (0.0,   0.0,    0.0)],
                         ['two',    (7.0,  2.0, -0.75),    (0.0,   0.0,     0.0)]]

        # Adding the states and transitions
        StateMachine.add('POSE_HOLD',     ControlMode(POSE_HOLD), transitions={'success':'POSE_HOLD - ' + waypoints[0][0]})
        StateMachine.add('POSE_HOLD - ' + waypoints[0][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[0])),transitions={'succeeded':'POSE_HOLD - ' + waypoints[1][0]})
        StateMachine.add('POSE_HOLD - ' + waypoints[1][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[1])),transitions={'succeeded':'DEPTH_HOLD'})
        StateMachine.add('DEPTH_HOLD',    ControlMode(DEPTH_HEADING_HOLD), transitions={'success':'DEPTH_HOLD - ' + waypoints[1][0]})
        StateMachine.add('DEPTH_HOLD - ' + waypoints[1][0], SimpleActionState('move_base',MoveBaseAction,goal=wpc.trackNewWaypoint(waypoints[1])),transitions={'succeeded':'succeeded'})

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
    # path_client(self, x_0, y_0, x_1, y_1, u_d, R):
    _goal1 = los.path_client(-2, -2, 15, 3, 0.2, -0.5, 0.5)
    _goal2 = los.path_client(15, 3, 20, -2, 0.2, -0.5, 0.5)

    # State machine 
    with transit:
        StateMachine.add('OPEN_LOOP',     ControlMode(OPEN_LOOP), transitions={'success':'transit'})
        StateMachine.add('transit', SimpleActionState('los_path',LosPathFollowingAction, goal=_goal1), transitions={'succeeded':'goal2'})
        StateMachine.add('goal2', SimpleActionState('los_path',LosPathFollowingAction, goal=_goal2), transitions={'succeeded':'succeeded'})

    # Define outcomes
    shapes = StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'completed'])
    
    # Creating a hierarchical state machine nesting several sm's
    with shapes:
        StateMachine.add('terminal', patrol, transitions={'succeeded':'square'})
        StateMachine.add('square', square, transitions={'succeeded':'transit'})
        StateMachine.add('transit', transit, transitions={'succeeded':'aborted'})

    
    sis = IntrospectionServer(str(rospy.get_name()), shapes, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    shapes.execute()
    rospy.spin()
    sis.stop()
