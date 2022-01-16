#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence, Concurrence, cb_interface, CBState
from smach_ros import SimpleActionState
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
import math


guidance_interface_dp_action_server=rospy.get_param("/guidance/dp/action_server")
guidance_interface_los_action_server=rospy.get_param("/guidance/LOS/action_server")

# rename the file accordingly too; put in own folder

def dp_move(x, y, z=-0.5, yaw_rad=0):

    goal = MoveBaseGoal()
    goal.target_pose.pose.position = Point(x,y,z)
    goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw_rad))

    return SimpleActionState(guidance_interface_dp_action_server, MoveBaseAction, goal=goal)

def los_move(x, y, z=-0.5):

    goal = LosPathFollowingGoal()
    goal.next_waypoint = Point(x,y,z)
    goal.forward_speed = rospy.get_param('~transit_speed', 0.3)
    goal.sphereOfAcceptance = rospy.get_param('~sphere_of_acceptance', 0.5)
    goal.desired_depth = z
    return SimpleActionState(guidance_interface_los_action_server, LosPathFollowingAction, goal=goal)


def circle_move(target_point, direction):
    goal = MoveGoal()

    goal.guidance_type = ''


def patrol_sequence(action_states):

    sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')
    counter = 0

    with sm:

        for state in action_states:
            counter = counter + 1
            sm.add("State-%d" %counter, state)

    return sm


@cb_interface(
    outcomes=['alligned', 'wrong_direction'],
    input_keys=['alligned_pose'],
    output_keys=['alligned_pose']
)
def allignment_checker(userdata):
    # TODO
    pass


def allign_with_target(target):
    """
    Returns a state that alligns with the specified target. 

    target: string on the form 'GATE' or 'BOUY'

    The returned state is responsible for chaning the circeling direction when needed.

    """

    # TODO: get target position (x,y) from landmark server

    # TODO: create a move_goal
    move_goal = None

    allignment_attempt = Concurrence(
        outcomes=['succeeded', 'preempted', 'wrong_direction'],
        outcome_map={
            'succeeded': {'ALLIGNMENT_CHECKER': 'alligned'},
            'wrong_direction': {'ALLIGNMENT_CHECKER': 'wrong_direction'}
        },
        default_outcome=['preempted'],
        child_termination_cb=None   # TODO: should allways terminate
    )

    with allignment_attempt:

        Concurrence.add(
            'CIRCLE_GATE', 
            SimpleActionState('controller/move', MoveAction, goal=move_goal) # TODO
        )
        Concurrence.add(
            'ALLIGNMENT_CHECKER', 
            CBState(allignment_checker)
        )


    
def create_circle_coordinates(start, centre, angle, counterclockwise = True):
    resolution = 0.1 #distance between points
    coordinates = []
    radius = math.sqrt(abs((start[0] - centre[0])**2) + abs((centre[1] - start[1])**2))
    circumference = 2*radius*math.pi
    number_of_points = int(math.ceil(circumference/resolution))
    angle_to_add = angle/number_of_points
    x = start[0]-centre[0]
    y = start[1]-centre[1]
    start_angle = math.atan2(y,x)*180/math.pi
    if not counterclockwise:
        start_angle -= angle
    for i in range(number_of_points + 1):     
        coordX = centre[0] + radius*math.cos(start_angle*math.pi/180)
        coordY = centre[1] + radius*math.sin(start_angle*math.pi/180)
        coordinates.append([coordX, coordY])
        start_angle += angle_to_add
    if not counterclockwise:
        return coordinates[::-1]
    else:
        return coordinates