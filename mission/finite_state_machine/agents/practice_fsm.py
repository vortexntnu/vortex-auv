#!/usr/bin/env python

import rospy

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Point, Pose, Quaternion

from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback

from tf.transformations import quaternion_from_euler



def make_base_goal(x, y, z, yaw):
    nav_goal = MoveBaseGoal()

    nav_goal.target_pose.pose.position = Point(x, y, z)
    nav_goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

    return nav_goal

def make_los_goal(x_prev, y_prev, x_next, y_next, depth, speed=0.20, sphere_of_acceptance=0.3):

    los_goal = LosPathFollowingGoal()

    los_goal.prev_waypoint.x = x_prev
    los_goal.prev_waypoint.y = y_prev

    los_goal.next_waypoint.x = x_next
    los_goal.next_waypoint.y = y_next

    los_goal.desired_depth.z = depth

    los_goal.forward_speed.linear.x = speed
    los_goal.sphereOfAcceptance = sphere_of_acceptance

    return los_goal


rospy.init_node('practice_fsm')

patrol = StateMachine(outcomes=[],
                        input_keys=[],
                        output_keys=[])

with patrol:

    StateMachine.add('CHECKPOINT_1',
                    SimpleActionState('los_path',
                                        LosPathFollowingAction,
                                        make_los_goal(0.0, 0.0, 2.0, 2.0, 0.0)),
                    transitions={'succeeded': 'CHECKPOINT_2',
                                'aborted': 'CHECKPOINT_1',
                                'preempted': 'CHECKPOINT_2'})

    StateMachine.add('CHECKPOINT_2',
                    SimpleActionState('los_path',
                                        LosPathFollowingAction,
                                        make_los_goal(2.0, 2.0, 0.0, 0.0, 0.0)),
                    transitions={'succeeded': 'CHECKPOINT_1',
                                'aborted': 'CHECKPOINT_2',
                                'preempted': 'CHECKPOINT_1'})    



# Create and start the SMACH Introspection server
intro_server = IntrospectionServer(str(rospy.get_name()), patrol,'/SM_ROOT')
intro_server.start()

patrol.execute()

intro_server.stop()





