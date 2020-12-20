#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction

class DPGuidance:
    """
    The dynamic positioning guidance system is very simple.
    It does not perform any calculations, but simply passes
    the setpoints directly to the DP controller in the
    /controller/dp node.

    The dp guidance node is different from the LOS node in the sense
    that it will continuously publish on the topic until a different
    guidance mode is requested (and thats it, nothing else).

    Strategy:
        change control mode when this is addressed, change to OPEN LOOP
        when a different guidance node is selected.
    """

    def __init__(self):

        rospy.init_node('dp')

        self.publish_guidance_data = False


        # Subscribers
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1)

        # Publishers
        self.pub_data_dp = rospy.Publisher('/guidance/dp_data', Pose, queue_size=1)

        # Create action server (previously move_base)
        # When a goal is sent from guidance_interface, the goalCB is called
        self.action_server = actionlib.SimpleActionServer(name='dp_action_server', ActionSpec=MoveBaseAction, auto_start=False)
        self.action_server.register_goal_callback(self.goalCB)
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

        
    def callback(self, msg):
        dp_data = Pose()
        self.pub_data_dp.publish(dp_data)

    def goalCB(self):
        """
        This will replace the actionGoalCallBack() in the controller_ros.cpp file:

        instead of the controller setting variables locally, it will set variables
        in this node, and will receive them "continuously" from this node through
        the data topic.

        Important to set local variables here and publish those since the goalCB 
        is not continuously called!
        """


        """
        void Controller::actionGoalCallBack()
        {
            // set current target position to previous position
            m_controller->x_d_prev = position;
            m_controller->x_d_prev_prev = position;
            m_controller->x_ref_prev = position;
            m_controller->x_ref_prev_prev = position;

            // accept the new goal - do I have to cancel a pre-existing one first?
            mGoal = mActionServer->acceptNewGoal()->target_pose;

            // print the current goal
            ROS_INFO("Controller::actionGoalCallBack(): driving to %2.2f/%2.2f/%2.2f", mGoal.pose.position.x, mGoal.pose.position.y, mGoal.pose.position.z);

            // Transform from Msg to Eigen
            tf::pointMsgToEigen(mGoal.pose.position, setpoint_position);
            tf::quaternionMsgToEigen(mGoal.pose.orientation, setpoint_orientation);

            // setpoint declared as private variable
            m_setpoints->set(setpoint_position, setpoint_orientation);

            // Integral action reset
            m_controller->integral = Eigen::Vector6d::Zero();

        }
        """
        pass

    def preemptCB(self):
        """
            void Controller::preemptCallBack()
            {

                //notify the ActionServer that we've successfully preempted
                ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");	

                // set the action state to preempted
                mActionServer->setPreempted();
            }
        """
        pass

if __name__ == '__main__':

    try:
        dp_guidance = DPGuidance()
        rospy.spin()

    except rospy.ROSInternalException as e:
        rospy.logerr(e)