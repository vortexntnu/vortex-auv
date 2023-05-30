#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose, Wrench

from libdockingfsm.Docking import Docking
from libdockingfsm.Helpers import distance_between_points

class DockingExecute(smach.State):

    def __init__(self, is_enabled=False):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.docking = Docking()

        # Duration for docking
        self.docking_duration = rospy.get_param("/tac/docking/docking_duration")

        self.force_z = rospy.get_param(
                "/joystick/scaling/heave"
            )
        
        self.wrench_pub = rospy.Publisher(
            rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)
        
        self.is_logged = False
        self.docking.task_manager_client.is_enabled = is_enabled

        rospy.on_shutdown(self.on_shutdown)

    def execute(self, userdata):
        self.docking.state_pub.publish("docking/execute")

        while not rospy.is_shutdown():
            if not self.docking.task_manager_client.is_enabled:
                # Handles task change
                if self.docking.task_manager_client.was_enabled:
                    rospy.loginfo("STOPPING DOCKING EXECUTE!")
                    self.is_logged = False
                    self.docking.dp_client.set_acceptance_margins([0.01,0.01,0.01,0.0,0.0,10.0])
                    self.docking.dp_client.goal.x_ref = self.docking.odom_pose
                    self.docking.dp_client.send_goal()
                    self.docking.task_manager_client.was_enabled = False
                continue

            if not self.is_logged:
                self.docking.task_manager_client.was_enabled = True
                rospy.loginfo("STARTING DOCKING EXECUTE!")
                rospy.sleep(rospy.Duration(1))
                self.docking.dp_client.disable()
                self.finished_docking_time = rospy.get_time() + self.docking_duration
                self.is_logged = True

            wrench_msg = Wrench()
            wrench_msg.force.z = -self.force_z * 0.5
            self.wrench_pub.publish(wrench_msg)

            if (self.finished_docking_time < rospy.get_time()):
                self.on_shutdown()
                return 'succeeded'

            self.docking.task_manager_client.was_enabled = True
            self.docking.rate.sleep()
    
    def on_shutdown(self):
        wrench_msg = Wrench()
        self.wrench_pub.publish(wrench_msg)

        self.docking.dp_client.set_acceptance_margins([0.01,0.01,0.01,0.0,0.0,10.0])
        self.docking.dp_client.goal.x_ref = self.docking.odom_pose
        self.docking.dp_client.send_goal()
