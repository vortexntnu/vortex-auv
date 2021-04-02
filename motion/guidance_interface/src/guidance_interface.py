#!/usr/bin/env python

from enum import IntEnum

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import Empty, SetBool

from vortex_msgs.msg import (
    LosPathFollowingAction,
    ControlModeAction,
    SetVelocityAction,
)

from vortex_msgs.srv import (
    ControlMode,
    ControlModeRequest,
    SetVelocity,
    SetVelocityRequest,
)


class ControlMode(IntEnum):
    OPEN_LOOP = 0
    POSE_HOLD = 1
    HEADING_HOLD = 2
    DEPTH_HEADING_HOLD = 3
    DEPTH_HOLD = 4
    POSE_HEADING_HOLD = 5
    CONTROL_MODE_END = 6


class JoyGuidance:
    def __init__(
        self, guidance_interface, action_server_name, activate_joystick_service_name
    ):
        self.guidance_interface = guidance_interface

        # set up servers and clients
#BUG: Services hang indefinitely, preventing any other code execution
        rospy.wait_for_service(activate_joystick_service_name)
        self.activate_joystick_service = rospy.ServiceProxy(
            activate_joystick_service_name, SetBool
        )
        self.joystick_controlmode_server = actionlib.SimpleActionServer(
            action_server_name,
            ControlModeAction,
            self.action_server_callback,
            auto_start=False,
        )
        self.joystick_controlmode_server.start()

    def activate_joystick(self, activate):
        """Requests joystick activation/deactivation from joystick_guidance

        Args:
            activate (bool): True activates joystick, False deactivates
        """
        request = SetBool()
        request.data = activate

#BUG: Services hang indefinitely, preventing any other code execution
        try:
            self.activate_joystick_service(request)
            rospy.loginfo("activate joystick service called")
        except rospy.ServiceException as exc:
            rospy.logerr(
                "Joystick activation service did not process request: " + str(exc)
            )

    def action_server_callback(self, control_mode_goal):
        self.guidance_interface.stop_all_guidance()

        self.guidance_interface.change_dp_control_mode(
            control_mode_goal.controlModeIndex
        )
        self.activate_joystick(True)

    def stop(self):
        self.activate_joystick(False)


class VelocityGuidance:
    def __init__(
        self,
        guidance_interface,
        set_velocity_service_name,
        stop_guidance_service_name,
        action_server_name,
    ):
        self.guidance_interface = guidance_interface

#BUG: Services hang indefinitely, preventing any other code execution
        # set up servers and clients
        rospy.wait_for_service(set_velocity_service_name)
        self.set_velocity_service = rospy.ServiceProxy(
            set_velocity_service_name, SetVelocity
        )
        rospy.wait_for_service(stop_guidance_service_name)
        self.stop_guidance_service = rospy.ServiceProxy(
            stop_guidance_service_name, Empty
        )

        self.action_server = actionlib.SimpleActionServer(
            action_server_name,
            SetVelocityAction,
            self.action_server_callback,
            auto_start=False,
        )
        self.action_server.start()

    def action_server_callback(self, set_velocity_goal):
        self.guidance_interface.stopAllGuidance()

        request = SetVelocityRequest()
        request.desired_velocity = set_velocity_goal.desired_velocity

        try:
            self.set_velocity_service(request)
        except rospy.ServiceException as exc:
            rospy.logerr("Set velocity service did not process request: " + str(exc))

    def stop(self):
        try:
            #self.stop_guidance_service()
            rospy.loginfo("vel guidance stopped!")
        except rospy.ServiceException as exc:
            rospy.logerr(
                "Stop velocity guidance service did not process request: " + str(exc)
            )


class DpGuidance:
    def __init__(
        self,
        guidance_interface,
        dp_guidance_action_server,
        guidance_interface_dp_action_server,
        dp_controller_control_mode_service,
    ):
        self.guidance_interface = guidance_interface
        self.timeout = rospy.get_param("/guidance/interface/action_timeout", 90)

        # set up servers and clients

#BUG: Services hang indefinitely, preventing any other code execution
        rospy.wait_for_service(dp_controller_control_mode_service)
        
        self.control_mode_service = rospy.ServiceProxy(
            dp_controller_control_mode_service, ControlMode
        )
        self.action_client = actionlib.SimpleActionClient(
            dp_guidance_action_server, MoveBaseAction
        )

#BUG: Potentially: MoveBaseAction should be MoveAction if fsm_helper() does not change
        self.action_server = actionlib.SimpleActionServer(
            guidance_interface_dp_action_server,
            MoveBaseAction,
            self.dp_callback,
            auto_start=False,
        )
        self.action_server.start()

    def dp_callback(self, goal):
        self.guidance_interface.stop_all_guidance()

        self.change_control_mode(ControlMode.POSE_HOLD)

#BUG: Exception in your execute callback: 'Pose' object has no attribute 'header'
        self.action_client.send_goal(
            goal, done_cb=self.guidance_finished_cb, feedback_cb=None
        )

        if not self.action_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
            self.action_server.set_aborted()
            rospy.loginfo("DP guidance aborted action due to timeout")

    def guidance_finished_cb(self, state, result):
        """
        Set the outcome of the action depending on
        the returning result.
        """

        if state == GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded()

        elif state == GoalStatus.PREEMPTED:
            self.action_server.set_preempted()

        else:
            self.action_server.set_aborted()

    def change_control_mode(self, control_mode_index):
        """Requests dp controller to change its control mode to the given mode

        Args:
            control_mode_index (int or bool): requested control mode
        """


#BUG: [ERROR] [1617332877.287364] [/guidance/interface]: Exception in your execute callback: issubclass() arg 1 must be a class
        if issubclass(control_mode_index, ControlMode):
            control_mode_index = (
                control_mode_index.value
            )  # since enum.field returns name and value
            rospy.loginfo("Control mode changed")
        else:
            # check if index is valid
            if control_mode_index not in [member.value for member in ControlMode]:
                rospy.logerr(
                    "Invalid control mode %s requested. Ignoring." % control_mode_index
                )
                return


#BUG: [ERROR] [1617332984.272044] [/guidance/interface]: Exception in your execute callback: 'ControlModeAction' object has no attribute 'controlModeIndex'
        request = ControlModeRequest()
        request.controlModeIndex = control_mode_index

        try:
            self.control_mode_service(request)
        except rospy.ServiceException as exc:
            rospy.logerr("Control mode service did not process request: " + str(exc))

    def stop(self):
        self.action_client.cancel_all_goals()
        self.change_control_mode(ControlMode.OPEN_LOOP)


class LosGuidance:
    def __init__(
        self,
        guidance_interface,
        los_guidance_action_server,
        guidance_interface_los_action_server,
    ):
        self.guidance_interface = guidance_interface
        self.timeout = rospy.get_param("/guidance/interface/action_timeout", 90)

        # set up servers and clients
        self.action_client = actionlib.SimpleActionClient(
            los_guidance_action_server, LosPathFollowingAction
        )

#BUG: Potentially: LosPathFollowingAction should be MoveAction if fsm_helper() does not change
        self.action_server = actionlib.SimpleActionServer(
            guidance_interface_los_action_server,
            LosPathFollowingAction,
            self.los_callback,
            auto_start=False,
        )
        self.action_server.start()

    def los_callback(self, goal):
        self.guidance_interface.stop_all_guidance()

        self.action_client.send_goal(
            goal, done_cb=self.guidance_finished_cb, feedback_cb=None
        )

        if not self.action_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
            self.action_server.set_aborted()
            rospy.loginfo("LOS guidance aborted action due to timeout")

    def guidance_finished_cb(self, state, result):
        """
        Set the outcome of the action depending on
        the returning result.
        """

        if state == GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded()

        elif state == GoalStatus.PREEMPTED:
            self.action_server.set_preempted()

        else:
            self.action_server.set_aborted()

    def stop(self):
        self.action_client.cancel_all_goals()


class GuidanceInterface:
    def __init__(self):
        
        self.joy_guidance = JoyGuidance(
            guidance_interface=self,
            action_server_name="/guidance_interface/joystick_server",
            activate_joystick_service_name="/joystick_guidance/activate_joystick_control",
        )

        self.vel_guidance = VelocityGuidance(
            guidance_interface=self,
            set_velocity_service_name="/vel_guidance/set_velocity",
            stop_guidance_service_name="/vel_guidance/stop_guidance",
            action_server_name="/guidance_interface/vel_server",
        )

        self.dp_guidance = DpGuidance(
            guidance_interface=self,
            dp_guidance_action_server='dp_action_server',
            guidance_interface_dp_action_server="/guidance_interface/dp_server",
            dp_controller_control_mode_service="/controller/controlmode_service",
        )

        self.los_guidance = LosGuidance(
            guidance_interface=self,
            los_guidance_action_server="los_action_server",
            guidance_interface_los_action_server="/guidance_interface/los_server",
        )

    def stop_all_guidance(self):
        self.joy_guidance.stop()
        self.vel_guidance.stop()
        self.dp_guidance.stop()
        self.los_guidance.stop()

    def change_dp_control_mode(self, control_mode_index):
        self.dp_guidance.change_control_mode(control_mode_index)


if __name__ == "__main__":
    rospy.init_node("guidance_interface")
    server = GuidanceInterface()
    rospy.spin()
