#!/usr/bin/env python

from enum import IntEnum

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

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


class ControlModeEnum(IntEnum):
    OPEN_LOOP = 0
    POSITION_HOLD = 1
    HEADING_HOLD = 2
    DEPTH_HEADING_HOLD = 3
    DEPTH_HOLD = 4
    POSITION_HEADING_HOLD = 5
    CONTROL_MODE_END = 6
    POSE_HOLD = 7


class JoyGuidance:
    def __init__(
        self, guidance_interface, action_server_name, activate_joystick_service_name
    ):
        self.guidance_interface = guidance_interface

        # set up servers and clients
        rospy.logdebug("Waiting for joystick activation service..")
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
        rospy.logdebug("JoyGuidance initialized")

    def activate_joystick(self, activate):
        """Requests joystick activation/deactivation from joystick_guidance

        Args:
            activate (bool): True activates joystick, False deactivates
        """
        request = SetBoolRequest()
        request.data = activate

        try:
            self.activate_joystick_service(request)
            return True
        except rospy.ServiceException as exc:
            rospy.logerr(
                "Joystick activation service did not process request: " + str(exc)
            )
            return False

    def action_server_callback(self, control_mode_goal):
        self.guidance_interface.stop_all_guidance()

        self.guidance_interface.change_dp_control_mode(
            control_mode_goal.controlModeIndex
        )

        if self.activate_joystick(True):
            self.joystick_controlmode_server.set_succeeded()
        else:
            self.joystick_controlmode_server.set_aborted()


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

        # set up servers and clients
        rospy.logdebug("Waiting for set velocity service..")
        rospy.wait_for_service(set_velocity_service_name)
        self.set_velocity_service = rospy.ServiceProxy(
            set_velocity_service_name, SetVelocity
        )
        rospy.logdebug("Waiting for stop vel guidance service..")
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
        rospy.logdebug("VelocityGuidance initialized")

    def action_server_callback(self, set_velocity_goal):
        self.guidance_interface.stop_all_guidance()

        request = SetVelocityRequest()
        request.desired_velocity = set_velocity_goal.desired_velocity

        try:
            self.set_velocity_service(request)
            self.action_server.set_succeeded()
        except rospy.ServiceException as exc:
            rospy.logerr("Set velocity service did not process request: " + str(exc))
            self.action_server.set_aborted()

    def stop(self):
        try:
            self.stop_guidance_service()
            rospy.logdebug("interface stopped vel guidance")
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
        self.max_duration = rospy.get_param("/guidance/interface/action_timeout", 90)
        self.timeout = False
        self.client_done = False

        # set up servers and clients
        rospy.logdebug("Waiting for dp control mode service..")
        rospy.wait_for_service(dp_controller_control_mode_service)
        self.control_mode_service = rospy.ServiceProxy(
            dp_controller_control_mode_service, ControlMode
        )
        rospy.logdebug("Connecting dp action client..")
        self.action_client = actionlib.SimpleActionClient(
            dp_guidance_action_server, MoveBaseAction
        )

        rospy.logdebug("Starting dp action server..")
        self.action_server = actionlib.SimpleActionServer(
            guidance_interface_dp_action_server,
            MoveBaseAction,
            self.dp_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.logdebug("DpGuidance initialized")

    def dp_callback(self, goal):
        self.guidance_interface.stop_all_guidance()
        self.change_control_mode(ControlModeEnum.POSITION_HEADING_HOLD)

        rospy.logdebug("Sending new goal to dp_guidance..")
        self.client_done =  False
        self.action_client.send_goal(
            goal, done_cb=self.guidance_finished_cb, feedback_cb=None
        )

        self.timeout = False
        rospy.Timer(rospy.Duration(self.max_duration), self.set_timeout, oneshot=True)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # preempt requested
            if self.action_server.is_preempt_requested():
                self.action_client.cancel_all_goals()
                self.action_server.set_preempted()
                break

            # client has finished its goal
            if self.client_done:
                state = self.action_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    self.action_server.set_succeeded()
                elif state == GoalStatus.PREEMPTED:
                    self.action_server.set_preempted()
                else:
                    self.action_server.set_aborted()
                break

            # timeout has occured
            if self.timeout:
                self.action_client.cancel_all_goals()
                self.action_server.set_aborted()
                rospy.logwarn("DP guidance aborted action due to timeout")
                break
            rate.sleep()


    def set_timeout(self):
        self.timeout = True
        
    def guidance_finished_cb(self, state, result):
        self.client_done = True
        

    def change_control_mode(self, control_mode):
        """Requests dp controller to change its control mode to the given mode

        Args:
            control_mode_index (int or bool): requested control mode
        """

        if issubclass(type(control_mode), ControlModeEnum):
            control_mode = control_mode.value  # since enum.field returns name and value
        else:
            # check if index is valid
            if control_mode not in [member.value for member in ControlModeEnum]:
                rospy.logerr(
                    "Invalid control mode %s requested. Ignoring." % control_mode
                )
                return

        request = ControlModeRequest()
        request.controlmode = control_mode

        try:
            self.control_mode_service(request)
        except rospy.ServiceException as exc:
            rospy.logerr("Control mode service did not process request: " + str(exc))

    def stop(self):
        if self.action_client.gh:   # goal handle. None if no goal exists.
            self.action_client.cancel_all_goals()
        self.change_control_mode(ControlModeEnum.OPEN_LOOP) 


class LosGuidance:
    def __init__(
        self,
        guidance_interface,
        los_guidance_action_server,
        guidance_interface_los_action_server,
    ):
        self.guidance_interface = guidance_interface
        self.max_duration = rospy.get_param("/guidance/interface/action_timeout", 90)
        self.timeout = False
        self.client_done = False

        # set up servers and clients
        rospy.logdebug("Connecting los action client..")
        self.action_client = actionlib.SimpleActionClient(
            los_guidance_action_server, LosPathFollowingAction
        )
        rospy.logdebug("Starting los action server..")
        self.action_server = actionlib.SimpleActionServer(
            guidance_interface_los_action_server,
            LosPathFollowingAction,
            self.los_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.logdebug("LosGuidance initialized")

    def los_callback(self, goal):
        self.guidance_interface.stop_all_guidance()

        rospy.logdebug("Sending new goal to los_guidance..")
        self.client_done =  False
        self.action_client.send_goal(
            goal, done_cb=self.guidance_finished_cb, feedback_cb=None
        )

        self.timeout = False
        rospy.Timer(rospy.Duration(self.max_duration), self.set_timeout, oneshot=True)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # preempt requested
            if self.action_server.is_preempt_requested():
                self.action_client.cancel_all_goals()
                self.action_server.set_preempted()
                break

            # client has finished its goal
            if self.client_done:
                state = self.action_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    self.action_server.set_succeeded()
                elif state == GoalStatus.PREEMPTED:
                    self.action_server.set_preempted()
                else:
                    self.action_server.set_aborted()
                break

            # timeout has occured
            if self.timeout:
                self.action_client.cancel_all_goals()
                self.action_server.set_aborted()
                rospy.logwarn("LOS guidance aborted action due to timeout")
                break
            rate.sleep()


    def set_timeout(self):
        self.timeout = True
        
    def guidance_finished_cb(self, state, result):
        self.client_done = True

    def stop(self):
        if self.action_client.gh:   # goal handle. None if no goal exists.
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
            dp_guidance_action_server="dp_action_server",
            guidance_interface_dp_action_server="/guidance_interface/dp_server",
            dp_controller_control_mode_service="/controller/controlmode_service",
        )

        self.los_guidance = LosGuidance(
            guidance_interface=self,
            los_guidance_action_server="los_action_server",
            guidance_interface_los_action_server="/guidance_interface/los_server",
        )
        rospy.logdebug("GuidanceInterface initialized")

    def stop_all_guidance(self):
        self.joy_guidance.stop()
        self.vel_guidance.stop()
        self.dp_guidance.stop()
        self.los_guidance.stop()

    def change_dp_control_mode(self, control_mode_index):
        self.dp_guidance.change_control_mode(control_mode_index)


if __name__ == "__main__":
    rospy.init_node("guidance_interface", log_level=rospy.DEBUG)
    server = GuidanceInterface()
    rospy.spin()
