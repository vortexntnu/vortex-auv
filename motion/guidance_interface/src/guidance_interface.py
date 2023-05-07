#!/usr/bin/python3

from enum import IntEnum, Enum

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest
from smach_ros import SimpleActionState

from vortex_msgs.msg import (
    LosPathFollowingAction,
    ControlModeAction,
    SetVelocityAction,
    MoveAction,
    LosPathFollowingGoal,
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
    ORIENTATION_HOLD = 8
    ORIENTATION_DEPTH_HOLD = 9


class GoalStatusEnum(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


class JoyGuidance:

    def __init__(self, guidance_interface, action_server_name,
                 activate_joystick_service_name):
        self.guidance_interface = guidance_interface

        # set up servers and clients
        rospy.logdebug("Waiting for joystick activation service..")
        rospy.wait_for_service(activate_joystick_service_name)
        self.activate_joystick_service = rospy.ServiceProxy(
            activate_joystick_service_name, SetBool)
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
                "Joystick activation service did not process request: " +
                str(exc))
            return False

    def action_server_callback(self, control_mode_goal):
        self.guidance_interface.stop_all_guidance()

        if self.activate_joystick(True):
            self.joystick_controlmode_server.set_succeeded()
        else:
            self.joystick_controlmode_server.set_aborted()

    def stop(self):
        self.activate_joystick(False)


class GuidanceInterface:

    def __init__(self):
        self.joy_guidance = JoyGuidance(
            guidance_interface=self,
            action_server_name="/guidance_interface/joystick_server",
            activate_joystick_service_name=
            "/joystick_guidance/activate_joystick_control",
        )

        rospy.logdebug("GuidanceInterface initialized")

    def stop_all_guidance(self):
        self.joy_guidance.stop()


if __name__ == "__main__":
    rospy.init_node("guidance_interface", log_level=rospy.INFO)
    server = GuidanceInterface()
    rospy.spin()
