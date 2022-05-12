import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    SetVelocityGoal,
    SetVelocityAction,
    DpSetpoint,
    ObjectPosition,
)
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler
from fsm_helper import (
    dp_move,
    get_pose_in_front,
    rotate_certain_angle,
    within_acceptance_margins,
)
from vortex_msgs.srv import ControlMode, SetVelocity


class GateSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("gate").object

        desired_velocity_topic = rospy.get_param(
            "/controllers/velocity_controller/desired_velocity_topic"
        )
        self.velocity_ctrl_client = rospy.ServiceProxy(
            desired_velocity_topic, SetVelocity
        )
        rospy.wait_for_service(desired_velocity_topic)

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)

        self.landmarks_pub = rospy.Publisher(
            "/fsm/object_positions_in", ObjectPosition, queue_size=1
        )

        self.recov_point = self.landmarks_client("recovery_point").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("gate_search")
        rate = rospy.Rate(10)

        # RECOVERY
        if self.recov_point.isDetected:
            goal = VtfPathFollowingGoal()
            goal.waypoints = [self.recov_point.objectPose.pose.position]
            goal.forward_speed = rospy.get_param("/fsm/fast_speed")
            goal.heading = "path_dependent_heading"

            self.vtf_client.wait_for_server()
            self.vtf_client.send_goal(goal)

            while (
                self.vtf_client.simple_state
                != actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                print("Recovering")
                rate.sleep()

        self.recov_point.objectPose.pose.position = self.odom.pose.pose.position
        self.recov_point.isDetected = True
        self.landmarks_pub.publish(self.recov_point)

        self.object.estimateFucked = False
        self.landmarks_pub.publish(self.object)

        while not self.object.isDetected:

            # SEARCH PATTERN
            goal = VtfPathFollowingGoal()
            goal.waypoints = [get_pose_in_front(self.odom.pose.pose, 1).position]
            goal.waypoints[0].z = -1.1
            goal.forward_speed = rospy.get_param("/fsm/medium_speed")
            goal.heading = "path_dependent_heading"
            self.vtf_client.wait_for_server()
            self.vtf_client.send_goal(goal)
            while (
                self.vtf_client.simple_state
                != actionlib.simple_action_client.SimpleGoalState.DONE
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("gate").object
                print("SEARCHING FOR GATE ...")
                rate.sleep()
            if self.object.isDetected:
                break

            goal = Pose()
            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, 45)
            vel_goal = Twist()
            vel_goal.angular.z = rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = (
                -0.01
            )  # should be ommited if drone is balanced and level underwater
            vel_goal.linear.x = 0.01  # should be ommited if drone is balanced and level underwater. Same other places.
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("gate").object
                print("SEARCHING FOR GATE ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, -90)
            vel_goal = Twist()
            vel_goal.angular.z = -rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = -0.01
            vel_goal.linear.x = 0.01
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("gate").object
                print("SEARCHING FOR GATE ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, 45)
            vel_goal = Twist()
            vel_goal.angular.z = rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = -0.01
            vel_goal.linear.x = 0.01
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("gate").object
                print("SEARCHING FOR GATE ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            print("SEARCHING FOR GATE ...")
            rospy.wait_for_service("send_positions")
            self.object = self.landmarks_client("gate").object
            rate.sleep()

        self.vtf_client.cancel_all_goals()

        print(
            "GATE POSITION DETECTED: "
            + str(self.object.objectPose.pose.position.x)
            + ", "
            + str(self.object.objectPose.pose.position.y)
            + ", "
            + str(self.object.objectPose.pose.position.z)
        )
        return "succeeded"


class GateConverge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["preempted", "succeeded", "aborted"],
            output_keys=["gate_converge_output"],
        )

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("gate").object

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("gate_converge")

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("gate").object
        goal_pose = get_pose_in_front(self.object.objectPose.pose, 0.5)
        print("get_pose_in_front returned:")
        print(goal_pose)
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/fast_speed")
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        # TODO: The commented out code below should be there.
        # However, the VTF action server prematurely finishes when it is. Investigate this.
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                break
            self.object = self.landmarks_client("gate").object
            # goal.waypoints = [self.object.objectPose.pose.position]
            print(
                "GATE POSITION DETECTED: "
                + str(goal.waypoints[0].x)
                + ", "
                + str(goal.waypoints[0].y)
                + ", "
                + str(goal.waypoints[0].z)
            )
            # goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            # self.vtf_client.send_goal(goal)
            userdata.gate_converge_output = self.object
            rate.sleep()

            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return "aborted"
        self.vtf_client.cancel_all_goals()

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7  # POSE_HOLD
        dp_goal.setpoint = self.odom.pose.pose
        self.dp_pub.publish(dp_goal)
        while not rospy.is_shutdown() and not self.object.estimateConverged:
            print("in dp hold")
            self.object = self.landmarks_client("gate").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0  # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return "aborted"
            rate.sleep()
        dp_goal = DpSetpoint()
        dp_goal.control_mode = 0  # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("gate").object
        userdata.gate_converge_output = self.object
        print(
            "GATE POSITION ESTIMATE CONVERGED AT: "
            + str(self.object.objectPose.pose.position.x)
            + "; "
            + str(self.object.objectPose.pose.position.y)
            + "; "
            + str(self.object.objectPose.pose.position.z)
        )

        return "succeeded"


class GateExecute(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["preempted", "succeeded", "aborted"], input_keys=["gate"]
        )

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

    def execute(self, userdata):
        self.state_pub.publish("gate_execute")
        goal = VtfPathFollowingGoal()
        goal_pose = get_pose_in_front(userdata.gate.objectPose.pose, -0.5)
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                break
            rate.sleep()

        return "succeeded"
