from turtle import forward
import rospy
import smach
from smach import StateMachine, State
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
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
from fsm_helper import (
    get_pose_in_front,
    get_position_on_line,
    rotate_certain_angle,
    within_acceptance_margins,
)

from vortex_msgs.srv import ControlMode, SetVelocity

forward_direction = 0 # 0 = x, 1 = y
z_compensation = -0.005

class BuoySearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("buoy").object

        self.recov_point = self.landmarks_client("recovery_point").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

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


        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("buoy/search")
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

        self.init_pose = self.odom.pose.pose
        
        path_segment_counter = 1
        while not self.object.isDetected:

            # SEARCH PATTERN
            goal = VtfPathFollowingGoal()
            goal.waypoints = [get_pose_in_front(self.init_pose, path_segment_counter, forward_direction).position]
            path_segment_counter += 1
            goal.waypoints[0].z = rospy.get_param("/fsm/operating_depth")
            goal.forward_speed = rospy.get_param("/fsm/medium_speed")
            goal.heading = "path_dependent_heading"
            self.vtf_client.wait_for_server()
            self.vtf_client.send_goal(goal)
            while (
                self.vtf_client.simple_state
                != actionlib.simple_action_client.SimpleGoalState.DONE
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("buoy").object
                print("SEARCHING FOR BUOY ...")
                rate.sleep()
            if self.object.isDetected:
                break

            goal = Pose()
            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, 60)
            vel_goal = Twist()
            vel_goal.angular.z = rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = z_compensation  # should be ommited if drone is balanced and level underwater
            vel_goal.linear.x = 0.01  # should be ommited if drone is balanced and level underwater. Same other places.
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("buoy").object
                print("SEARCHING FOR BUOY ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, -120)
            vel_goal = Twist()
            vel_goal.angular.z = -rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = z_compensation
            vel_goal.linear.x = 0.01
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("buoy").object
                print("SEARCHING FOR BUOY ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            goal = rotate_certain_angle(goal, 60)
            vel_goal = Twist()
            vel_goal.angular.z = rospy.get_param("/fsm/turn_speed")
            vel_goal.linear.z = z_compensation
            vel_goal.linear.x = 0.01
            self.velocity_ctrl_client(vel_goal, True)
            while (
                not within_acceptance_margins(goal, self.odom, True)
                and not self.object.isDetected
            ):
                self.object = self.landmarks_client("buoy").object
                print("SEARCHING FOR BUOY ...")
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)
            if self.object.isDetected:
                break

            print("SEARCHING FOR BUOY ...")
            rospy.wait_for_service("send_positions")
            self.object = self.landmarks_client("buoy").object
            rate.sleep()

        self.vtf_client.cancel_all_goals()

        print(
            "BUOY POSITION DETECTED: "
            + str(self.object.objectPose.pose.position.x)
            + ", "
            + str(self.object.objectPose.pose.position.y)
            + ", "
            + str(self.object.objectPose.pose.position.z)
        )
        return "succeeded"


class BuoyConverge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["preempted", "succeeded", "aborted"],
            output_keys=["buoy_converge_output"],
        )

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("buoy").object

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
        self.state_pub.publish("buoy/converge")

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("buoy").object
        goal_pose = get_pose_in_front(self.object.objectPose.pose, -0.75, forward_direction)
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
            self.object = self.landmarks_client("buoy").object
            # goal.waypoints = [self.object.objectPose.pose.position]
            print(
                "BUOY POSITION DETECTED: "
                + str(goal.waypoints[0].x)
                + ", "
                + str(goal.waypoints[0].y)
                + ", "
                + str(goal.waypoints[0].z)
            )
            # goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            # self.vtf_client.send_goal(goal)
            userdata.buoy_converge_output = self.object
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
            self.object = self.landmarks_client("buoy").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0  # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return "aborted"
            rate.sleep()
        dp_goal = DpSetpoint()
        dp_goal.control_mode = 0  # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("buoy").object
        userdata.buoy_converge_output = self.object
        print(
            "BUOY POSITION ESTIMATE CONVERGED AT: "
            + str(self.object.objectPose.pose.position.x)
            + "; "
            + str(self.object.objectPose.pose.position.y)
            + "; "
            + str(self.object.objectPose.pose.position.z)
        )

        return "succeeded"

class BuoyExecute(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["preempted", "succeeded", "aborted"], input_keys=["buoy"]
        )

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        desired_velocity_topic = rospy.get_param(
            "/controllers/velocity_controller/desired_velocity_topic"
        )
        self.velocity_ctrl_client = rospy.ServiceProxy(
            desired_velocity_topic, SetVelocity
        )
        rospy.wait_for_service(desired_velocity_topic)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg


    # TODO: Align orientation with DP
    def execute(self, userdata):
        starting_pose = self.odom.pose.pose


        rospy.loginfo("TOUCHING BUOY")
        goal = VtfPathFollowingGoal()
        goal_pose = get_pose_in_front(userdata.buoy.objectPose.pose, 0.1, forward_direction)
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "point_dependent_heading"
        goal.heading_point.x = goal_pose.position.x
        goal.heading_point.y = goal_pose.position.y

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        touching_threshold = 20 # seconds, TODO: tune
        starting_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                break

            if (rospy.Time.now().to_sec() - starting_time) > touching_threshold:
                rospy.loginfo("Touching buoy time threshold reached!")
                self.vtf_client.cancel_all_goals()
                break

            rate.sleep()

        rospy.loginfo("GOING BACK")
        goal = VtfPathFollowingGoal()
        goal.waypoints = [starting_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "point_dependent_heading"
        goal.heading_point.x = goal_pose.position.x
        goal.heading_point.y = goal_pose.position.y

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                break
            rate.sleep()

        rospy.loginfo("REALIGNING")
        goal = VtfPathFollowingGoal()
        goal_pose = starting_pose
        if forward_direction == 0:
            goal_pose.position.y = 0
            goal.heading_point.x = 100
            goal.heading_point.y =  0
        elif forward_direction == 1:
            goal_pose.position.x = 0
            goal.heading_point.x = 0
            goal.heading_point.y = 100
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "point_dependent_heading"

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
