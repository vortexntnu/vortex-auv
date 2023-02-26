import rospy
import smach
import actionlib
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    DpSetpoint,
    ObjectPosition,
)
from landmarks.srv import request_position
from vortex_msgs.srv import SetVelocity

from search.forward_sweep import ForwardSweepSearch

forward_direction = 0  # 0 = x, 1 = y
z_compensation = -0.015


class BuoySearch(smach.State):
    def __init__(self):
        self.task = "buoy"
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(self.task).object

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

        self.search_pattern = ForwardSweepSearch(self.task)

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish(f"{self.task}/search")
        rate = rospy.Rate(10)

        # RECOVERY
        if self.recov_point.isDetected:
            print(f"No {self.task} found, recovering...")
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
                rate.sleep()

        self.recov_point.objectPose.pose.position = self.odom.pose.pose.position
        self.recov_point.isDetected = True
        self.landmarks_pub.publish(self.recov_point)

        self.object.estimateFucked = False
        self.landmarks_pub.publish(self.object)

        # This currently blocks until an object is detected. However, we should have a timeout
        # which will set object.isFucked to True and reset this.
        object_found = self.search_pattern.run()

        if object_found:
            print(
                f"{self.task} POSITION DETECTED:"
                f"{self.object.objectPose.pose.position.x}, "
                f"{self.object.objectPose.pose.position.y}, "
                f"{self.object.objectPose.pose.position.z}"
            )

            return "succeeded"
        else:
            return "aborted"


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
        goal_pose = self.object.objectPose.pose
        goal_pose.position.x -= 0.75

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
        starting_time = rospy.Time.now().to_sec()
        converging_threshold = 60
        while not rospy.is_shutdown() and not self.object.estimateConverged:
            if (rospy.Time.now().to_sec() - starting_time) > converging_threshold:
                rospy.loginfo("FAILED TO CONVERGE ON BUOY! TRYING EXECUTE!")
                out = self.landmarks_client("buoy").object
                out.objectPose = self.odom.pose.pose
                userdata.buoy_converge_output = out
                break

            rospy.loginfo("WAITING FOR BUOY GMF TO CONVERGE")
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
        # goal_pose = get_pose_in_front(userdata.buoy.objectPose.pose, 0.1, 0)
        goal_pose = userdata.buoy.objectPose.pose
        goal_pose.position.x += 0.05

        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "point_dependent_heading"
        goal.heading_point.x = goal_pose.position.x
        goal.heading_point.y = goal_pose.position.y

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        touching_threshold = 8  # seconds, TODO: tune
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
        goal_pose.position.y = 0
        goal.heading_point.x = 100
        goal.heading_point.y = 0
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
