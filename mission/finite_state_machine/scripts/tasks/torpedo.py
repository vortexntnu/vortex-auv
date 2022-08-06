import rospy
import smach
import actionlib
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    DpSetpoint,
    ObjectPosition,
)
from landmarks.srv import request_position
from fsm_helper import within_acceptance_margins
from search.forward_sweep import ForwardSweepSearch

class TorpedoSearch(smach.State):
    def __init__(self):
        self.task = "torpedo_poster"
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
            print(f"{self.task} POSITION DETECTED:" 
                f"{self.object.objectPose.pose.position.x}, "
                f"{self.object.objectPose.pose.position.y}, "
                f"{self.object.objectPose.pose.position.z}"
            )

            return "succeeded"
        else:
            return "aborted"


class TorpedoConverge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["preempted", "succeeded", "aborted"],
            output_keys=["torpedo_converge_output"],
        )

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("torpedo_poster").object

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
        self.state_pub.publish("torpedo/converge")

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("torpedo_poster").object
        rospy.loginfo(self.object.objectPose.pose)
        #goal_pose = get_pose_in_front(self.object.objectPose.pose, -0.75, 0)
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
            self.object = self.landmarks_client("torpedo_poster").object
            # goal.waypoints = [self.object.objectPose.pose.position]
            print(
                "TORPEDO POSITION DETECTED: "
                + str(goal.waypoints[0].x)
                + ", "
                + str(goal.waypoints[0].y)
                + ", "
                + str(goal.waypoints[0].z)
            )
            # goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            # self.vtf_client.send_goal(goal)
            userdata.torpedo_converge_output = self.object
            rate.sleep()

            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return "aborted"
        self.vtf_client.cancel_all_goals()

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7  # POSE_HOLD
        dp_goal.setpoint = self.odom.pose.pose


        # At this point, we switch from converging on the torpedo_poster to the torpedo_target

        self.object = self.landmarks_client("torpedo_target").object
        self.dp_pub.publish(dp_goal)
        while not rospy.is_shutdown() and not self.object.estimateConverged:
            rospy.loginfo("WAITING FOR TORPEDO GMF TO CONVERGE")
            self.object = self.landmarks_client("torpedo_target").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0  # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return "aborted"
            rate.sleep()
        dp_goal = DpSetpoint()
        dp_goal.control_mode = 0  # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("torpedo_target").object
        userdata.torpedo_converge_output = self.object
        print(
            "TORPEDO POSITION ESTIMATE CONVERGED AT: "
            + str(self.object.objectPose.pose.position.x)
            + "; "
            + str(self.object.objectPose.pose.position.y)
            + "; "
            + str(self.object.objectPose.pose.position.z)
        )

        return "succeeded"

class TorpedoExecute(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["preempted", "succeeded", "aborted"], input_keys=["torpedo"]
        )

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        self.fire = rospy.Publisher("/torpedo", Int32, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg


    # TODO: Smoother transition to alignign
    def execute(self, userdata):
        starting_pose = self.odom.pose.pose

        rospy.loginfo("ALIGNING WITH HOLE")
        #goal_pose = get_pose_in_front(userdata.torpedo.objectPose.pose, -0.43, 0) # 0.38 to torpedo, then a 5cm margin
        goal_pose = userdata.torpedo.objectPose.pose
        goal_pose.position.z += 0.145 # This is the offset between BODY and torpedo center in z
        goal_pose.position.x -= 0.43

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7  # POSE_HOLD
        dp_goal.setpoint = goal_pose
        self.dp_pub.publish(dp_goal)

        rate = rospy.Rate(5)
        while (
            not within_acceptance_margins(goal_pose, self.odom)
        ):
            rate.sleep()

        rospy.sleep(10) # Allow controller oscillations to settle
        rospy.loginfo("ALIGNED!")


        rospy.loginfo("FIRE 1!")
        self.fire.publish(Int32())
        rospy.sleep(1)
        rospy.loginfo("FIRE 2!")
        self.fire.publish(Int32())

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 0  # OPEN_LOOP
        self.dp_pub.publish(dp_goal)

        rospy.loginfo("REALIGNING")
        goal = VtfPathFollowingGoal()
        goal_pose = starting_pose
        goal_pose.position.y = 0
        goal.heading_point.x = 100
        goal.heading_point.y =  0
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
