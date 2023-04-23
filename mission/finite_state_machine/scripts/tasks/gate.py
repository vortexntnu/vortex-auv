import rospy
import smach
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from landmarks.srv import request_position
import actionlib
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    DpSetpoint,
    ObjectPosition,
)
from nav_msgs.msg import Odometry

from fsm_helper import (
    rotate_certain_angle,
    within_acceptance_margins,
)
from vortex_msgs.srv import SetVelocity

from search.forward_sweep import ForwardSweepSearch


class GateSearch(smach.State):

    def __init__(self):
        self.task = "gate"

        smach.State.__init__(self,
                             outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(self.task).object

        self.landmarks_pub = rospy.Publisher("/fsm/object_positions_in",
                                             ObjectPosition,
                                             queue_size=1)

        self.recov_point = self.landmarks_client("recovery_point").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server,
                                                       VtfPathFollowingAction)

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

            while (self.vtf_client.simple_state
                   != actionlib.simple_action_client.SimpleGoalState.DONE):
                rate.sleep()

        self.recov_point.objectPose.pose.position = self.odom.pose.pose.position
        self.recov_point.isDetected = True
        self.landmarks_pub.publish(self.recov_point)

        self.object.estimateFucked = False
        self.landmarks_pub.publish(self.object)

        if rospy.get_param("/fsm/do_coinflip"):
            pass

        # This currently blocks until an object is detected. However, we should have a timeout
        # which will set object.isFucked to True and reset this.
        object_found = self.search_pattern.run()

        if object_found:
            return "succeeded"
        else:
            return "aborted"


class GateConverge(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["preempted", "succeeded", "aborted"],
            output_keys=["gate_converge_output"],
        )

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("gate").object

        self.dp_pub = rospy.Publisher("/controllers/dp_data",
                                      DpSetpoint,
                                      queue_size=1)
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server,
                                                       VtfPathFollowingAction)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("gate/converge")

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("gate").object
        goal_pose = self.object.objectPose.pose
        goal_pose.position.x -= 0.2
        goal_pose.position.z = rospy.get_param("/fsm/operating_depth")
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
            if (self.vtf_client.simple_state ==
                    actionlib.simple_action_client.SimpleGoalState.DONE):
                break
            self.object = self.landmarks_client("gate").object
            # goal.waypoints = [self.object.objectPose.pose.position]

            print("GATE POSITION DETECTED:"
                  f"{goal.waypoints[0].x}, "
                  f"{goal.waypoints[0].y}, "
                  f"{goal.waypoints[0].z}")

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
            rospy.loginfo("WAITING FOR GATE GMF TO CONVERGE")
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
        print("GATE POSITION ESTIMATE CONVERGED AT:"
              f"{self.object.objectPose.pose.position.x}, "
              f"{self.object.objectPose.pose.position.y}, "
              f"{self.object.objectPose.pose.position.z}")

        return "succeeded"


class GateExecute(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["preempted", "succeeded", "aborted"],
                             input_keys=["gate"])

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server,
                                                       VtfPathFollowingAction)

        desired_velocity_topic = rospy.get_param(
            "/controllers/velocity_controller/desired_velocity_topic")
        self.velocity_ctrl_client = rospy.ServiceProxy(desired_velocity_topic,
                                                       SetVelocity)
        rospy.wait_for_service(desired_velocity_topic)

        self.dp_pub = rospy.Publisher("/controllers/dp_data",
                                      DpSetpoint,
                                      queue_size=1)
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("gate/execute")
        goal = VtfPathFollowingGoal()
        # goal_pose = get_pose_in_front(userdata.gate.objectPose.pose, 0.5, 0)
        goal_pose = userdata.gate.objectPose.pose
        # AUV needs to be aligned with where we wish to go through
        goal_pose.position.x += 0.5
        goal_pose.position.z = rospy.get_param("/fsm/operating_depth")
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()

        rospy.loginfo("MOVING THROUGH GATE")
        while not rospy.is_shutdown():
            if (self.vtf_client.simple_state ==
                    actionlib.simple_action_client.SimpleGoalState.DONE):
                break
            rate.sleep()

        starting_pose = self.odom.pose.pose

        rate = rospy.Rate(100)
        rospy.loginfo("PERFORMING ACROBATICS")
        for i in range(8):
            rospy.loginfo(f"{i+1} out of 8 90 degree turns!")
            goal = Pose()
            goal.position = self.odom.pose.pose.position
            goal.orientation = self.odom.pose.pose.orientation
            # Cursed below, to ensure that the AUV rotates the full 720 degrees
            angle = 90
            if i == 7:
                angle += 60
            goal = rotate_certain_angle(goal, angle)
            vel_goal = Twist()
            vel_goal.angular.z = rospy.get_param("/fsm/turn_speed")
            self.velocity_ctrl_client(vel_goal, True)

            # vel_goal.linear.z = z_compensation  # should be ommited if drone is balanced and level underwater
            # vel_goal.linear.x = 0.01  # should be ommited if drone is balanced and level underwater. Same other places.

            while not within_acceptance_margins(goal, self.odom, True):
                rate.sleep()
            self.velocity_ctrl_client(vel_goal, False)

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 0  # OPEN_LOOP
        self.dp_pub.publish(dp_goal)

        rospy.loginfo("REALIGNING")
        goal = VtfPathFollowingGoal()
        goal_pose = starting_pose
        goal_pose.position.x += 0.3
        goal_pose.position.y = 0
        goal.heading_point.x = 100
        goal.heading_point.y = 0

        goal.waypoints = [goal_pose.position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "point_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(5)
        rate.sleep()
        while not rospy.is_shutdown():
            if (self.vtf_client.simple_state ==
                    actionlib.simple_action_client.SimpleGoalState.DONE):
                break
            rate.sleep()

        return "succeeded"
