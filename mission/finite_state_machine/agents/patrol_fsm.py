#!/usr/bin/env	python
import	rospy
from	smach	import	StateMachine		
from	smach_ros	import	SimpleActionState, IntrospectionServer		
from	move_base_msgs.msg	import	MoveBaseAction,	MoveBaseGoal
from nav_msgs.msg import Odometry

waypoints	=	[
				['one',	(5.0,	0.0, -0.75),	(0.0,	0.0,	0.0,	1.0)],
				['two',	(10.0,	0.0, -0.75),	(0.0,	0.0,	-0.984047240305,	0.177907360295)]
]


class Mission():

    def __init__(self):
        
        # ros init
        rospy.init_node('mission_fsm', anonymous=True)

        # rate
        self.rate = rospy.Rate(100) #Hz

        # Subscriber
        #self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)

    #def positionCallback(self, msg):
        #pass

    # Checks the status message from the server
    def status_cb(self, status, result):
        print(status)

if __name__ == '__main__':

    try:
        mission = Mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("Unable to run constructor")

    patrol = StateMachine(['succeeded','aborted','preempted'])

    with patrol:
        for i, w in enumerate(waypoints):
            # object constructor
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = "map"
            goal_pose.target_pose.header.stamp = rospy.Time.now()

            # adding coordiantes
            goal_pose.target_pose.pose.position.x = w[1][0]
            goal_pose.target_pose.pose.position.y = w[1][1]
            goal_pose.target_pose.pose.position.z = w[1][2]

            # adding attitude
            goal_pose.target_pose.pose.orientation.x = w[2][0]
            goal_pose.target_pose.pose.orientation.y = w[2][1]
            goal_pose.target_pose.pose.orientation.z = w[2][2]
            goal_pose.target_pose.pose.orientation.w = w[2][3]

            # Adding the states and transitions
            StateMachine.add(w[0],
                             SimpleActionState('move_base',
                                            MoveBaseAction,
                                            goal=goal_pose),
                             transitions={'succeeded':waypoints[(i + 1) % len(waypoints)][0]})



    patrol.execute()
    rospy.spin()