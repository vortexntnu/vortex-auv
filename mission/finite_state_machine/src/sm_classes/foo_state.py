import rospy
import smach


class FooState(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['foo_state_input','foo_ctrl_name'])

    def execute(self,userdata):
        rospy.loginfo("executing foo state")
        pose = userdata.foo_state_input
        ctrl_name = userdata.foo_ctrl_name
        #2410 rospy.loginfo("foo class received %f,%f,%f", pos.x,pos.y,pos.z)
        rospy.loginfo("foo class received %f,%f,%f", pose.position.x,pose.position.y,pose.position.z)
        rospy.loginfo("foo class received " + ctrl_name)
        return 'succeeded'