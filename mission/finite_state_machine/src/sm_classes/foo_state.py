import rospy
import smach

class FooState(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['foo_state_input'])

    def execute(self,userdata):
        
        pos = userdata.foo_state_input

        rospy.loginfo("foo class received %f,%f,%f", pos.x,pos.y,pos.z)        
        
        return 'succeeded'