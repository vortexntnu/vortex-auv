import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joynav
from nav_msgs.msg import Odometry



class Set_depth(smach_state):
    def __init__(self):
        smach.State.__init__(self, outcomes={'submerged'},output_keys={})
        self.sub_depth = rospy.Subscriber('/odometry/filtered',
                                          Odometry, self.callback,queue_size=1)
        pub = rospy.Publisher('desired_depth', std_msgs.msg.float, queue_size=1)
        

    def callb)ack(self, msg):
        self.current_depth = msg.pose.pose.position.z

    def execute(self):
        pub.publish(std_msgs.msg.float(3.))
        rospy.loginfo('Searching')
        rosrun depth_hold depth_hold_node()
        while self.current_depth != self.desired_depth
            return'submerging'
        return'submerged


class Move(smach_state):
    def __init__(self):
        smach.State.__init__(self,outcome={'movement_complete'},output_keys={})
        self.sub_pos = rospy.Subscriber('/odometry/filtered',
                                          Odometry, self.callback,queue_size=1)
        pub = rospy.Publisher('desired_depth', std_msgs.msg.float, queue_size=1)

        self.desired_x = 1
        self.desired_y = 2

    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
    def execute(self):
        pub.publish(std_msg.msg.float((self.desired_x,self.desired_y))
        roslaunch.MoveX(self.desired_x)
        roslaunch.MoveY(self.desired_y)
        while (self.current_x, self.current_y) != (self.desired_x, self.desired_y)
            return'moving'
        return'movement_complete'


class Surface(smach_state):
    def __init__(self):
        smach.State.__init__(self.outcomes={'surfaced'})
        def execute(self):
            #disengage all nodes
        return'surfaced'

def main():
    rospy.init_node("test_machine_node")
    sm = smach.StateMachine(outcomes = {'Done'})

    sis = smach_ros.IntrospectionServer('Test_machine_server', sm, '/SM_ROOT')
    sis.start()

    with sm:
        smach.StateMachine.add('Set_depth', Set_depth(),
                                transitions={'submerged':'Move'}
        smach.StateMachine.add('Move', Move(),
                                transitions={'movement_complete':'Surface'})
        smach-StateMachine.add('Surface', Surface(),
                                transitions = {'Surfaced':'Done'}