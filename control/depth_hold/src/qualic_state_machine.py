import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import os
import smach
import smach_ros
from vortex_msgs.msg import PropulsionCommand, CameraObjectInfo
from sensor_msgs.msg import Joy



#Sketch of plan

'''
1 First we need to search to find the gate
    - Need topic from perception with coordinate in the camera
2 Once located, make a target point in the middle of the gate.
    - Make PI/PID tok keep the target in the center
3 Once you are too close so that you can't see the gate anymore, drive through it

'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Found','Not_Found'},output_keys={'Found_gate'})
        self.motion_search = [0,0,0,0,0,0.2]
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info', CameraObjectInfo, 
                                            self.execute, bool,queue_size=1000)
        
        self.x_pos = None
        self.y_pos = None

    def execute(self, msg):
        rospy.loginfo('Searching')
        self.x_pos = msg.x_pos
        self.y_pos = msg.y_pos
        coordinate = [self.x_pos, self.y_pos]

        while coordinate == [-1,-1]:
            #Crappy search pattern for gate
            #Will be massivly improved
            self.pub_motion(self.motion_search)

        return 'Found', 'Found_gate'

class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Centered','Lost'})
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',CameraObjectInfo, 
                                            self.execute, bool,queue_size=1000)
        self.x_pos = None
        self.y_pos = None
        self.gate_lost = [-1,-1]
        
    def execute(self, msg):
        rospy.loginfo('Centering')
        rosrun camera_centering camera_centering_node #Centers object in camera frame
        if sub_gate == self.gate_lost:   #if we loose the gate
            return 'Lost'
        elif sub_gate  1 and gate_centered == 1:  #if we see the gate and it is in center
            return 'Centered'

class Drive(smach.State):
    def __init__(self)
        smach.State.__init__(self, outcomes={'Complete','Complete_bouy','Lost_boy', 'Lost_gate'}, input_keys={'Input_gate','Input_bouy'})
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)

    def execute(self):
        if 'Input_gate' == 1:
            while sub_gate == 1: #Gate is visable and centered
                roslaunch Drive()   #makes the drone move forward
                if gate_passed == 1: #The gate has been passed
                    rosnode kill -Camera_centering
                    return 'Complete'
                else: #Gate lost for some reason
                    return 'Lost'

        if 'Input_bouy' == 1:
            while sub_bouy == 1: #Gate is visable and centered
                roslaunch Drive()   #makes the drone move forward
                if bouy_passed == 1: #The gate has been passed
                    rosnode kill -Camera_centering
                    return 'Complete_bouy'
                else: #Gate lost for some reason
                    return 'Lost_bouy'

class Search_Bouy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Found_b'},output_keys={'Found_bouy'})
        self.motion_search = [0,0,0,0,0,0.2]
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_bouy = rospy.Subscriber('camera_object_info',      #Do we see the bouy?
                                          bool,queue_size=1000)

    def execute(self):
        rospy.loginfo('Searching_b')
        while self.sub_bouy == None:
            #Crappy search pattern for gate
            #Will be massivly improved
            self.pub_motion(self.motion_search)
        return 'Found_b', 'Found_bouy'


def main():
    rospy.init_node("gate_passing")
    sm = smach.StateMachine(outcomes = {'Gate_Passed'})

    sis = smach_ros.IntrospectionServer('Gate_passing_server', sm, '/SM_ROOT')
    sis.start()

    with sm:
        smach.StateMachine.add('Search', Search(),
                                transitions={'Found':'Center'}
                                remapping={'Found_gate':'sm_data1'})
        smach.StateMachine.add('Center', Center(),
                                transitions={'Centered':'Drive'})
        smach-StateMachine.add('Search_b', Search_Bouy,
                                transitions = {'Found_b':'Center'}
                                remapping={'Found_bouy','sm_data2'})
        smach-StateMachine.add('Drive', Drive(),
                                transitions={'Lost':'Search',
                                            'Complete','Search_b'
                                            'Lost_bouy':'Seach_b',
                                            'Complete_bouy', 'Search'}
                                remapping={'Input_bouy':'sm_data2',
                                            '_gate':'sm_data1'})

if __name__ == '__main__':
    main()
    print("Main function started")

