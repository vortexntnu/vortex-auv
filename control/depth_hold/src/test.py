#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
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
        smach.State.__init__(self, outcomes={'Not_found','Found'})
        
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = None
        self.found = False
        
        self.motion_search = [0,0,0,0,0,0.01]

    def execute(self, data):
        print("search")
        while self.found == False:
            self.sub_gate = rospy.Subscriber('camera_object_info', CameraObjectInfo, queue_size=1000)
            return 'Found'
        else:
            return 'Found'
    
    def callback(self, msg):
        #We don't have our target in sight - search for it
        motion_msg = PropulsionCommand()
        motion_msg.header.stamp = rospy.get_rostime()
        motion_msg.motion = self.motion_search
        motion_msg.control_mode = [False, False, False, False, False, False]
        
        if (msg.confidence == 0):
            self.found = False
            
            self.pub_motion.publish(motion_msg)
            
            
        
        #We have found the gate
        else:
            #Unsubscribe
            self.sub_gate = None
            self.found = True
            
            

class Center(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes={'Centered','Lost'})
        self.sub_gate = None
        self.gate_lost = [-1,-1]
        self.gate_centered = [416]

        
    def execute(self):
        rospy.loginfo('Centering')
        os.system("rosrun camera_centering camera_centering_node")
        
        
        return 'Centered'

def main():

    rospy.init_node("gate_passing")
    sm = smach.StateMachine(outcomes = {'Gate_Passed'})

    

    sis = smach_ros.IntrospectionServer('Gate_passing_server', sm, '/SM_ROOT')
    

    with sm:
        smach.StateMachine.add('Search', Search(),
                                transitions={'Found':'Center', 
                                             'Not_found':'Search'})
        smach.StateMachine.add('Center', Center(),
                                transitions={'Centered':'Gate_Passed', 
                                             'Lost':'Search'})
    sis.start()
    sm.execute()
    sis.stop()
if __name__ == "__main__":
    main()
    #rospy.spin()

