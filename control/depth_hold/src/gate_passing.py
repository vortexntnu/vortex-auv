import rospy
import os
import smach
import smach_ros
from vortex_msgs.msg import PropulsionCommand, CameraObjectInfo

#USE MANTA_THRUSTE_MANGER instead of 

class Search(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes={'Not found','Found'})
        
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info', CameraObjectInfo, self.callback, queue_size=1000)
        
        self.found_gate = False
        
        #Sets the searching speed in yaw
        self.motion_search = [0,0,0,0,0,0.1]

    def callback(self, cam_msg):
        if cam_msg.confidence == 0:
            self.found_gate = False
        else:
            self.found_gate = True

            #Unsubscribe
            self.sub_gate = None

    def execute(self, userdata):
        print("Search")
        while self.found_gate == False:
            print("not found")
            motion_msg = PropulsionCommand()
            motion_msg.motion = self.motion_search
            motion_msg.control_mode = [False,False,False,False,False,False]
            self.pub_motion.publish(motion_msg)
            return 'Not found'
        
        print("FOUND")
        return 'Found'

            

class Center(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes={'Centered','Lost'})
        self.pub_motion = rospy.Publisher('manta_thruster_manager',
                                          PropulsionCommand,queue_size=1)
        self.motion_forward=[1, 0, 0, 0, 0, 0]
        self.passed = False
        self.lost = False
    def execute(self, userdata):
        rospy.loginfo('Centering')
        print("Centering")
        #os.system("rosrun camera_centering camera_centering_node")
        if self.passed == False:
            motion_msg = PropulsionCommand()
            motion_msg.motion = self.motion_forward
            motion_msg.control_mode = [False,False,False,False,False,False]
            self.pub_motion.publish(motion_msg)
            print(motion_msg.motion)
        
        if self.passed:
            return 'Centered'
        
        
        return 'Lost'
def main():

    rospy.init_node("gate_passing")
    sm = smach.StateMachine(outcomes = {'Gate_Passed'})

    

    sis = smach_ros.IntrospectionServer('Gate_passing_server', sm, '/SM_ROOT')
    

    with sm:
        smach.StateMachine.add('Search', Search(),
                                transitions={'Found':'Center', 
                                             'Not found':'Search'})
        smach.StateMachine.add('Center', Center(),
                                transitions={'Centered':'Gate_Passed', 
                                             'Lost':'Center'})
    sis.start()
    sm.execute()
    sis.stop()
if __name__ == "__main__":
    main()
    rospy.spin()

