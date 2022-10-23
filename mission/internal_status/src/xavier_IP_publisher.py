 #!/usr/bin/env python
#To be put into the Xavier
import time
import rospy
from std_msgs.msg import String
import subprocess
 
def xavier_IP_monitor():
    pub = rospy.Publisher('xavier_IP_monitor', String, queue_size=10)
    #while pub.get_num_connections() < 1:
     #   time.sleep(0.5)

    rospy.init_node('xavier_IP_monitor', anonymous=False)
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        cmd = "ip -4 -o address show dev eth0 | awk '{print $4}'"
        IP_bytes = subprocess.check_output(cmd, shell = True )
        IP_str = IP_bytes.decode('utf-8')
        
        rospy.loginfo(IP_str)
        pub
        pub.publish(IP_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        xavier_IP_monitor()
    except rospy.ROSInterruptException:
        pass