#include "mcu_interface.h"

MCU_Interface::MCU_Interface ():loop_rate(10){
    pwm_sub = nh.subscribe("/pwm",10,&MCU_Interface::pwm_cb,this);
}

void MCU_Interface::pwm_cb(vortex_msgs::Pwm pwm_msg){

}

void MCU_Interface::execute(){
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"mcu_interface_new");
    MCU_Interface mcu_i;
    mcu_i.execute();
}