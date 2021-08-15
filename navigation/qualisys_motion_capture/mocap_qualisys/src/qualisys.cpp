/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mocap_qualisys/QualisysDriver.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "qualisys");
  ros::NodeHandle nh("~");

  mocap::QualisysDriver driver(nh);
  if(!driver.init()) {
    ROS_INFO("Initialization of the Qualisys driver failed!");
    return -1;
  }
  ROS_INFO("Successfully initialized QTM interface node!");
  
  // ros::Rate r(200.0);
 bool status = true;
  while(ros::ok() && status == true)
  { 
    //ROS_INFO("Runing");
    status = driver.run();
    //ROS_INFO("Spining");
    ros::spinOnce();
    //ROS_INFO("Sleeping");
    //ROS_INFO("Cycle time: %f", r.cycleTime().toSec());
    // r.sleep();
  }
  ROS_INFO("QTM interface node shutting down");
  //driver.disconnect();

  return 0;
}
