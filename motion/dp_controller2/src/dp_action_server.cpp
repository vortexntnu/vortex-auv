#include "dp_controller2/dp_action_server.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <math.h>






 Eigen::Vector3d SmallestAngle(Eigen::Vector3d euler_angles){
    Eigen::Vector3d smallest_euler_angles = Eigen::Vector3d::Zero();
    for (int i = 0; i < euler_angles.size(); i++){
        if (euler_angles(i) > M_PI){
            smallest_euler_angles(i) = euler_angles(i) - 2*M_PI;
        }
        else if (euler_angles(i) < -M_PI){
            smallest_euler_angles(i) = euler_angles(i) + 2*M_PI;
        }
        else {
            smallest_euler_angles(i) = euler_angles(i);
        }
    }
    
    return smallest_euler_angles;
}

//Quaternion to Euler
Eigen::Vector3d QuaterniondToEuler(Eigen::Quaterniond q){
Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
return euler;
}


DpAction::DpAction(std::string name) : as_(nh_, name, boost::bind(&DpAction::executeCB, this, _1), false),
  action_name_(name)
{
  as_.start();
  std::cout << "HEISANN2" << std::endl;
}

void DpAction::executeCB(const dp_controller2::dpGoalConstPtr &goal)
{
  goal_.x_ref= goal->x_ref;
  goal_.DOF = goal->DOF;
  std::cout << std::endl << "-------------" << std::endl;
  std::cout << std::endl << goal_.x_ref << std::endl;
  std::cout << std::endl << goal->x_ref<< std::endl;
  std::cout << std::endl << "-------------" << std::endl;
  ros::Rate r(1);
  bool success = false;

  float test_radius = 1;
  float test_deg = 20;

  Eigen::Vector6d error = Eigen::Vector6d::Zero();

  while(!as_.isPreemptRequested() && ros::ok() && !as_.isNewGoalAvailable()){
    run_controller = true;
    feedback_.error.clear();

    Eigen::Vector3d x_ref_pos;
    Eigen::Quaterniond x_ref_ori;
    tf::pointMsgToEigen(goal_.x_ref.position, x_ref_pos);
    tf::quaternionMsgToEigen(goal_.x_ref.orientation, x_ref_ori);

    Eigen::Vector3d error_pos = x_ref_pos - pose.segment(0,3);
    std::cout << std::endl << "et eller annet piss" << std::endl;
    std::cout << std::endl << x_ref_pos << std::endl;
    std::cout << std::endl << pose.segment(0,3) << std::endl;

    Eigen::Vector3d error_ori = QuaterniondToEuler(x_ref_ori) - pose.segment(3,3);

    error << error_pos, error_ori;



    for (int i = 0; i < 6; i++){
      feedback_.error.push_back(error[i]);
    }

    std::cout << std::endl << "check for new goal" << std::endl;
    if (as_.isNewGoalAvailable()){
      std::cout << std::endl << "new goal" << std::endl;
    }

     // publish the feedback
    as_.publishFeedback(feedback_);
    Eigen::VectorXd DOF = Eigen::VectorXd::Zero(6,1);
    for (int i = 0; i < 6; i++){
        // std::cout << "DOF:" << i << std::endl;
        DOF(i) = goal_.DOF[i];
      }
    std::cout << "DOF:" << DOF << std::endl;
    error = DOF.cwiseProduct(error);
    double distance_from_goal = error.segment(0,3).norm();
    std::cout << std::endl << "avstand fra målet:" << std::endl << distance_from_goal << std::endl;
    std::cout << std::endl << "feil:" << std::endl << error << std::endl;
    Eigen::Vector3d error_ori_deg = error.segment(3,3)*180/M_PI;
    if(distance_from_goal < test_radius && abs(error_ori_deg[0]) < test_deg && abs(error_ori_deg[1]) < test_deg && abs(error_ori_deg[2]) < test_deg && success == false){
      success = true;
      result_.finished = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }

    r.sleep();

  }

  //if(!success) as_.setPreempted();
  as_.setPreempted();
  // run_controller = false;

  // feedback_.sequence.clear();
  // feedback_.sequence.push_back(0);
  // feedback_.sequence.push_back(1);

  // // helper variablesorientation
  // // publish info to the console for the user
  // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

  // // start executing the action
  // for(int i=1; i<=goal->order; i++)
  // {
  //   // check that preempt has not been requested by the client
  //   if (as_.isPreemptRequested() || !ros::ok())
  //   {
  //     ROS_INFO("%s: Preempted", action_name_.c_str());
  //     // set the action state to preempted
  //     as_.setPreempted();
  //     success = false;
  //     break;
  //   }
  //   feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
  //   // publish the feedback
  //   as_.publishFeedback(feedback_);
  //   // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes

  //   feedback_.test.push_back(9);
  //   feedback_.test.push_back(0);
  //   feedback_.test.push_back(0);
  //   feedback_.test.push_back(9);
  //   for (int i: feedback_.test){
  //     std::cout << i << std::endl;
  //   }

  //   geometry_msgs::Pose pose_test_hei;
  //   pose_test_hei.orientation.w = 1;
  //   pose_test_hei.orientation.x = 0;
  //   pose_test_hei.orientation.y = 0;
  //   pose_test_hei.orientation.z = 0;
  //   pose_test_hei.position.x = 0;
  //   pose_test_hei.position.y = 0;
  //   pose_test_hei.position.z = 0;

  //   feedback_.pose_test = pose_test_hei;
  //   std::cout << feedback_.pose_test << std::endl;



  //   r.sleep();
  // }

  // if(success)
  // {
  //   result_.sequence = feedback_.sequence;
  //   ROS_INFO("%s: Succeeded", action_name_.c_str());
  //   // set the action state to succeeded
  //   as_.setSucceeded(result_);
  // }


  
}