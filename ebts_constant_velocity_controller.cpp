// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Vertical track
#include <ebts_movement/ebts_constant_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unistd.h>
#include <chrono>


#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"


namespace ebts_movement {

bool EbtsConstantVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};//{{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianVelocityExampleController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void EbtsConstantVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  direction_down=true;
  direction_right=true;
  direction_left=true;
  direction_up=true;
  direction_forward = true;
  direction_forward_2 = true;
  direction_back=true;
  v_x=0;
  v_y=0;
  v_z=0;
  a_x=0.000025;
  a_y=0.00001;
  a_z=0.00001;
  d_x=0.15;
  d_y=0.1;
  d_z=0.01;
  v_x_to_init_pose=sqrt(1000*a_y*0.600);
  v_y_to_init_pose=sqrt(1000*a_y*0.112);
  v_z_to_init_pose=sqrt(1000*a_z*0.233); 
  v_x_final=sqrt(1000*a_x*d_x); //wisewordsfromNurlan: vfinal was calculated using vf^2=vi^2+2ax, vi=0, x=real_x/2, multiply by 1000 because a_x was in milliseconds 
  v_y_final=sqrt(1000*a_y*d_y);
  v_z_final=sqrt(1000*a_z*d_z);
  phase=102;
  count = 0;
  total_count=2; //# of textures
  repetition=0; // exploring the same texture track nth time
  total_repetitions=10; // exploring the same texture track n times
  changing_lanes=true; // changing to the next column when there are more than one column of textures 
  go_initial=true; // means the robot is at move_to_start position 
  tactile_exploration=false;  //indicator of exploring a texture surface excluding the transitions between textures for data collection
  waiting_counter=0;
  const_speed_loop_counter=0;  // # of seconds for const speed movement
  
}

void EbtsConstantVelocityController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  

  // double time_max = 4.0;
  // double v_max = 0.05;
  // double angle = M_PI / 4.0;
  // double cycle = std::floor(  pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max)  );
  // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  // double v_x = std::cos(angle) * v;
  // double v_z = -std::sin(angle) * v;


  elapsed_time_ += period;

  //go to initial position



  switch(phase){
    case 101:  // forward to the initial position
      if(direction_forward){
        v_y +=a_y;
        std::cout<<"Accelerating forward"<<std::endl;
        if(v_y>=v_y_to_init_pose){
          direction_forward=false;
          }
      }
      else{
        v_y-=a_y;
        std::cout<<"Decelerating forward"<<std::endl;
        if(v_y<=0){
          v_y=0;
          direction_forward=true;
          phase=102;
        }
      }
      break;

    case 102: // left to the initial position
      if(direction_left){
        //std::cout<<"tactile_exploration=true"<<std::endl;
        v_x+=a_x;
        std::cout<<"Accelerating left"<<std::endl;
        if(v_x>=v_x_to_init_pose){
          direction_left=false;
        }
      }
      else{
        v_x-=a_x;
        //std::cout<<"tactile_exploration=false"<<std::endl;
        std::cout<<"Decelerating left"<<std::endl;
        if(v_x<=0){
          v_x=0;
          direction_left=true;
          phase=1;
        }
      }
      break;
    
    case 1: 
      if(direction_down){
        v_z-=a_z;
        std::cout<<"Accelerate down"<<std::endl;
        if(go_initial){
          if(v_z<=-v_z_to_init_pose){     //getting to the initial position
            direction_down=false;
            
          }
        } 
        else{
          if(v_z<=-v_z_final){           //going down in between the texture exploration process
            direction_down=false;
          }
        }
      }
      else{
        v_z+=a_z;
        std::cout<<"Decelerate down"<<std::endl;
        if(v_z>=0){
          v_z=0;
          direction_down=true;
          go_initial=false;                  
          phase=2;                 
        }
      }
      break;
    case 2: //going right
      if(direction_right){
        //tactile_exploration=true;
        //std::cout<<"tactile_exploration=true"<<std::endl;
        v_x-=a_x;
        std::cout<<"Accelerating right"<<std::endl;
        if(v_x<=-v_x_final){
          direction_right=false;
          phase=8;
        }
      }
      else{
        v_x+=a_x;
        //std::cout<<"tactile_exploration=false"<<std::endl;
        std::cout<<"Decelerating right"<<std::endl;
        //tactile_exploration=false;
        if(v_x>=0){
          direction_right=true;
          waiting_counter=0;
          phase=103;
        }
      }
      break;
    case 103:  // stopping for 0.5 seconds before reversing the direction from right to left                              
      waiting_counter++;
      if(waiting_counter==500){
        std::cout<<waiting_counter<<std::endl;
        phase=3;
      }
      break;
    case 3: //going left
      if(direction_left){
        //tactile_exploration=true;
        //std::cout<<"tactile_exploration=true"<<std::endl;
        v_x+=a_x;
        std::cout<<"Accelerating left"<<std::endl;
        if(v_x>=v_x_final){
          direction_left=false;
          phase=9;
        }
      }
      else{
        v_x-=a_x;
        //std::cout<<"tactile_exploration=false"<<std::endl;
        std::cout<<"Decelerating left"<<std::endl;
        if(v_x<=0){
          v_x=0;
          //tactile_exploration=false;
          direction_left=true;
          repetition++;
          if(repetition==total_repetitions){
            repetition=0;
            phase=4;
          }
          else{
            
            phase=2;
          }
        }
      }
      break;
    case 4: // going up
      if(direction_up){
        v_z+=a_z;
        std::cout<<"Accelerating up"<<std::endl;
        if(v_z>=v_z_final){
          direction_up=false;
        }
      }
      else{
        v_z-=a_z;
          std::cout<<"Decelerating up"<<std::endl;
        if(v_z<=0){
          v_z=0;
          direction_up=true;
          phase = 5;
          if(count == total_count-1){
            // if(changing_lanes){
            //   phase = 6;
            // }
            // else {
              phase=6;
            //}
          }else{
            phase = 5;
          }
        }
      }
      break;
    case 5: //going forward 
      if(direction_forward){
        v_y +=a_y;
        std::cout<<"Accelerating forward"<<std::endl;
        if(v_y>=v_y_final){
          direction_forward=false;
        }
      }
      else{
        v_y-=a_y;
          std::cout<<"Decelerating forward"<<std::endl;
        if(v_y<=0){
          v_y=0;
          direction_forward=true;
          if(count < total_count){
            phase=1;
            count++;
          }
          else{
            phase = 8;
          }
          
        }
      }
      break;
      
    case 8: //constant end effector movement to the right (phase 2)                             
      v_x=v_x;
      const_speed_loop_counter++;
      tactile_exploration=true;
      if(const_speed_loop_counter==4000){   //repeat for 3 seconds
        phase=2;
        const_speed_loop_counter=0;
        tactile_exploration=false;
      }
      break;

    case 9: //constant end effector movement to the right (phase 3)     
      v_x=v_x;
      const_speed_loop_counter++;
      tactile_exploration=true;
      if(const_speed_loop_counter==4000){   //repeat for 3 seconds
        phase=3;
        const_speed_loop_counter=0;
        tactile_exploration=false;
      }
      break;

    //  case 6: //changing lanes: going backwards 
    //    if(direction_back){
    //      v_y-=a_y;
    //      std::cout<<"changing lanes: Accelerating backwards"<<std::endl;
    //      if(v_y<=-2*v_y_final){
    //        direction_back=false;
    //      }
    //    }
    //    else{
    //      v_y+=a_y;
    //      std::cout<<"changing lanes: Decelerating backwards"<<std::endl;
    //      if(v_y>=0){
    //        direction_back=true;
    //        changing_lanes=false;
    //        phase=1;
    //        count=0;
    //      }
    //    }
    //    break;

    // case 7: //changing lanes: going right
    //   if(direction_right){
    //     v_x-=a_x;
    //     std::cout<<"changing lanes: Accelerating right"<<std::endl;
    //     if(v_x<=-0.866*v_x_final){
    //       direction_right=false;
    //     }
    //   }
    //   else{
    //     v_x+=a_x;
    //     std::cout<<"Decelerating right"<<std::endl;
    //     if(v_x>=0){
    //       phase=1;
    //       direction_right=true;
    //     }
    //   }
    //   break;


    

  }


  msg.data =tactile_exploration;
  chatter_pub.publish(msg);
  std::cout<< ros::Time::now()<<": "<<tactile_exploration<<std::endl;

  std::cout<<"Vz=" <<v_z <<" || Vy=" <<v_y<<" || Vx=" <<v_x<<std::endl;
  std::array<double, 6> command = {{v_y, v_x, v_z, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

enum phases {
    RC_RED, RC_ORANGE, RC_YELLOW, RC_GREEN, RC_BLUE, RC_INDIGO, RC_VIOLET
};

void EbtsConstantVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(ebts_movement::EbtsConstantVelocityController,
                       controller_interface::ControllerBase)
