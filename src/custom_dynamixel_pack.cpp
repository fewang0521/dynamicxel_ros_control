#include "custom_dynamixel_pack/custom_dynamixel_pack.h"
#include <stdio.h>
//namespace hardware_interface
//{
MyRobot::MyRobot(ros::NodeHandle& nh){
   nh_=nh;
   init();
   controller_manager_ = new controller_manager::ControllerManager(this, nh_);
   nh_.param("/ROBOT/hardware_interface/loop_hz", loop_hz_, 0.1);
   ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
   non_realtime_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot(){
  robot1.com_end();
}

void MyRobot::init()
{
  nh_.getParam("/ROBOT/hardware_interface/joints", joint_names_);
  num_joints_=joint_names_.size();
  hardware_interface::JointStateHandle state_handle_a(joint_names_[0], &pos[0], &vel[0], &eff[0]);
  joint_state_interface_.registerHandle(state_handle_a);
  hardware_interface::JointHandle pos_handle_a(state_handle_a, &cmd[0]);
  position_joint_interface_.registerHandle(pos_handle_a);
//hardware_interface::JointHandle joint_effort_handle(state_handle_a, &eff[0]);
//effort_joint_interface_.registerHandle(joint_effort_handle);
    
 
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
//registerInterface(&effort_joint_interface_);

  char cstr[] ="/dev/rfcomm0";
  robot1.com_start(cstr);
}
void MyRobot::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  sleep(1);
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}
void MyRobot::read(){

   pos[0]=(double)robot1.robot_read();
   //printf("%d is pos value\n", pos[0]);
}
void MyRobot::write(ros::Duration& elapsed_time) {
   robot1.robot_write((int)cmd[0]);
}
   
//}
