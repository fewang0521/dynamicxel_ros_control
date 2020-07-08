#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "robot/robot.h"

#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle& nh);
  ~MyRobot();
  void init();
  void update(const ros::TimerEvent& e);
  virtual void read();
  virtual void write(ros::Duration& elapsed_time);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
//hardware_interface::EffortJointInterface effort_joint_interface_;
  robot robot1;

  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  controller_manager::ControllerManager* controller_manager_;
  double p_error;

  int num_joints_;
  int joint_mode_;
  std::vector<std::string> joint_names_;
  int joint_types_;
  double cmd[1];
  double pos[1];
  double vel[1];
  double eff[1];
};
