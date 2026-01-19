/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad
 */

#ifndef KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_
#define KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// ros_control
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//ros_jointstates
#include <sensor_msgs/JointState.h>

// Timers
#include <chrono>

// UDP server
#include <kuka_rsi_cartesian_hw_interface/udp_server.h>

// RSI
#include <kuka_rsi_cartesian_hw_interface/rsi_state.h>
#include <kuka_rsi_cartesian_hw_interface/rsi_command.h>

#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <robotnik_msgs/set_CartesianEuler_pose.h>
#include <robotnik_msgs/Cartesian_Euler_pose.h>

namespace kuka_rsi_cartesian_hw_interface
{

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class KukaHardwareInterface : public hardware_interface::RobotHW
{

private:

  // ROS node handle
  ros::NodeHandle nh_;

  unsigned int n_dof_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  std::vector<double> rsi_abs_cart_correction_;
  unsigned long long ipoc_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_pub_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;
  
  //Cartesian movement from topic
  ros::Subscriber pad_subs;
  std::vector<double> cartesian_pad_cmds_;
  
  void padcallback(const robotnik_trajectory_pad::CartesianEuler::ConstPtr& c);
  
  ros::ServiceServer set_kuka_odometry_abs;
  ros::ServiceServer set_kuka_odometry_rel;
  ros::ServiceServer set_kuka_odometry_abs_fast;
  ros::ServiceServer set_kuka_odometry_rel_fast;

  //Publishers of robot state
  ros::Publisher cart_pos_pub;
  ros::Publisher kuka_moving_pub;
  robotnik_msgs::Cartesian_Euler_pose cart_pos;
  std_msgs::Bool msgs_kuka_moving;
  
  
  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  ros::Time last_publish_time_;
  double publish_rate_;
  
  //for the service
  float aut_cmds_[6]; //desired position
  float pose_init_[6];
  bool service_set_kuka_abs;
  bool service_set_kuka_rel;
  float step_tr[6];
  float moving_[6];
  float velocity_trajectory_kuka;
  float t_cyc;
  int counter_not_moving;
  float total_time;
  float velocity_factor;
  float prev_distance_to_end;
  float prev_angle_A_error;
  float first_angle_A_error;
  float prev_angle_B_error;
  float prev_angle_C_error;
  float breaking_distance;
  float breaking_angle;
  float slope;
  float distance_from_start;
  float distance_to_end;
  float angle_A_error;
  float angle_A_moved_from_start;
  float angle_B_error;
  float angle_B_moved_from_start;
  float angle_C_error;
  float angle_C_moved_from_start;
  float rot_A;
  float pos_init_A6;
  bool first_time;
  float upper_limit_A6, lower_limit_A6;
  bool range_A6;
 
  //publisher
  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;


public:

  KukaHardwareInterface();
  ~KukaHardwareInterface();

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);
 
  bool setKukaOdometry_abs(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setKukaOdometry_rel(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setKukaOdometry_abs_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setKukaOdometry_rel_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);


};

} // namespace kuka_rsi_hw_interface

#endif
