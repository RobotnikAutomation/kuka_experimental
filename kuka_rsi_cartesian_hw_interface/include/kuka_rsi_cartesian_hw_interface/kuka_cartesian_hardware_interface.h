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
 * Author: Ángel Soriano
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
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

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
#include <kuka_rsi_cartesian_hw_interface/set_A1_A6.h>

#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <robotnik_msgs/set_CartesianEuler_pose.h>
#include <robotnik_msgs/Cartesian_Euler_pose.h>

#include <cmath>

namespace kuka_rsi_cartesian_hw_interface
{

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;
static const int Z_FORCE_UPPER_LIMIT = 8;
static const float Z_FORCE_LOWER_LIMIT = 7.5;
static const int A6_LOWER_LIMIT = 0;
static const int A6_UPPER_LIMIT = 360;
static const int A1_LOWER_LIMIT = -115;
static const int A1_UPPER_LIMIT = 130;
static const int JOINT_MOVE_VELOCITY_FACTOR = 1;
static const int JOINT_MOVE_FAST_VELOCITY_FACTOR = 2; 
static const int CARTESIAN_MOVE_VELOCITY_FACTOR = 1;
static const float CARTESIAN_MOVE_FAST_VELOCITY_FACTOR = 4;
static const float ROBOT_VELOCITY = 7.5; // mm/s
static const float T_CYC = 0.12;					// milisec

static const float MIN_TOTAL_DISTANCE_THRESHOLD = 1000.0; // mm 
static const float MIN_RAMP_VALUE = 0.18;                 // Valor de rampa si la distancia total es muy corta  _/^^^\_
static const int PLATEAU_VALUE= 1.0;                      // Valor de la meseta entre aceleración y deceleración (vel. máx)
static const float MIN_RAMP_FACTOR = 0.05;                // Valor mínimo para arrancar más rápido
static const int BREAKING_DISTANCE = 150;                 // mm distancia en la que se inicia aceleración o desaceleración
static const float BREAKING_ANGLE = 5.0;                    // grados en los que se inicia la aceleración o deseceleración

static const float MIN_JOINT_STEP = 0.015;

static const int MAX_CONT_NOT_MOVING = 100;               //iteraciones máximas para parar el control si el robot no se está moviendo

static const int MIN_X_LIMIT = -650;                      //lowest X mm value for the tcp
static const int MAX_Z_LIMIT = 2500;                      //highest Z mm value for the tcp (caution with the ceiling)

static const float UP_LIMIT_A6 = 340;                  // 14+180; //cambiada configuración  muñeca
static const float LOW_LIMIT_A6 = 30;                   //-349+180; //cambiada configuración  muñeca

static const float MIN_ERROR_THRESHOLD_A1 = 0.1;                  //umbral mínimo de error para mover
static const float MAX_STEP_A1 = 0.2;                             
static const double MIN_STEP_A1 = 0.001;                      
static const int MIN_ERROR_THRESHOLD_A6 = 1;                    //umbral mínimo de error para mover
static const float MAX_STEP_A6 = 0.1;                             
static const double MIN_STEP_A6 = 0.015;   

// Conversión de grados a radianes
inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// Conversión de radianes a grados
inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

// Normaliza un ángulo en grados al rango [-180, 180]
inline double normalizeAngleDeg(double angle) {
  // Se utiliza fmod para obtener el residuo y se ajusta al rango deseado
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0)
      angle += 360.0;
  return angle - 180.0;
}

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
  
  struct CartesianPadCommand {
    double x;   // Posición en X (mm o la unidad definida)
    double y;   // Posición en Y
    double z;   // Posición en Z
    double A1;  // Movimiento para la articulación A1 (usualmente asociado a pitch)
    double A6;  // Movimiento para la articulación A6 (usualmente asociado a roll)
    double yaw; // Giro o yaw de la herramienta

    // Constructor que inicializa todos los valores a cero
    CartesianPadCommand() : x(0.0), y(0.0), z(0.0), A1(0.0), A6(0.0), yaw(0.0) {}
  };

  struct RSIMessageStruct {
    double x; // Posición en X
    double y; // Posición en Y
    double z; // Posición en Z
    double a; // Ángulo A
    double b; // Ángulo B
    double c; // Ángulo C
    double a1; // joint A1
    double a2; // joint A2
    double a3; // joint A3
    double a4; // joint A4
    double a5; // joint A5
    double a6; // joint A6

    // Constructor por defecto
    RSIMessageStruct() : x(0.0), y(0.0), z(0.0), a(0.0), b(0.0), c(0.0), a1(0.0), a2(0.0), a3(0.0), a4(0.0), a5(0.0), a6(0.0) {}

    // Constructor parametrizado
    RSIMessageStruct(double _x, double _y, double _z, double _a, double _b, double _c, double _a1, double _a2, double _a3, double _a4, double _a5, double _a6)
        : x(_x), y(_y), z(_z), a(_a), b(_b), c(_c), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6) {}

    // Método para convertir a un std::vector<double> de 12 posiciones
    std::vector<double> toVector() const {
        // Se crea un vector de 12 elementos inicializado en cero
        std::vector<double> vec(12, 0.0);
        // Se asignan las 6 variables a las primeras posiciones
        vec[0] = x;
        vec[1] = y;
        vec[2] = z;
        vec[3] = a;
        vec[4] = b;
        vec[5] = c;
        vec[6] = a1;
        vec[7] = a2;
        vec[8] = a3;
        vec[9] = a4;
        vec[10] = a5;
        vec[11] = a6;
        // Las posiciones de 6 a 11 quedan como 0.0 (o puedes asignar otros valores si es necesario)
        return vec;
    }
  };

  // RSI
  RSIState rsi_state_;
  //RSICommand rsi_command_;
  std::vector<double> rsi_initial_command_;
  std::vector<double> rsi_command_;
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
  ros::Subscriber pad_sub_;
  ros::Subscriber phidget_sub_;
  CartesianPadCommand cartesian_pad_cmds_;
  
  void padCallback(const robotnik_trajectory_pad::CartesianEuler::ConstPtr& c);
  void phidgetCallback(const std_msgs::Float64::ConstPtr& c);
  
  ros::ServiceServer set_kuka_absolute_goal_pose_;
  ros::ServiceServer set_kuka_relative_goal_pose_;
  ros::ServiceServer set_kuka_absolute_goal_pose_fast_;
  ros::ServiceServer set_kuka_relative_goal_pose_fast_;
  ros::ServiceServer set_kuka_joints_A1_and_A6_;
  ros::ServiceServer set_moveRelTool_;

  //Publishers of robot state and ROS msgs
  ros::Publisher cartesian_robot_pose_pub_;
  ros::Publisher robot_is_moving_pub_;
  robotnik_msgs::Cartesian_Euler_pose current_cartesian_robot_pose_;
  std_msgs::Bool robot_is_moving_msg_;
  
  
  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  ros::Time last_publish_time_;
  double publish_rate_;
  
  //for the service
  float cartesian_goal_pose_[6]; //desired position
  float joint_A1_goal_pose_;
  float joint_A6_goal_pose_;
  float start_cartesian_pose_request_[6];
  float start_joint_pose_request_[6];
  bool cartesian_correction_request_;
  bool joint_correction_request_;
  float cartesian_step_[3]; //x,y,z
  float joint_step_[3]; //A,B,C
  float moving_[6];
  float velocity_trajectory_kuka;
  float t_cyc;
  int counter_not_moving_;
  float total_distance_to_cover_;
  float total_time;
  float prev_distance_remaining_;
  float prev_angle_A_error;
  float initial_angle_A_error_;
  float prev_angle_B_error;
  float first_angle_B_error;
  float prev_angle_C_error;
  float first_angle_C_error;
  float prev_A6_error;
  float start_A1_error_request_;
  float start_A6_error_request_;
  float prev_A1_error;
  float slope;
  float distance_traveled_;
  float distance_remaining_;
  float angle_A_error;
  float angle_A_moved_from_start;
  float angle_B_error;
  float angle_B_moved_from_start;
  float angle_C_error;
  float angle_C_moved_from_start;
  float A1_moved_from_start;
  float A6_moved_from_start;
  float pos_init_A6;
  float req_A6;
  float A1_current_error_;
  float A6_current_error_;
  bool A6_in_valid_range;
  float accumulated_A1_rotation_rad; //temporal correction
  float limit_low_x;
  bool move_relative_to_tool_; //to move relatively to tool coordinates
  bool z_force_limit_reached_;
  float step_max_A1;


 
  //publisher
  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;


public:

  KukaHardwareInterface();
  ~KukaHardwareInterface();

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);
 
  bool setAbsoluteCartGoalPose(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setRelativeCartGoalPose(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setAbsoluteCartGoalPoseFast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool setRelativeCartGoalPoseFast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response);
  bool moveJointsA1andA6(kuka_rsi_cartesian_hw_interface::set_A1_A6::Request &request, kuka_rsi_cartesian_hw_interface::set_A1_A6::Response &response);
  bool setMoveRelTool(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  
  bool settingRelativeCartGoalPose(
		robotnik_msgs::set_CartesianEuler_pose::Request &req,
		robotnik_msgs::set_CartesianEuler_pose::Response &res,
		float velocity_factor_param);

  bool settingAbsoluteCartGoalPose(
    robotnik_msgs::set_CartesianEuler_pose::Request &req,
    robotnik_msgs::set_CartesianEuler_pose::Response &res,
    float velocity_factor_param);

};

} // namespace kuka_rsi_hw_interface

#endif
