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
 * Author: Ángel Soriano <asoriano@robotnik.es>
 */

#include <kuka_rsi_cartesian_hw_interface/kuka_cartesian_hardware_interface.h>
#include "joint_state_controller/joint_state_controller.h"
#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <stdexcept>
#include <robotnik_msgs/set_odometry.h>
#include <cmath>
#include <sys/time.h>

struct timespec tvalBefore1, tvalAfter1, tvalMid, tvalMid2;

namespace kuka_rsi_cartesian_hw_interface
{

	KukaHardwareInterface::KukaHardwareInterface() : joint_position_(6, 0.0), 
	joint_velocity_(6, 0.0), joint_effort_(6, 0.0), joint_position_command_(12, 0.0), 
	joint_velocity_command_(6, 0.0), joint_effort_command_(6, 0.0), joint_names_(6), 
	rsi_initial_command_(12, 0.0), ipoc_(0), n_dof_(6), total_distance_to_cover_(0)
	{
		in_buffer_.resize(1024);
		out_buffer_.resize(1024);
		remote_host_.resize(1024);
		remote_port_.resize(1024);

		if (!nh_.getParam("controller_joint_names", joint_names_))
		{
			ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
					  "on the parameter server.");
			throw std::runtime_error("Cannot find required parameter "
									 "'controller_joint_names' on the parameter server.");
		}
		// get publishing period
		if (!nh_.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set");
			throw std::runtime_error("Cannot find required parameter ");
		}
		// realtime publisher
		realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh_, "joint_states", 4));
		// publishes the cartesian position of the robot
		cartesian_robot_pose_pub_ = nh_.advertise<robotnik_msgs::Cartesian_Euler_pose>("cartesian_pos_kuka", 10);
		// publishes if the robot is moving through service
		robot_is_moving_pub_ = nh_.advertise<std_msgs::Bool>("kuka_moving", 10);
		// subscriber
		pad_sub_ = nh_.subscribe<robotnik_trajectory_pad::CartesianEuler>("/kuka_pad/cartesian_move", 1, &KukaHardwareInterface::padCallback, this);
		phidget_sub_ = nh_.subscribe<std_msgs::Float64>("/phidget_load/load_mean", 1, &KukaHardwareInterface::phidgetCallback, this);
		// service
		set_kuka_absolute_goal_pose_ = nh_.advertiseService("setKukaAbs", &KukaHardwareInterface::setAbsoluteCartGoalPose, this);
		set_kuka_relative_goal_pose_ = nh_.advertiseService("setKukaRel", &KukaHardwareInterface::setRelativeCartGoalPose, this);
		set_kuka_absolute_goal_pose_fast_ = nh_.advertiseService("setKukaAbsFast", &KukaHardwareInterface::setAbsoluteCartGoalPoseFast, this);
		set_kuka_relative_goal_pose_fast_ = nh_.advertiseService("setKukaRelFast", &KukaHardwareInterface::setRelativeCartGoalPoseFast, this);
		set_kuka_joints_A1_and_A6_ = nh_.advertiseService("setKukaA1A6", &KukaHardwareInterface::moveJointsA1andA6, this);
		set_moveRelTool_ = nh_.advertiseService("setMoveRelTool", &KukaHardwareInterface::setMoveRelTool, this);

		initial_angle_A_error_ = 0;
	}

	KukaHardwareInterface::~KukaHardwareInterface()
	{
	}

	//Inicialization
	void KukaHardwareInterface::start()
	{
		// for the service
		cartesian_correction_request_ = false;
		cartesian_correction_request_ = false;
		robot_is_moving_msg_.data = false;
		joint_correction_request_ = false;
		counter_not_moving_ = 0;
		A6_in_valid_range = true;
		move_relative_to_tool_ = false;
		z_force_limit_reached_ = false;
		accumulated_A1_rotation_rad = 0.0;
		// Wait for connection from robot
		server_.reset(new UDPServer(local_host_, local_port_));

		ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!!");

		int bytes = server_->recv(in_buffer_);

		// Drop empty <rob> frame with RSI <= 2.3
		if (bytes < 100)
		{
			bytes = server_->recv(in_buffer_);
		}

		rsi_state_ = RSIState(in_buffer_);
		for (std::size_t i = 0; i < n_dof_; ++i)
		{
			joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
			joint_position_command_[i] = joint_position_[i];
			rsi_initial_command_[i] = rsi_state_.initial_cart_position[i];
		}
		ipoc_ = rsi_state_.ipoc;
		// out_buffer_ = RSICommand('R',rsi_initial_command_, ipoc_).xml_doc;
		out_buffer_ = RSICommand(rsi_initial_command_, ipoc_).xml_doc;
		ROS_INFO("SENT to robot:%s", out_buffer_.c_str());

		server_->send(out_buffer_);
		// Set receive timeout to 1 second
		server_->set_timeout(1000);
		ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");
		// initialize time
		last_publish_time_ = ros::Time::now();
		// get joints and allocate message
		for (unsigned i = 0; i < n_dof_; i++)
		{
			// joint_state_.push_back(hw->getHandle(joint_names[i]));
			realtime_pub_->msg_.name.push_back(joint_names_[i]);
			realtime_pub_->msg_.position.push_back(0.0);
			realtime_pub_->msg_.velocity.push_back(0.0);
			realtime_pub_->msg_.effort.push_back(0.0);
		}
	}

	void KukaHardwareInterface::configure()
	{
		if (nh_.getParam("rsi/listen_address", local_host_) && nh_.getParam("rsi/listen_port", local_port_))
		{
			ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
								  "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
		}
		else
		{
			ROS_ERROR("Failed to get RSI listen address or listen port from parameter server!");
			throw std::runtime_error("Failed to get RSI listen address or listen port from parameter server.");
		}
		rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
	}

	// callback from topic kuka_pad/cartesian_move
	void KukaHardwareInterface::padCallback(const robotnik_trajectory_pad::CartesianEuler::ConstPtr &cartesian_move)
	{

		cartesian_pad_cmds_.x = cartesian_move->x;
		cartesian_pad_cmds_.y = cartesian_move->y;
		cartesian_pad_cmds_.z = cartesian_move->z;
		cartesian_pad_cmds_.A1 = cartesian_move->pitch; // Used for joint A1 movement
		cartesian_pad_cmds_.A6 = cartesian_move->roll;	// Used for joint A6 movement
		cartesian_pad_cmds_.yaw = cartesian_move->yaw;

		// TRANSFORMATION FOR TOOL ORIENTATION MOVEMENT
		if (move_relative_to_tool_)
		{
			float rot_A_rad = deg2rad(current_cartesian_robot_pose_.A + 90);
			cartesian_pad_cmds_.x = -(cartesian_move->x * cos(rot_A_rad) - cartesian_move->y * sin(rot_A_rad));
			cartesian_pad_cmds_.y = -(cartesian_move->y * cos(rot_A_rad) + cartesian_move->x * sin(rot_A_rad));
		}
	}

	bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
	{
		clock_gettime(CLOCK_REALTIME, &tvalMid2);
		in_buffer_.resize(1024);

		if (server_->recv(in_buffer_) == 0)
		{
			return false;
		}
		// ROS_INFO("Received from robot:%s", in_buffer_.c_str());
		if (rt_rsi_pub_->trylock())
		{
			rt_rsi_pub_->msg_.data = in_buffer_;
			rt_rsi_pub_->unlockAndPublish();
		}

		rsi_state_ = RSIState(in_buffer_);
		// limit rate of publishing
		if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				// we're actually publishing, so increment time
				last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
				realtime_pub_->msg_.header.stamp = time;
				// ROS_INFO("READ time: %f", (-last_publish_time_.toSec()+time.toSec()));
				// update and publish by /joint_states
				for (std::size_t i = 0; i < n_dof_; ++i)
				{
					realtime_pub_->msg_.position[i] = DEG2RAD * rsi_state_.positions[i];

					realtime_pub_->msg_.velocity[i] = 0;

					realtime_pub_->msg_.effort[i] = 0;
				}

				// Update the absolute cartesian pose of the robot
				current_cartesian_robot_pose_.x = rsi_state_.cart_position[0];
				current_cartesian_robot_pose_.y = rsi_state_.cart_position[1];
				current_cartesian_robot_pose_.z = rsi_state_.cart_position[2];
				current_cartesian_robot_pose_.A = rsi_state_.cart_position[3];
				current_cartesian_robot_pose_.B = rsi_state_.cart_position[4];
				current_cartesian_robot_pose_.C = rsi_state_.cart_position[5];

				realtime_pub_->unlockAndPublish();
				cartesian_robot_pose_pub_.publish(current_cartesian_robot_pose_);
			}
		}
		ipoc_ = rsi_state_.ipoc;

		return true;
	}

	bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
	{
		out_buffer_.resize(1024);

		RSIMessageStruct RSI_message;

		// Write part of the cartesian movement services, angle B and C is commented
		if (cartesian_correction_request_ && !joint_correction_request_)
		{
			// In service
			robot_is_moving_msg_.data = true;
			
			slope = 1;

			distance_traveled_ = sqrt(
				pow(((rsi_state_.cart_position[0] * cos(accumulated_A1_rotation_rad) - rsi_state_.cart_position[1] * sin(accumulated_A1_rotation_rad)) - start_cartesian_pose_request_[0]), 2) +
				pow(((rsi_state_.cart_position[1] * cos(accumulated_A1_rotation_rad) + rsi_state_.cart_position[0] * sin(accumulated_A1_rotation_rad)) - start_cartesian_pose_request_[1]), 2) +
				pow((rsi_state_.cart_position[2] - start_cartesian_pose_request_[2]), 2));

			distance_remaining_ = sqrt(
				pow((-(rsi_state_.cart_position[0] * cos(accumulated_A1_rotation_rad) - rsi_state_.cart_position[1] * sin(accumulated_A1_rotation_rad)) + cartesian_goal_pose_[0]), 2) +
				pow((-(rsi_state_.cart_position[1] * cos(accumulated_A1_rotation_rad) + rsi_state_.cart_position[0] * sin(accumulated_A1_rotation_rad)) + cartesian_goal_pose_[1]), 2) +
				pow((-rsi_state_.cart_position[2] + cartesian_goal_pose_[2]), 2));
			// ROS_INFO(" In Service distance from start:%f distance to end: %f",distance_traveled_,distance_remaining_);

			angle_A_error = -rsi_state_.cart_position[3] + cartesian_goal_pose_[3];

			angle_A_moved_from_start = sqrt(
				pow((rsi_state_.cart_position[3] - start_cartesian_pose_request_[3]), 2));
			// Errors in B and C commented
			angle_B_moved_from_start = sqrt(
				pow((rsi_state_.cart_position[4] - start_cartesian_pose_request_[4]), 2));
			angle_B_error = 0; // first_angle_B_error-copysign(angle_B_moved_from_start,first_angle_B_error);
			// angle_C_error=-rsi_state_.cart_position[5] + cartesian_goal_pose_[5];

			angle_C_moved_from_start = sqrt(
				pow((rsi_state_.cart_position[5] - start_cartesian_pose_request_[5]), 2));
			angle_C_error = 0; // first_angle_C_error-copysign(angle_C_moved_from_start,first_angle_C_error);

			// A,B,C moves between [-179,179]
			angle_A_error = normalizeAngleDeg(angle_A_error);
			
			// Fase de "inicio corto": si la distancia total es muy pequeña
			if (total_distance_to_cover_ < MIN_TOTAL_DISTANCE_THRESHOLD)
			{
				slope = MIN_RAMP_VALUE;
			}
			else if (distance_traveled_ <= BREAKING_DISTANCE)
			{ // Fase de aceleración: se incrementa proporcionalmente al avance
				slope = distance_traveled_ / BREAKING_DISTANCE;
			}
			else if (distance_remaining_ < BREAKING_DISTANCE)
			{ // Fase de desaceleración: se reduce proporcionalmente a la distancia restante
				slope = distance_remaining_ / BREAKING_DISTANCE;
			}
			else
			{ // Fase de meseta: movimiento a velocidad máxima
				slope = PLATEAU_VALUE;
			}
			// Garantizar un valor mínimo para arrancar rápidamente
			if (slope < MIN_RAMP_FACTOR)
			{ // to make the start faster 0.05
				slope = MIN_RAMP_FACTOR;
			}
			// Si ya se recorrió o se excedió la distancia objetivo, detener el movimiento
			if (distance_traveled_ >= total_distance_to_cover_)
			{ // if it arrived to the desired position or it went further
				slope = 0;
			}
			// Aplicar la rampa a los pasos de traslación (x,y,z)			
			RSI_message.x = cartesian_step_[0] * slope;
			RSI_message.y = cartesian_step_[1] * slope;
			RSI_message.z = cartesian_step_[2] * slope;

			ROS_INFO("Steps: %f %f", RSI_message.x, RSI_message.y);
			
			// --- Comprobación y ajuste del ángulo del eje A (rotación) ---
			
			// Si A6 está fuera de rango, se ajusta el error de A para conservar el signo del error inicial
			if (!A6_in_valid_range)
			{
				angle_A_error = copysign(angle_A_error, initial_angle_A_error_);
				ROS_INFO("trajectory with A6 out of range angle error: %f first angle error: %f", angle_A_error, initial_angle_A_error_);
			}

			// Calcular el paso incremental para la rotación del eje A
			// Rotation of A angle
			if (fabs(angle_A_error) > 1.0)
			{

				if (angle_A_moved_from_start < BREAKING_ANGLE)
				{
					// Fase de aceleración: el paso es proporcional al ángulo ya movido
					joint_step_[0] = copysign(0.1 * (angle_A_moved_from_start / BREAKING_ANGLE), angle_A_error);
				}
				else if (fabs(angle_A_error) < BREAKING_ANGLE)
				{	// Fase de desaceleración: el paso se reduce proporcionalmente al error actual
					joint_step_[0] = copysign(0.1 * (fabs(angle_A_error) / BREAKING_ANGLE), angle_A_error);
				}
				else
				{	// Fase de meseta: se utiliza un paso constante
					joint_step_[0] = copysign(0.1, angle_A_error);
				}
				// Se asegura que el paso mínimo para el eje A sea de MIN_JOINT_STEP 0.015
				if (fabs(joint_step_[0]) < MIN_JOINT_STEP)
				joint_step_[0] = copysign(MIN_JOINT_STEP, angle_A_error);
			}
			else
			{
				joint_step_[0] = 0;
			}
			// ROS_INFO("Angle A to go %f  error %f",cartesian_goal_pose_[3], angle_A_error);
			// ROS_INFO("First error A %f reqA6 %f", initial_angle_A_error_, req_A6);
			RSI_message.a = joint_step_[0];

			// --- Rotación de los ejes B y C ---
			// No hay rampas de aceleración/deceleración. Se mueve siempre a la mínima velocidad

			// Rotation of B angle
			if (fabs(angle_B_error) > 1.0)
			{
				joint_step_[1] = copysign(MIN_JOINT_STEP, angle_B_error);
			}
			else
			{
				joint_step_[1] = 0;
			}
			//RSI_message.b = joint_step_[1];
			// Rotation of C angle
			if (fabs(angle_C_error) > 1)
			{
				joint_step_[2] = copysign(MIN_JOINT_STEP, angle_C_error);
			}
			else
			{
				joint_step_[2] = 0;
			}
			//RSI_message.c = joint_step_[2];
			// ROS_INFO("Step angle C: %f Actual pose:%f Destination:%f",joint_step_[1],rsi_state_.cart_position[5],cartesian_goal_pose_[5] );
			
			// --- Comprobación del estado de movimiento del robot ---

			// Si la distancia restante es menor a 1 y los errores angulares son menores a 1,
			// o si el robot no se mueve durante demasiados ciclos, se finaliza el servicio.			
			if ((distance_remaining_ < 1 &&
				 fabs(angle_A_error) < 1 &&
				 fabs(angle_B_error) < 1 &&
				 fabs(angle_C_error) < 1) ||
				counter_not_moving_ >= MAX_CONT_NOT_MOVING)
			{ // Last loop of the service or it stopped 100 cycles of not moving interrupts the service
				cartesian_correction_request_ = false;
				robot_is_moving_msg_.data = false;
				A6_in_valid_range = true;
				ROS_INFO("LAST ITERATION");
			}

			// Si el robot no ha variado significativamente su posición y orientación 
			// desde la última comprobación, se incrementa un contador de inactividad.
			if (fabs(prev_distance_remaining_ - distance_remaining_) < 0.1 &&
				fabs(prev_angle_A_error - angle_A_error) < 0.001 &&
				fabs(prev_angle_B_error - angle_B_error) < 0.001 &&
				fabs(prev_angle_C_error - angle_C_error) < 0.001)
			{
				counter_not_moving_++;
				ROS_INFO("NOT MOVING, moved: %f angle: %f", 
					fabs(prev_distance_remaining_ - distance_remaining_), 
					fabs(prev_angle_A_error - angle_A_error));
			}
			else
			{
				ROS_INFO(" MOVING, moved: %f", fabs(prev_distance_remaining_ - distance_remaining_));
				prev_distance_remaining_ = distance_remaining_;
				prev_angle_A_error = angle_A_error;
				prev_angle_B_error = angle_B_error;
				prev_angle_C_error = angle_C_error;
				counter_not_moving_ = 0;
			}
			
		}
		//correction of A1 or A6 requested
		else if (joint_correction_request_)
		{
			float step_A1 = 0;
			float step_A6 = 0;
			// Se indica que el robot está en movimiento
			robot_is_moving_msg_.data = true;
			// ----- Cálculo de errores y desplazamientos para el eje A1 -----
			// Se calcula el error actual para A1 y A6 (objetivo menos posición actual)
			A1_current_error_ = joint_A1_goal_pose_ - rsi_state_.positions[0];
			A6_current_error_ = joint_A6_goal_pose_ - rsi_state_.positions[5];
			// Se calcula el desplazamiento acumulado en A1 y A6 desde el inicio del servicio
			A1_moved_from_start = fabs(rsi_state_.positions[0] - start_joint_pose_request_[0]);
			A6_moved_from_start = fabs(rsi_state_.positions[5] - start_joint_pose_request_[5]);
			//----------------A1-------------------
			// Si el error en A1 es mayor a 0.1 (umbral mínimo)
			if (fabs(A1_current_error_) > MIN_ERROR_THRESHOLD_A1)
			{// Fase de aceleración o desaceleración para el eje A1				
				if (A1_moved_from_start < (2 * BREAKING_ANGLE))
				{	// Durante la aceleración: el paso es proporcional al ángulo ya movido
					// slope for A1 angle
					step_A1 = copysign(MAX_STEP_A1 * (A1_moved_from_start / (2 * BREAKING_ANGLE)), A1_current_error_);
				}
				else if (fabs(A1_current_error_) <= (2 * BREAKING_ANGLE))
				{	// Durante la desaceleración: el paso es proporcional al error actual
					step_A1 = copysign(MAX_STEP_A1 * ((fabs(A1_current_error_)) / (2 * BREAKING_ANGLE)), A1_current_error_);
					// ROS_INFO("Arriving %f",sqrt(pow((A1_current_error_),2))/(2*BREAKING_ANGLE) );
				}
				else
				{	// En meseta: se utiliza el paso máximo definido para A1
					step_A1 = copysign(MAX_STEP_A1, A1_current_error_);
				}
				// Si el paso calculado es demasiado pequeño o si el error inicial era muy pequeño, se establece un paso mínimo
				if (fabs(step_A1) <= MIN_STEP_A1 || fabs(start_A1_error_request_) < 4 * BREAKING_ANGLE)
				{	// shorter than breaking angle, doing it slow, no slope
					step_A1 = copysign(MIN_STEP_A1, A1_current_error_); // A1 acc slower than A6 0.001 0.003					
				}
			}
			
			// Se asigna el paso calculado al vector de correcciones para el eje A1 (índice 6)
			RSI_message.a1 = step_A1;
			//----------------A6-------------------
			// Si el error en A6 es mayor a 1 (umbral mínimo)
			if (fabs(A6_current_error_) > MIN_ERROR_THRESHOLD_A6)
			{
				if (A6_moved_from_start < (2 * BREAKING_ANGLE))
				{ 	// Fase de aceleración para A6: el paso es proporcional al ángulo ya movido
					// slope for A6 angle
					step_A6 = copysign(MAX_STEP_A6 * (A6_moved_from_start / (2 * BREAKING_ANGLE)), A6_current_error_);
				}
				else if (sqrt(pow((A6_current_error_), 2)) < (2 * BREAKING_ANGLE))
				{ 	// Fase de desaceleración para A6: el paso se calcula en función del error actual
					step_A6 = copysign(MAX_STEP_A6 * (fabs(A6_current_error_) / (2 * BREAKING_ANGLE)), A6_current_error_);
				}
				else
				{	// En meseta: se usa un paso fijo
					step_A6 = copysign(MAX_STEP_A6, A6_current_error_);
				}
				// Se garantiza un paso mínimo para A6 (0.015) si el calculado es menor o si el error inicial es pequeño
				if (fabs(step_A6) < MIN_STEP_A6 || fabs(start_A6_error_request_) < 4 * BREAKING_ANGLE)
				{
					step_A6 = copysign(MIN_STEP_A6, A6_current_error_); 
				}
			}
			
			//ROS_INFO("step A6 %f, error A6 %f", step_A6, A6_current_error_);
			// Se asigna el paso calculado para A6 (índice 11) al vector de correcciones
			RSI_message.a6 = step_A6;

			// ----- Verificación del estado de movimiento -----
			// Check if it arrived or it stopped moving
			if (fabs(A1_current_error_) < 1 &&
				fabs(A6_current_error_) < 1 ||
				counter_not_moving_ >= MAX_CONT_NOT_MOVING)
			{ 	// Last loop of the service or it stopped 100 cycles of not moving interrupts the service
				// Si los errores son muy pequeños o ha pasado mucho tiempo sin movimiento, se considera que se alcanzó el objetivo.
				accumulated_A1_rotation_rad += deg2rad(rsi_state_.positions[0] - start_joint_pose_request_[0]);
				joint_correction_request_ = false;
				robot_is_moving_msg_.data = false;
				A6_in_valid_range = true;
				ROS_INFO("LAST ITERATION");
			}
			// Check if it is moving
			// Se verifica si no ha habido cambio desde la última iteración
			if (fabs(prev_A1_error - A1_current_error_) < MIN_STEP_A1 &&
				fabs(prev_A6_error - A6_current_error_) < MIN_STEP_A6)
			{
				counter_not_moving_++;
				ROS_INFO("NOT MOVING A1 %f A6 %f", fabs(prev_A1_error - A1_current_error_), fabs(prev_A6_error - A6_current_error_));
			}
			else
			{
				ROS_INFO(" MOVING");
				prev_A1_error = A1_current_error_;
				prev_A6_error = A6_current_error_;
				counter_not_moving_ = 0;
			}
			// Write part of the pad  that can move the robot in x,y,z and angle A
		}
		else if (!cartesian_correction_request_)
		{
			RSI_message.x = cartesian_pad_cmds_.x * cos(accumulated_A1_rotation_rad) - cartesian_pad_cmds_.y * sin(accumulated_A1_rotation_rad);
			RSI_message.y = cartesian_pad_cmds_.y * cos(accumulated_A1_rotation_rad) + cartesian_pad_cmds_.x * sin(accumulated_A1_rotation_rad);
			RSI_message.z = cartesian_pad_cmds_.z;

			// Limits in Z coming from the service, to block if overpressing
			if (z_force_limit_reached_ && RSI_message.z < 0)
			{
				RSI_message.z = 0.0;
				ROS_INFO("Blocking -Z");
			}
			//	limits of angle of the tool
			if ((rsi_state_.positions[5] >= UP_LIMIT_A6 && cartesian_pad_cmds_.yaw > 0) || 
			(rsi_state_.positions[5] <= LOW_LIMIT_A6 && cartesian_pad_cmds_.yaw < 0))
			{
				// ROS_INFO(" PAD: %f Posicion A:%f Axis6: %f",cartesian_pad_cmds_.yaw,rsi_state_.cart_position[3],rsi_state_.positions[5]);
				ROS_INFO("Limits of Angle A reached. PAD: %f Posicion A:%f Axis6: %f", cartesian_pad_cmds_.yaw, rsi_state_.cart_position[3], rsi_state_.positions[5]);
			}
			else
			{	// yaw
				RSI_message.a = cartesian_pad_cmds_.yaw;
			}
			// Axis movement
			// joint A1
			RSI_message.a1 = cartesian_pad_cmds_.A1;
			// joint A6
			RSI_message.a6 = cartesian_pad_cmds_.A6;
		}

		// Limits of -x to avoid wall collision. Taking into account temporal correction
		float x_disp_real = (RSI_message.x + 
			RSI_message.y * sin(accumulated_A1_rotation_rad)) / cos(accumulated_A1_rotation_rad); // corrected x
		if (rsi_state_.cart_position[0] <= MIN_X_LIMIT && x_disp_real < 0)
		{
			RSI_message.x = 0;
			RSI_message.y = 0;
			ROS_INFO("-x out of range");
		}
		if (rsi_state_.cart_position[2] >= MAX_Z_LIMIT && RSI_message.z > 0)
		{
			RSI_message.z = 0;
			ROS_INFO("+z out of range");
		}

		// out_buffer_ = RSICommand('R',RSI_message.toVector(), ipoc_).xml_doc;
		out_buffer_ = RSICommand(RSI_message.toVector(), ipoc_).xml_doc;		

		// ROS_INFO("Send to robot:%s", out_buffer_.c_str());
		server_->send(out_buffer_);

		robot_is_moving_pub_.publish(robot_is_moving_msg_);

		return true;
	}

	// Servicio para movimiento absoluto (normal)
	bool KukaHardwareInterface::setAbsoluteCartGoalPose(robotnik_msgs::set_CartesianEuler_pose::Request &req,
		robotnik_msgs::set_CartesianEuler_pose::Response &res)
	{
	return settingAbsoluteCartGoalPose(req, res, CARTESIAN_MOVE_VELOCITY_FACTOR);
	}

	// Servicio para movimiento absoluto (fast)
	bool KukaHardwareInterface::setAbsoluteCartGoalPoseFast(robotnik_msgs::set_CartesianEuler_pose::Request &req,
				robotnik_msgs::set_CartesianEuler_pose::Response &res)
	{
	return settingAbsoluteCartGoalPose(req, res, CARTESIAN_MOVE_FAST_VELOCITY_FACTOR);
	}

	bool KukaHardwareInterface::setRelativeCartGoalPose(robotnik_msgs::set_CartesianEuler_pose::Request &req,
		robotnik_msgs::set_CartesianEuler_pose::Response &res)
	{
	return settingRelativeCartGoalPose(req, res, CARTESIAN_MOVE_VELOCITY_FACTOR);
	}

	bool KukaHardwareInterface::setRelativeCartGoalPoseFast(robotnik_msgs::set_CartesianEuler_pose::Request &req,
				robotnik_msgs::set_CartesianEuler_pose::Response &res)
	{
	return settingRelativeCartGoalPose(req, res, CARTESIAN_MOVE_FAST_VELOCITY_FACTOR);
	}


	bool KukaHardwareInterface::moveJointsA1andA6(kuka_rsi_cartesian_hw_interface::set_A1_A6::Request &request, kuka_rsi_cartesian_hw_interface::set_A1_A6::Response &response)
	{
		counter_not_moving_ = 0;
		for (std::size_t i = 0; i < n_dof_; ++i) // position at the start of the service
		{
			start_joint_pose_request_[i] = rsi_state_.positions[i];
		}

		joint_A1_goal_pose_ = request.A1;
		joint_A6_goal_pose_ = request.A6;

		// A6 can go from 0 to 360 and A1 from -115 to 130
		if (joint_A6_goal_pose_ < A6_LOWER_LIMIT || joint_A6_goal_pose_ > A6_UPPER_LIMIT 
			|| joint_A1_goal_pose_ < A1_LOWER_LIMIT || joint_A1_goal_pose_ > A1_UPPER_LIMIT)
		{
			response.ret = false;
			return true;
		}

		prev_A1_error = joint_A1_goal_pose_ - start_joint_pose_request_[0];
		prev_A6_error = joint_A6_goal_pose_ - start_joint_pose_request_[5];

		start_A1_error_request_ = prev_A1_error;
		start_A6_error_request_ = prev_A6_error;
		joint_correction_request_ = true;
		response.ret = true;
		return true;
	}
	// Service to enable or disable the movement relative to the tool coordinates with the pad
	bool KukaHardwareInterface::setMoveRelTool(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
	{
		if (request.data == true)
		{
			move_relative_to_tool_ = true;
			response.success = true;
		}
		else if (request.data == false)
		{
			move_relative_to_tool_ = false;
			response.success = true;
		}
		return true;
	}
	
	// Topic to read the pressing weight made by the tool and enable to block the negative Z direction of movement of the pad
	void KukaHardwareInterface::phidgetCallback(const std_msgs::Float64::ConstPtr &force_z_axis)
	{
		// ROS_INFO("phidget received %f", force_z_axis->data);
		if (force_z_axis->data >= Z_FORCE_UPPER_LIMIT)
		{
			z_force_limit_reached_ = true;
		}
		else if (force_z_axis->data <= Z_FORCE_LOWER_LIMIT)
		{
			z_force_limit_reached_ = false;
		}
	}
	
	bool KukaHardwareInterface::settingRelativeCartGoalPose(
		robotnik_msgs::set_CartesianEuler_pose::Request &req,
		robotnik_msgs::set_CartesianEuler_pose::Response &res,
		float velocity_factor_param)
	{
		// Se reinicia el contador de ciclos sin movimiento
		counter_not_moving_ = 0;
		// Se guarda la posición inicial del robot (para ejes 2 en adelante) 
    	// que se usará como referencia para medir el desplazamiento
		for (std::size_t i = 2; i < n_dof_; ++i)
		{
			start_cartesian_pose_request_[i] = rsi_state_.cart_position[i];
		}
		// Se calculan los componentes X e Y de la posición inicial
    	// aplicando una transformación rotacional basada en accumulated_A1_rotation_rad
		start_cartesian_pose_request_[0] = rsi_state_.cart_position[0] * cos(accumulated_A1_rotation_rad) - 
										   rsi_state_.cart_position[1] * sin(accumulated_A1_rotation_rad);
		start_cartesian_pose_request_[1] = rsi_state_.cart_position[1] * cos(accumulated_A1_rotation_rad) + 
										   rsi_state_.cart_position[0] * sin(accumulated_A1_rotation_rad);
		// Calcula la posición meta sumando la posición inicial
		// Se calcula la posición objetivo sumando el desplazamiento relativo (del request)
	    // a la posición inicial. La transformación rotacional también se aplica.
		cartesian_goal_pose_[0] = req.x * cos(accumulated_A1_rotation_rad) - req.y * sin(accumulated_A1_rotation_rad) + start_cartesian_pose_request_[0];
		cartesian_goal_pose_[1] = req.y * cos(accumulated_A1_rotation_rad) + req.x * sin(accumulated_A1_rotation_rad) + start_cartesian_pose_request_[1];
		cartesian_goal_pose_[2] = req.z + start_cartesian_pose_request_[2];
		cartesian_goal_pose_[3] = req.A + start_cartesian_pose_request_[3];
		cartesian_goal_pose_[4] = req.B + start_cartesian_pose_request_[4];
		cartesian_goal_pose_[5] = req.C + start_cartesian_pose_request_[5];

	 	// Se calcula la distancia total a recorrer (solo considerando traslación)
		total_distance_to_cover_ = sqrt(
			pow((cartesian_goal_pose_[0] - start_cartesian_pose_request_[0]), 2) +
			pow((cartesian_goal_pose_[1] - start_cartesian_pose_request_[1]), 2) +
			pow((cartesian_goal_pose_[2] - start_cartesian_pose_request_[2]), 2));
		// Se guarda la distancia inicial para el control del movimiento
		prev_distance_remaining_ = total_distance_to_cover_;
		// Se calculan los errores angulares entre la meta y la posición inicial para los ángulos A, B y C
		prev_angle_A_error = cartesian_goal_pose_[3] - start_cartesian_pose_request_[3];
		prev_angle_B_error = cartesian_goal_pose_[4] - start_cartesian_pose_request_[4];
		prev_angle_C_error = cartesian_goal_pose_[5] - start_cartesian_pose_request_[5];
		// Normalización del error del ángulo C para que esté entre -180 y 180 grados
		prev_angle_C_error = normalizeAngleDeg(prev_angle_C_error);
		/*if (prev_angle_C_error < -180)
			prev_angle_C_error += 360;
		else if (prev_angle_C_error > 180)
			prev_angle_C_error -= 360;*/
		// Se calcula el paso absoluto en mm basado en la velocidad, el ciclo de tiempo y el factor de velocidad
		float step_abs = ROBOT_VELOCITY * T_CYC * velocity_factor_param; // en mm
		// Se guarda la posición inicial del eje A6 y se calcula el valor solicitado para A6
		pos_init_A6 = rsi_state_.positions[5];
		req_A6 = rsi_state_.positions[5] + prev_angle_A_error;
		// Si el ángulo solicitado para A6 está fuera de los límites permitidos, se ajusta el error 
		if (req_A6 >= UP_LIMIT_A6 || req_A6 <= LOW_LIMIT_A6)
		{
			// Se ajusta el error angular para A, invirtiendo el signo proporcionalmente
			prev_angle_A_error = prev_angle_A_error + 360 * (-prev_angle_A_error) / fabs(prev_angle_A_error);
			initial_angle_A_error_ = prev_angle_A_error;
			A6_in_valid_range = false;
		}
		else
		{
			A6_in_valid_range = true;
			ROS_INFO("kuka_hardware_interface::LIMIT of A6 reached!");
		}
		// Se calcula el paso incremental para cada eje de traslación (índices 0, 1 y 2)
		for (std::size_t i = 0; i < n_dof_ - 3; ++i)
		{
			if (total_distance_to_cover_ < 1)
				cartesian_step_[i] = 0;
			else
				cartesian_step_[i] = step_abs * (cartesian_goal_pose_[i] - start_cartesian_pose_request_[i]) / total_distance_to_cover_;
		}
		// Se activa el modo de correcciones cartesianas
		cartesian_correction_request_ = true;
		res.ret = true;
		return true;
	}
	
	// Función común para configurar la meta (goal) en modo absoluto
	bool KukaHardwareInterface::settingAbsoluteCartGoalPose(
		robotnik_msgs::set_CartesianEuler_pose::Request &req,
		robotnik_msgs::set_CartesianEuler_pose::Response &res,
		float velocity_factor_param)
	{
		counter_not_moving_ = 0;

		// Guardar la posición de referencia (para índices 2 en adelante)
		for (std::size_t i = 2; i < n_dof_; ++i) {
			start_cartesian_pose_request_[i] = rsi_state_.cart_position[i];
		}
		// Transformar las componentes X e Y de la posición inicial
		start_cartesian_pose_request_[0] = rsi_state_.cart_position[0] * cos(accumulated_A1_rotation_rad) -
											rsi_state_.cart_position[1] * sin(accumulated_A1_rotation_rad);
		start_cartesian_pose_request_[1] = rsi_state_.cart_position[1] * cos(accumulated_A1_rotation_rad) +
											rsi_state_.cart_position[0] * sin(accumulated_A1_rotation_rad);
		// Para modo absoluto se usa la posición proporcionada (sin sumarle la posición de inicio)
		cartesian_goal_pose_[0] = req.x * cos(accumulated_A1_rotation_rad) - req.y * sin(accumulated_A1_rotation_rad);
		cartesian_goal_pose_[1] = req.y * cos(accumulated_A1_rotation_rad) + req.x * sin(accumulated_A1_rotation_rad);
		cartesian_goal_pose_[2] = req.z;
		cartesian_goal_pose_[3] = req.A;
		cartesian_goal_pose_[4] = req.B;
		cartesian_goal_pose_[5] = req.C;

		total_distance_to_cover_ = sqrt(
			pow((cartesian_goal_pose_[0] - start_cartesian_pose_request_[0]), 2) +
			pow((cartesian_goal_pose_[1] - start_cartesian_pose_request_[1]), 2) +
			pow((cartesian_goal_pose_[2] - start_cartesian_pose_request_[2]), 2));

		// (Opcional: cálculos intermedios de distancias, si se desean)
		float dist_start_end = sqrt(
			pow((cartesian_goal_pose_[0] - start_cartesian_pose_request_[0]), 2) +
			pow((cartesian_goal_pose_[1] - start_cartesian_pose_request_[1]), 2));
		float dist_origin_end = sqrt(
			pow((cartesian_goal_pose_[0]), 2) + pow((cartesian_goal_pose_[1]), 2));
		float dist_origin_start = sqrt(
			pow((start_cartesian_pose_request_[0]), 2) + pow((start_cartesian_pose_request_[1]), 2));

		// Cálculo de la traslación angular para A6 (ajuste por la traslación)
		float tras_A6 = (atan2(cartesian_goal_pose_[1], cartesian_goal_pose_[0]) -
						atan2(start_cartesian_pose_request_[1], start_cartesian_pose_request_[0])) * 180 / M_PI;

		prev_distance_remaining_ = total_distance_to_cover_;
		prev_angle_A_error = cartesian_goal_pose_[3] - start_cartesian_pose_request_[3];
		prev_angle_B_error = cartesian_goal_pose_[4] - start_cartesian_pose_request_[4];
		prev_angle_C_error = cartesian_goal_pose_[5] - start_cartesian_pose_request_[5];

		// Normalización de errores angulares
		prev_angle_C_error = normalizeAngleDeg(prev_angle_C_error);
		/*if (prev_angle_C_error < -180)
			prev_angle_C_error += 360;
		else if (prev_angle_C_error > 180)
			prev_angle_C_error -= 360;
		*/
		prev_angle_A_error = normalizeAngleDeg(prev_angle_A_error);
		/*if (prev_angle_A_error < -180)
			prev_angle_A_error += 360;
		else if (prev_angle_A_error > 180)
			prev_angle_A_error -= 360;
		*/
		float step_abs = ROBOT_VELOCITY * T_CYC * velocity_factor_param; // en mm

		// Para algunos errores se guardan los valores iniciales
		initial_angle_A_error_ = prev_angle_A_error;
		first_angle_B_error = prev_angle_B_error;
		if (fabs(first_angle_B_error) <= 1) {
			first_angle_B_error = 0;
			prev_angle_B_error = 0;
		}
		first_angle_C_error = prev_angle_C_error;
		if (fabs(first_angle_C_error) <= 1) {
			first_angle_C_error = 0;
			prev_angle_C_error = 0;
		}

		pos_init_A6 = rsi_state_.positions[5];
		req_A6 = rsi_state_.positions[5] + prev_angle_A_error - tras_A6;
		// Ajuste para A6 si el valor solicitado está fuera de límites
		if (req_A6 >= UP_LIMIT_A6 || req_A6 <= LOW_LIMIT_A6) {
			prev_angle_A_error = prev_angle_A_error + 360 * (-prev_angle_A_error) / fabs(prev_angle_A_error);
			initial_angle_A_error_ = prev_angle_A_error;
			A6_in_valid_range = false;
		} else {
			A6_in_valid_range = true;
		}

		for (std::size_t i = 0; i < n_dof_ - 3; ++i) {
			if (total_distance_to_cover_ < 1)
				cartesian_step_[i] = 0;
			else
				cartesian_step_[i] = step_abs * (cartesian_goal_pose_[i] - start_cartesian_pose_request_[i]) / total_distance_to_cover_;
		}
		// Se activa el modo de correcciones cartesianas
		cartesian_correction_request_ = true;
		res.ret = true;
		return true;
	}
	
}
