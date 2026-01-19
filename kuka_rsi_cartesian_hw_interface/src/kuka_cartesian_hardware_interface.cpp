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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_cartesian_hw_interface/kuka_cartesian_hardware_interface.h>
#include "joint_state_controller/joint_state_controller.h"
#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <stdexcept>
#include <robotnik_msgs/set_odometry.h>
#include <cmath>
#include <sys/time.h>

 struct timespec  tvalBefore1, tvalAfter1, tvalMid, tvalMid2;
 float total_distance_service=0;
namespace kuka_rsi_cartesian_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0), joint_position_command_(6, 0.0), joint_velocity_command_(
        6, 0.0), joint_effort_command_(6, 0.0), joint_names_(6), rsi_initial_joint_positions_(6, 0.0), rsi_joint_position_corrections_(
        6, 0.0), ipoc_(0), n_dof_(6), cartesian_pad_cmds_(6, 0.0), rsi_abs_cart_correction_(6, 0.0)
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
  if (!nh_.getParam("publish_rate", publish_rate_)){
    ROS_ERROR("Parameter 'publish_rate' not set");
    throw std::runtime_error("Cannot find required parameter ");
  }
  
  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh_, "joint_states", 4));
  //publishes the cartesian position of the robot
  cart_pos_pub=nh_.advertise<robotnik_msgs::Cartesian_Euler_pose>("cartesian_pos_kuka",10);
  //publishes if the robot is moving through service 
  kuka_moving_pub=nh_.advertise<std_msgs::Bool>("kuka_moving",10);
  
  
  //subscriber
  pad_subs = nh_.subscribe<robotnik_trajectory_pad::CartesianEuler>("/kuka_pad/cartesian_move", 1, &KukaHardwareInterface::padcallback, this);
  //service
  set_kuka_odometry_abs=nh_.advertiseService("setKukaAbs",&KukaHardwareInterface::setKukaOdometry_abs,this);
  set_kuka_odometry_rel=nh_.advertiseService("setKukaRel",&KukaHardwareInterface::setKukaOdometry_rel,this);
  set_kuka_odometry_abs_fast=nh_.advertiseService("setKukaAbsFast",&KukaHardwareInterface::setKukaOdometry_abs_fast,this);
  set_kuka_odometry_rel_fast=nh_.advertiseService("setKukaRelFast",&KukaHardwareInterface::setKukaOdometry_rel_fast,this);

}

KukaHardwareInterface::~KukaHardwareInterface()
{

}
//callback from topic /cartesian_move
void KukaHardwareInterface::padcallback(const robotnik_trajectory_pad::CartesianEuler::ConstPtr& cartesian_move)
{

  // TRANSFORMATION FOR TOOL ORIENTATION angles in radians
  rot_A=rsi_state_.cart_position[3]+90;
  cartesian_pad_cmds_[0]=cartesian_move->x*cos(rot_A*M_PI/180)-cartesian_move->y*sin(rot_A*M_PI/180);
  cartesian_pad_cmds_[1]=cartesian_move->y*cos(rot_A*M_PI/180)+cartesian_move->x*sin(rot_A*M_PI/180);
  
  cartesian_pad_cmds_[2]=cartesian_move->z;
  cartesian_pad_cmds_[3]=cartesian_move->pitch;
  cartesian_pad_cmds_[4]=cartesian_move->roll;
  cartesian_pad_cmds_[5]=cartesian_move->yaw;
  
}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
	clock_gettime(CLOCK_REALTIME,&tvalMid2);
	in_buffer_.resize(1024);

	if (server_->recv(in_buffer_) == 0)
		{
		return false;
	}
	//ROS_INFO("Received from robot:%s", in_buffer_.c_str());
	if (rt_rsi_pub_->trylock()){
		rt_rsi_pub_->msg_.data = in_buffer_;
		rt_rsi_pub_->unlockAndPublish();
	}

	rsi_state_ = RSIState(in_buffer_);
	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){
		// try to publish
		if (realtime_pub_->trylock()){
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
			realtime_pub_->msg_.header.stamp = time;
			//ROS_INFO("READ time: %f", (-last_publish_time_.toSec()+time.toSec()));
			//update and publish by /joint_states
			for (std::size_t i = 0; i < n_dof_; ++i)		
			{
				realtime_pub_->msg_.position[i] = DEG2RAD * rsi_state_.positions[i];

				realtime_pub_->msg_.velocity[i] = 0;

				realtime_pub_->msg_.effort[i] = 0;

			}
		//Absolute cartesian pos of the robot
		cart_pos.x=rsi_state_.cart_position[0];
		cart_pos.y=rsi_state_.cart_position[1];
		cart_pos.z=rsi_state_.cart_position[2];
		cart_pos.A=rsi_state_.cart_position[3];
		cart_pos.B=rsi_state_.cart_position[4];
		cart_pos.C=rsi_state_.cart_position[5];
			
		
		
		realtime_pub_->unlockAndPublish();
		cart_pos_pub.publish(cart_pos);
		}
	} 
	ipoc_ = rsi_state_.ipoc;

	return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
	out_buffer_.resize(1024);
	
	for (std::size_t i = 0; i < n_dof_; ++i)
	{
		rsi_abs_cart_correction_[i]=0;
		rsi_joint_position_corrections_[i]=rsi_abs_cart_correction_[i];
	}

	if((service_set_kuka_abs || service_set_kuka_rel)){
		first_time=true;
		//In service
		msgs_kuka_moving.data=true;
		slope=1;
		
		distance_from_start=sqrt(
			pow((rsi_state_.cart_position[0]-pose_init_[0]),2)+
			pow((rsi_state_.cart_position[1]-pose_init_[1]),2)+
			pow((rsi_state_.cart_position[2]-pose_init_[2]),2));
			
		distance_to_end=sqrt(
			pow((-rsi_state_.cart_position[0]+aut_cmds_[0]),2)+
			pow((-rsi_state_.cart_position[1]+aut_cmds_[1]),2)+
			pow((-rsi_state_.cart_position[2]+aut_cmds_[2]),2));
		//ROS_INFO(" In Service distance from start:%f distance to end: %f",distance_from_start,distance_to_end);
		
		angle_A_error=-rsi_state_.cart_position[3] + aut_cmds_[3];
		if(range_A6){
			angle_A_moved_from_start=sqrt(
				pow((rsi_state_.cart_position[3]-pose_init_[3]),2));
		}else{
			//take it from the axis 6
			angle_A_moved_from_start=sqrt(
				pow((rsi_state_.positions[5]-pos_init_A6),2));
		}
		angle_B_error=-rsi_state_.cart_position[4] + aut_cmds_[4];
		angle_C_moved_from_start=sqrt(
			pow((rsi_state_.cart_position[4]-pose_init_[4]),2));
		angle_C_error=-rsi_state_.cart_position[5] + aut_cmds_[5];
		angle_C_moved_from_start=sqrt(
			pow((rsi_state_.cart_position[5]-pose_init_[5]),2));

		//A,B,C moves between [-179,179]
		
		if(angle_A_error < -180){
			angle_A_error = angle_A_error + 360;
		}else if(angle_A_error > 180)
			angle_A_error = angle_A_error - 360;
			
		if(angle_B_error < -180){
			angle_B_error = angle_B_error + 360;
		}else if(angle_B_error > 180)
			angle_B_error = angle_B_error - 360;			
				
		if(angle_C_error < -180){
			angle_C_error = angle_C_error + 360;
		}else if(angle_C_error > 180)
			angle_C_error = angle_C_error - 360;	
				
		
	
		//TRANSLATION
		if(distance_from_start<=breaking_distance && distance_to_end<breaking_distance){ //trajectory is shorter than breaking distance
			slope=0.05;
			//ROS_INFO("Too short");
		}else if(distance_from_start<=breaking_distance){ //acceleration part
			slope=distance_from_start/breaking_distance;
		}else if(distance_to_end<breaking_distance){ //deceleration part
			slope=distance_to_end/breaking_distance;
		}else{ //middle part of the trajectory
			slope=1;
		}
		if(slope<0.05){ //to make the start faster 0.05
			slope=0.05;
		}
		
		if(distance_from_start>=total_distance_service){ //if it arrived to the desired position or it went further
				slope=0;
		}
		//ROS_INFO("Slope: %f",slope);
		for (std::size_t i = 0; i < n_dof_-3; ++i)//x,y,z,
		{
		
			rsi_abs_cart_correction_[i]=step_tr[i]*slope; 
			rsi_joint_position_corrections_[i]=rsi_abs_cart_correction_[i];
		
		}
		
		//Check if A will be out of range
		if(!range_A6){
			angle_A_error=first_angle_A_error-angle_A_moved_from_start*copysign(1,first_angle_A_error); 
			ROS_INFO("trajectory with A6 out of range angle error: %f first angle error: %f",angle_A_error,first_angle_A_error);
		}
		//Rotation of A angle
		if(sqrt(pow((angle_A_error),2))>1){
			//if(range_A6){

					if(angle_A_moved_from_start<breaking_angle){
						//slope for A angle
						step_tr[3]=copysign(0.1*(angle_A_moved_from_start/breaking_angle),angle_A_error);
					}else if(sqrt(pow((angle_A_error),2))<breaking_angle){
						step_tr[3]=copysign(0.1*(sqrt(pow((angle_A_error),2))/breaking_angle),angle_A_error);
					}else{
						step_tr[3]=copysign(0.1,angle_A_error);
					}
					if(sqrt(pow(step_tr[3],2))<sqrt(pow(0.015,2)))
						step_tr[3]=copysign(0.015,angle_A_error);
			//}
			/*else{
				step_tr[3]=copysign(0.02,first_angle_A_error);
				ROS_INFO("trajectory with A6 out of range step: %f",step_tr[3]);
			}*/
		
		
		}else{
			step_tr[3]=0;
		}

		rsi_abs_cart_correction_[3]=step_tr[3];
		rsi_joint_position_corrections_[3]=rsi_abs_cart_correction_[3];
		
		//Rotation of B angle
		if(sqrt(pow((angle_B_error),2))>1){
			if(rsi_state_.cart_position[3]<-90){
				step_tr[4]=copysign(0.01,-angle_B_error);
			}else
				{
				step_tr[4]=copysign(0.01,angle_B_error);
				}
				
		}else{
			step_tr[4]=0;
		}
		//rsi_abs_cart_correction_[4]=step_tr[4];
		//rsi_joint_position_corrections_[4]=rsi_abs_cart_correction_[4];
		
		//ROS_INFO("Step angle B: %f Actual pose:%f Destination:%f",step_tr[4],rsi_state_.cart_position[4],aut_cmds_[4] );
		
		//Rotation of C angle 
		if(sqrt(pow((angle_C_error),2))>1){
			if(rsi_state_.cart_position[3]<-90){
				step_tr[5]=copysign(0.015,-angle_C_error);
			}else
				{
				step_tr[5]=copysign(0.015,angle_C_error);
				}
		}else{
			step_tr[5]=0;
		}
		//rsi_abs_cart_correction_[5]=step_tr[5];
		//rsi_joint_position_corrections_[5]=rsi_abs_cart_correction_[5];
		//ROS_INFO("Step angle C: %f Actual pose:%f Destination:%f",step_tr[5],rsi_state_.cart_position[5],aut_cmds_[5] );

		if((distance_to_end<1 && 
		sqrt(pow((angle_A_error),2))<1 && 
		sqrt(pow((angle_B_error),2))<1 && 
		sqrt(pow((angle_C_error),2))<1)
		|| counter_not_moving>=100){ //Last loop of the service or it stopped 200 cycles of not moving interrupts the service
			service_set_kuka_abs=false;
			service_set_kuka_rel=false;
			msgs_kuka_moving.data=false;
			range_A6=true;
			ROS_INFO("LAST ITERATION");
		}
		
		if(sqrt(pow((prev_distance_to_end-distance_to_end),2)) < 0.1 && 
		sqrt(pow((prev_angle_A_error-angle_A_error),2)) < 0.001 &&
		sqrt(pow((prev_angle_B_error-angle_B_error),2)) < 0.001 &&
		sqrt(pow((prev_angle_C_error-angle_C_error),2)) < 0.001
		){ 
			counter_not_moving++;
			ROS_INFO("NOT MOVING, moved: %f angle: %f",sqrt(pow((prev_distance_to_end-distance_to_end),2)),sqrt(pow((prev_angle_A_error-angle_A_error),2)));
		}else{
			ROS_INFO(" MOVING, moved: %f",sqrt(pow((prev_distance_to_end-distance_to_end),2)));
			prev_distance_to_end=distance_to_end;	
			prev_angle_A_error=angle_A_error;
			prev_angle_B_error=angle_B_error;
			prev_angle_C_error=angle_C_error;
			counter_not_moving=0;
		}
		
	}else if(!service_set_kuka_abs && !service_set_kuka_rel){
		
		for (std::size_t i = 0; i < n_dof_-3; ++i)
		{
			rsi_abs_cart_correction_[i]=cartesian_pad_cmds_[i];
			rsi_joint_position_corrections_[i]=rsi_abs_cart_correction_[i];
			
		}
		ROS_INFO(" NOT In Service, PAD");
		//in kuka [4] is yaw [5] is pitch [6] ir roll
		//rsi_abs_cart_correction_[3]=rsi_abs_cart_correction_[3]+cartesian_pad_cmds_[5];

		/*if(((rsi_state_.cart_position[3]>98 or rsi_state_.cart_position[3]<-2) and cartesian_pad_cmds_[5]>=0)
		or ((rsi_state_.cart_position[3]<0 or rsi_state_.cart_position[3]>100) and cartesian_pad_cmds_[5]<=0)){*/
		
		//	limits of angle of the tool
		if((rsi_state_.positions[5]>=upper_limit_A6 && cartesian_pad_cmds_[5]>0) || (rsi_state_.positions[5]<=lower_limit_A6 && cartesian_pad_cmds_[5]<0)){ 
			//ROS_INFO(" PAD: %f Posicion A:%f Axis6: %f",cartesian_pad_cmds_[5],rsi_state_.cart_position[3],rsi_state_.positions[5]);
			ROS_INFO("Limits of Angle A reached. PAD: %f Posicion A:%f Axis6: %f",cartesian_pad_cmds_[5],rsi_state_.cart_position[3],rsi_state_.positions[5]);
		}else{
			rsi_abs_cart_correction_[3]=cartesian_pad_cmds_[5];
			rsi_joint_position_corrections_[3]=rsi_abs_cart_correction_[3];
			}
		//}
		//if(((rsi_state_.cart_position[4]>-10 and rsi_state_.cart_position[4]<9) and cartesian_pad_cmds_[4]>=0)
		//or ((rsi_state_.cart_position[4]<10 and rsi_state_.cart_position[4]>-9) and cartesian_pad_cmds_[4]<=0)){
			//rsi_abs_cart_correction_[4]=rsi_abs_cart_correction_[4]+cartesian_pad_cmds_[4];
			rsi_abs_cart_correction_[4]=cartesian_pad_cmds_[4];
			rsi_joint_position_corrections_[4]=rsi_abs_cart_correction_[4];
		/*}else
			ROS_INFO(" LIMITE ALCANZADO EN B! Porque poseB:%f y PAD: %f",rsi_state_.cart_position[4],cartesian_pad_cmds_[4]);
		if(((rsi_state_.cart_position[5]>170 or rsi_state_.cart_position[5]<-169)and cartesian_pad_cmds_[3]>=0)
		or ((rsi_state_.cart_position[5]>169 or rsi_state_.cart_position[5]<-170)and cartesian_pad_cmds_[3]<=0)){*/
			//rsi_abs_cart_correction_[5]=rsi_abs_cart_correction_[5]+cartesian_pad_cmds_[3];
			rsi_abs_cart_correction_[5]=cartesian_pad_cmds_[3];
			rsi_joint_position_corrections_[5]=rsi_abs_cart_correction_[5];
		//}else
		//	ROS_INFO(" LIMITE ALCANZADO EN C! Porque poseC:%f y PAD: %f",rsi_state_.cart_position[5],cartesian_pad_cmds_[3]);
	}

	out_buffer_ = RSICommand('R',rsi_joint_position_corrections_ , ipoc_).xml_doc;
	if(first_time){
			//ROS_INFO(" NOT In Service, PAD");
			ROS_INFO("Send to robot:%s", out_buffer_.c_str());
			first_time=false;
		}

	server_->send(out_buffer_);
	kuka_moving_pub.publish(msgs_kuka_moving);
  
  return true;
}


//service for abs coord
bool KukaHardwareInterface::setKukaOdometry_abs(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	

	counter_not_moving=0;
	for (std::size_t i = 0; i < n_dof_; ++i) //position at the start of the service
	{
		pose_init_[i]=rsi_state_.cart_position[i];
	}
	
	//end position wanted
	aut_cmds_[0]=request.x;
	aut_cmds_[1]=request.y;
	aut_cmds_[2]=request.z;
	aut_cmds_[3]=request.A;
	aut_cmds_[4]=request.B;
	aut_cmds_[5]=request.C;
	velocity_factor=1;
	
	total_distance_service=sqrt(
			pow((aut_cmds_[0]-pose_init_[0]),2)+
			pow((aut_cmds_[1]-pose_init_[1]),2)+
			pow((aut_cmds_[2]-pose_init_[2]),2));

	prev_distance_to_end=total_distance_service;
	prev_angle_A_error=aut_cmds_[3]-pose_init_[3];
	prev_angle_B_error=aut_cmds_[4]-pose_init_[4];
	prev_angle_C_error=aut_cmds_[5]-pose_init_[5];
	if(prev_angle_C_error<-180){
		prev_angle_C_error=prev_angle_C_error+360;
	}else if(prev_angle_C_error>180){
		prev_angle_C_error=prev_angle_C_error-360;	
	}
	if(prev_angle_A_error<-180){
		prev_angle_A_error=prev_angle_A_error+360;
	}else if(prev_angle_A_error>180){
		prev_angle_A_error=prev_angle_A_error-360;	
	}
	float step_abs=velocity_trajectory_kuka*t_cyc*velocity_factor; //in mm
	
	pos_init_A6=rsi_state_.positions[5];
	float req_A6=rsi_state_.positions[5]+prev_angle_A_error;
	//out of range A6
	if(req_A6>=upper_limit_A6 || req_A6<=lower_limit_A6){
		prev_angle_A_error=prev_angle_A_error+360*(-prev_angle_A_error)/fabs(prev_angle_A_error);
		first_angle_A_error=prev_angle_A_error;
		range_A6=false;
	}else{
		range_A6=true;
	}
	
	
	 for (std::size_t i = 0; i < n_dof_-3; ++i) //division of the steps in distance to steps in axis
	{
			if(total_distance_service<1){
				step_tr[i]=0;
			}else{
				step_tr[i]=step_abs*(aut_cmds_[i]-pose_init_[i])/(total_distance_service);
			}
			
	}
	
  service_set_kuka_abs=true;
  response.ret=true;
  return true;
}
//service for abs coord
bool KukaHardwareInterface::setKukaOdometry_abs_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
	counter_not_moving=0;

	for (std::size_t i = 0; i < n_dof_; ++i) //position at the start of the service
	{
		pose_init_[i]=rsi_state_.cart_position[i];
	}
	
	//end position wanted
	aut_cmds_[0]=request.x;
	aut_cmds_[1]=request.y;
	aut_cmds_[2]=request.z;
	aut_cmds_[3]=request.A;
	aut_cmds_[4]=request.B;
	aut_cmds_[5]=request.C;
	velocity_factor=4.5;
	
	total_distance_service=sqrt(
			pow((aut_cmds_[0]-pose_init_[0]),2)+
			pow((aut_cmds_[1]-pose_init_[1]),2)+
			pow((aut_cmds_[2]-pose_init_[2]),2));
	prev_distance_to_end=total_distance_service;
	prev_angle_A_error=aut_cmds_[3]-pose_init_[3];
	prev_angle_B_error=aut_cmds_[4]-pose_init_[4];
	prev_angle_C_error=aut_cmds_[5]-pose_init_[5];
	if(prev_angle_C_error<-180){
		prev_angle_C_error=prev_angle_C_error+360;
	}else if(prev_angle_C_error>180)
		prev_angle_C_error=prev_angle_C_error-360;
	if(prev_angle_A_error<-180){
		prev_angle_A_error=prev_angle_A_error+360;
	}else if(prev_angle_A_error>180){
		prev_angle_A_error=prev_angle_A_error-360;	
	}
	float step_abs=velocity_trajectory_kuka*t_cyc*velocity_factor; //in mm
	pos_init_A6=rsi_state_.positions[5];
	float req_A6=rsi_state_.positions[5]+prev_angle_A_error;
	//out of range A6
	if(req_A6>=upper_limit_A6 || req_A6<=lower_limit_A6){
		prev_angle_A_error=prev_angle_A_error+360*(-prev_angle_A_error)/fabs(prev_angle_A_error);
		first_angle_A_error=prev_angle_A_error;
		range_A6=false;
	}else{
		range_A6=true;
	}
	
	
	 for (std::size_t i = 0; i < n_dof_-3; ++i) //division of the steps in distance to steps in axis
	{
			if(total_distance_service<1){
				step_tr[i]=0;
			}else{
				step_tr[i]=step_abs*(aut_cmds_[i]-pose_init_[i])/(total_distance_service);
			}
			
			
	}
		
  service_set_kuka_abs=true;
  response.ret=true;
  return true;
	
  response.ret=true;
  return true;
}

//service for rel coord
bool KukaHardwareInterface::setKukaOdometry_rel(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
		counter_not_moving=0;
		for (std::size_t i = 0; i < n_dof_; ++i) //position at the start of the service
	{
		pose_init_[i]=rsi_state_.cart_position[i];
	}
	
	//end position wanted
	aut_cmds_[0]=request.x+pose_init_[0];
	aut_cmds_[1]=request.y+pose_init_[1];
	aut_cmds_[2]=request.z+pose_init_[2];
	aut_cmds_[3]=request.A+pose_init_[3];
	aut_cmds_[4]=request.B+pose_init_[4];
	aut_cmds_[5]=request.C+pose_init_[5];
	velocity_factor=1;
	
	
	total_distance_service=sqrt(
			pow((aut_cmds_[0]-pose_init_[0]),2)+
			pow((aut_cmds_[1]-pose_init_[1]),2)+
			pow((aut_cmds_[2]-pose_init_[2]),2));
	
	prev_distance_to_end=total_distance_service;
	prev_angle_A_error=aut_cmds_[3]-pose_init_[3];
	prev_angle_B_error=aut_cmds_[4]-pose_init_[4];
	prev_angle_C_error=aut_cmds_[5]-pose_init_[5];
	if(prev_angle_C_error<-180){
		prev_angle_C_error=prev_angle_C_error+360;
	}else if(prev_angle_C_error>180)
		prev_angle_C_error=prev_angle_C_error-360;
		
	float step_abs=velocity_trajectory_kuka*t_cyc*velocity_factor; //in mm
	
	pos_init_A6=rsi_state_.positions[5];
	float req_A6=rsi_state_.positions[5]+prev_angle_A_error;
	//out of range A6
	if(req_A6>=upper_limit_A6 || req_A6<=lower_limit_A6){
		prev_angle_A_error=prev_angle_A_error+360*(-prev_angle_A_error)/fabs(prev_angle_A_error);
		first_angle_A_error=prev_angle_A_error;
		range_A6=false;
	}else{
		range_A6=true;
	}
	
	
	
	
	 for (std::size_t i = 0; i < n_dof_-3; ++i) //division of the steps in distance to steps in axis
	{

			if(total_distance_service<1){
				step_tr[i]=0;
			}else{
				step_tr[i]=step_abs*(aut_cmds_[i]-pose_init_[i])/(total_distance_service);
			}
			
			
	}
	service_set_kuka_rel=true;
  response.ret=true;
  return true;
}

//service for rel coord
bool KukaHardwareInterface::setKukaOdometry_rel_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
			counter_not_moving=0;
			for (std::size_t i = 0; i < n_dof_; ++i) //position at the start of the service
	{
		pose_init_[i]=rsi_state_.cart_position[i];
	}
	
	//end position wanted
	aut_cmds_[0]=request.x+pose_init_[0];
	aut_cmds_[1]=request.y+pose_init_[1];
	aut_cmds_[2]=request.z+pose_init_[2];
	aut_cmds_[3]=request.A+pose_init_[3];
	aut_cmds_[4]=request.B+pose_init_[4];
	aut_cmds_[5]=request.C+pose_init_[5];
	velocity_factor=2;

	total_distance_service=sqrt(
			pow((aut_cmds_[0]-pose_init_[0]),2)+
			pow((aut_cmds_[1]-pose_init_[1]),2)+
			pow((aut_cmds_[2]-pose_init_[2]),2));
	
	prev_distance_to_end=total_distance_service;
	prev_angle_A_error=aut_cmds_[3]-pose_init_[3];
	prev_angle_B_error=aut_cmds_[4]-pose_init_[4];
	prev_angle_C_error=aut_cmds_[5]-pose_init_[5];
	if(prev_angle_C_error<-180){
		prev_angle_C_error=prev_angle_C_error+360;
	}else if(prev_angle_C_error>180)
		prev_angle_C_error=prev_angle_C_error-360;
			
	float step_abs=velocity_trajectory_kuka*t_cyc*velocity_factor; //in mm
	
	pos_init_A6=rsi_state_.positions[5];
	float req_A6=rsi_state_.positions[5]+prev_angle_A_error;
	//out of range A6
	if(req_A6>=upper_limit_A6 || req_A6<=lower_limit_A6){
		prev_angle_A_error=prev_angle_A_error+360*(-prev_angle_A_error)/fabs(prev_angle_A_error);
		first_angle_A_error=prev_angle_A_error;
		range_A6=false;
	}else{
		range_A6=true;
	}
	
	
	 for (std::size_t i = 0; i < n_dof_-3; ++i) //division of the steps in distance to steps in axis
	{

			if(total_distance_service<1){
				step_tr[i]=0;
			}else{
				step_tr[i]=step_abs*(aut_cmds_[i]-pose_init_[i])/(total_distance_service);
			}
			
			
	}
	service_set_kuka_rel=true;
	response.ret=true;
	return true;
}


void KukaHardwareInterface::start()
{
	//for the service
	service_set_kuka_abs=false; 
	service_set_kuka_rel=false;
	msgs_kuka_moving.data=false;
	
	
	velocity_trajectory_kuka=7.5; // mm/ms 
    t_cyc=0.12; // milisec
	velocity_factor=1;
	counter_not_moving=0;
	breaking_distance=150; //in mm 150
	breaking_angle=5; //deg
	
	upper_limit_A6=14;
	lower_limit_A6=-349;
	range_A6=true;
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

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
    rsi_initial_joint_positions_[i] = rsi_state_.initial_cart_position[i];

  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand('R',rsi_initial_joint_positions_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000); 
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");
  // initialize time
  last_publish_time_ = ros::Time::now();
  // get joints and allocate message
  for (unsigned i=0; i<n_dof_; i++){
		  //joint_state_.push_back(hw->getHandle(joint_names[i]));
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




}

