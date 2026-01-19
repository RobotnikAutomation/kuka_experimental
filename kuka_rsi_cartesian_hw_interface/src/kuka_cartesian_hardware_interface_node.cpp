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

#include <sys/time.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

  ros::init(argc, argv, "kuka_rsi_cartesian_hardware_interface");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  kuka_rsi_cartesian_hw_interface::KukaHardwareInterface kuka_rsi_cartesian_hw_interface;
  kuka_rsi_cartesian_hw_interface.configure();

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;
  //struct timespec  tvalBefore1, tvalAfter1, tvalMid, tvalMid2;
  //controller_manager::ControllerManager controller_manager(&kuka_rsi_hw_interface, nh);

  kuka_rsi_cartesian_hw_interface.start();

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  
  stopwatch_last = stopwatch_now;

  
  // Run as fast as possible
  while (ros::ok())
  //while (!g_quit)
  {
	  //clock_gettime(CLOCK_REALTIME,&tvalMid2);
    // Receive current state from robot
    if (!kuka_rsi_cartesian_hw_interface.read(timestamp, period))
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers
    //controller_manager.update(timestamp, period);
	/*clock_gettime(CLOCK_REALTIME,&tvalMid);
	int microseconds_interval2=((tvalMid.tv_sec - tvalMid2.tv_sec)*1000000000L
          +tvalMid.tv_nsec) - tvalMid2.tv_nsec;
           if(microseconds_interval2/1000>4500)
				//ROS_INFO("time  read  microseconds %d",microseconds_interval2/1000);
    */
    // Send new setpoint to robot
    kuka_rsi_cartesian_hw_interface.write(timestamp, period);
	/*clock_gettime(CLOCK_REALTIME,&tvalAfter1);
	int microseconds_interval=((tvalAfter1.tv_sec - tvalBefore1.tv_sec)*1000000000L
           +tvalAfter1.tv_nsec) - tvalBefore1.tv_nsec;
     if(microseconds_interval2/1000>4500){
				//ROS_INFO("time interval read+write microseconds %d",microseconds_interval/1000);
				//ROS_INFO("time  write microseconds %d",(microseconds_interval-microseconds_interval2)/1000);
			}
     clock_gettime(CLOCK_REALTIME,&tvalBefore1);
    */
	
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;

}
