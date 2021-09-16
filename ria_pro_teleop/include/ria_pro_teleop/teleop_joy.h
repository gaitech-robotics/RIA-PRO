/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@gaitech.co.kr>

Copyright (c) 2020, Gaitech Korea Co., Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RIA_PRO_TELEOP__TELEOP_JOY_H_
#define RIA_PRO_TELEOP__TELEOP_JOY_H_

#include <string>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

#define OPERATING_MODE 0
#define SPEED_UP 1
#define SPEED_DOWN 2

class RiaProTeleop
{
public:
  RiaProTeleop(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~RiaProTeleop() = default;

  /**
   * \brief Joystick callback
   * \param msg Joystick message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Change the teleoperating mode (differential, mecanum)
   * \param msg Joystick message
   */
  void changeMode(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Increase or decrease the maximum speed according to the button being pressed
   * \param msg Joystick message
   */
  void changeSpeed(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Publish the velocity command according to joystick axis values
   * \param msg Joystick message
   */
  void publishCmdVel(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Watchdog tracks the joystick message
   */
  void watchdog();

private:
  /// ROS parameters
  ros::NodeHandle nh_, nh_priv_;
  ros::Publisher pub_vel_;
  ros::Subscriber sub_joy_;

  /// Time stamp
  std::chrono::steady_clock::time_point last_time_;

  /// Joystick axes
  int axis_linear_x_, axis_linear_y_, axis_angular_;

  /// Joystick buttons
  int speed_up_, speed_down_, deadman_, operating_mode_;

  /// Maximum values
  double max_linear_x_, max_linear_y_, max_angular_, cur_scale_;

  /// Current operating mode
  int cur_mode_;

  /// Button state
  bool pressedButton_[3];

  /// Watchdog related
  bool watchdog_;
};

#endif  // RIA_PRO_TELEOP__TELEOP_JOY_H_