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

#include "ria_pro_teleop/teleop_joy.h"

RiaProTeleop::RiaProTeleop(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , axis_linear_x_(1)
  , axis_linear_y_(0)
  , axis_angular_(3)
  , speed_up_(2)
  , speed_down_(0)
  , deadman_(4)
  , operating_mode_(6)
  , max_linear_x_(1.5)
  , max_linear_y_(1.5)
  , max_angular_(2.0)
  , cur_mode_(1)
  , cur_scale_(0.4)
  , watchdog_(false)
{
  nh_priv_.getParam("axis_linear_x", axis_linear_x_);
  nh_priv_.getParam("axis_linear_y", axis_linear_y_);
  nh_priv_.getParam("axis_angular", axis_angular_);
  nh_priv_.getParam("speed_up", speed_up_);
  nh_priv_.getParam("speed_down", speed_down_);
  nh_priv_.getParam("deadman", deadman_);
  nh_priv_.getParam("operating_mode", operating_mode_);
  nh_priv_.getParam("max_linear_x", max_linear_x_);
  nh_priv_.getParam("max_linear_y", max_linear_y_);
  nh_priv_.getParam("max_angular", max_angular_);

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  sub_joy_ = nh_.subscribe("/joy", 10, &RiaProTeleop::joyCallback, this);

  for (int i = 0; i < 3; i++)
    pressedButton_[i] = false;

  last_time_ = std::chrono::steady_clock::now();
}

void RiaProTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[deadman_])
  {
    last_time_ = std::chrono::steady_clock::now();

    // changeMode(msg);
    changeSpeed(msg);
    publishCmdVel(msg);
  }
}

void RiaProTeleop::changeMode(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[operating_mode_])
  {
    if (!pressedButton_[OPERATING_MODE])
    {
      pressedButton_[OPERATING_MODE] = true;
      if (cur_mode_ == 1)
      {
        cur_mode_ = 2;
        ROS_INFO("Operating mode changed to mecanum drive mode");
      }
      else if (cur_mode_ == 2)
      {
        cur_mode_ = 1;
        ROS_INFO("Operating mode changed to differential drive mode");
      }
    }
  }
  else
    pressedButton_[OPERATING_MODE] = false;
}

void RiaProTeleop::changeSpeed(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[speed_up_])
  {
    if (!pressedButton_[SPEED_UP])
    {
      pressedButton_[SPEED_UP] = true;
      cur_scale_ += 0.2;
      if (cur_scale_ >= 1.0)
        cur_scale_ = 1.0;
    }
  }
  else if (msg->buttons[speed_down_])
  {
    if (!pressedButton_[SPEED_DOWN])
    {
      pressedButton_[SPEED_DOWN] = true;
      cur_scale_ -= 0.2;
      if (cur_scale_ <= 0.0)
        cur_scale_ = 0.0;
    }
  }
  else
  {
    pressedButton_[SPEED_UP] = false;
    pressedButton_[SPEED_DOWN] = false;
  }
}

void RiaProTeleop::publishCmdVel(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  vel.linear.x = cur_scale_ * max_linear_x_ * msg->axes[axis_linear_x_];
  if (cur_mode_ == 2)
    vel.linear.y = cur_scale_ * max_linear_y_ * msg->axes[axis_linear_y_];
  vel.angular.z = cur_scale_ * max_angular_ * msg->axes[axis_angular_];

  pub_vel_.publish(vel);
}

void RiaProTeleop::watchdog()
{
  std::chrono::steady_clock::time_point cur_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = cur_time - last_time_;
  const double elapsed_time = elapsed.count();
  last_time_ = cur_time;

  if (elapsed_time < 0.5)
    watchdog_ = false;
  else if (elapsed_time > 0.5 && watchdog_ == false)
  {
    watchdog_ = true;
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    pub_vel_.publish(vel);
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate rate(30.0);

  RiaProTeleop teleop(nh, nh_priv);

  while (ros::ok())
  {
    ros::spinOnce();
    teleop.watchdog();
    rate.sleep();
  }

  return EXIT_FAILURE;
}