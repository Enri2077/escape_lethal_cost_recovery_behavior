/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rotate_recovery/rotate_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace rotate_recovery {
	RotateRecovery::RotateRecovery() : local_costmap_(NULL), initialized_(false), world_model_(NULL) {
	}

	void RotateRecovery::initialize(std::string name, tf2_ros::Buffer *,
									costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap) {
		if (!initialized_) {
			local_costmap_ = local_costmap;

			// get some parameters from the parameter server
			ros::NodeHandle private_nh("~/" + name);

			private_nh.param("escape_cost", escape_cost_, 100);
			private_nh.param("lethal_cost", lethal_cost_, 253);
			private_nh.param("min_gradient_window_size", min_gradient_window_size_, 2);
			private_nh.param("max_gradient_window_size", max_gradient_window_size_, 10);
			private_nh.param("frequency", frequency_, 20.0);
			private_nh.param("min_in_place_vel_theta", min_rotational_vel_, 0.4);
			private_nh.param("yaw_goal_tolerance", tolerance_, 0.1);

			if(min_gradient_window_size_ < 1){
				ROS_WARN("min_gradient_window_size can not be less than 1. Setting to 1.");
			}

			world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

			initialized_ = true;
		} else {
			ROS_ERROR("You should not call initialize twice on this object, doing nothing");
		}
	}

	RotateRecovery::~RotateRecovery() {
		delete world_model_;
	}

	void RotateRecovery::runBehavior() {
		if (!initialized_) {
			ROS_ERROR("This object must be initialized before runBehavior is called");
			return;
		}

		if (local_costmap_ == NULL) {
			ROS_ERROR("The costmap passed to the RecoveryBehavior object cannot be NULL. Doing nothing.");
			return;
		}

		ros::Rate r(frequency_);
		ros::NodeHandle n;
		ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		ros::Publisher escape_direction_pub = n.advertise<geometry_msgs::PoseStamped>("escape_direction", 10);

		geometry_msgs::PoseStamped global_pose;
		local_costmap_->getRobotPose(global_pose);

		costmap_2d::Costmap2D *lcm = local_costmap_->getCostmap();
		double current_angle;
		unsigned int cell_x, cell_y;
		lcm->worldToMap(global_pose.pose.position.x, global_pose.pose.position.y, cell_x, cell_y);
		int current_cost = lcm->getCost(cell_x, cell_y);

		int gradient_window_size = min_gradient_window_size_;

		ROS_WARN("Escape lethal cost recovery behavior started. Current_cost: [%i]", current_cost);

		while (n.ok() && current_cost > escape_cost_) {
			local_costmap_->getRobotPose(global_pose);
			lcm->worldToMap(global_pose.pose.position.x, global_pose.pose.position.y, cell_x, cell_y);
			current_cost = lcm->getCost(cell_x, cell_y);
			int cost_x_1 = lcm->getCost(cell_x - gradient_window_size, cell_y);
			int cost_x_2 = lcm->getCost(cell_x + gradient_window_size, cell_y);
			int cost_y_1 = lcm->getCost(cell_x, cell_y - gradient_window_size);
			int cost_y_2 = lcm->getCost(cell_x, cell_y + gradient_window_size);

			int cost_gradient_x = cost_x_2 - cost_x_1;
			int cost_gradient_y = cost_y_2 - cost_y_1;

			// If the robot is in a minimum or maximum of the costmap, either abort or increase the gradient window
			if (cost_gradient_x == 0 && cost_gradient_y == 0) {
				// If the robot is in a local maximum, we must increasing the gradient window to find the escape direction
				if (current_cost >= lethal_cost_){
					gradient_window_size += 1;
					if (gradient_window_size > max_gradient_window_size_){
						ROS_WARN("Still stuck in local maximum. Aborting recovery behavior.");
						return;
					}
					ROS_WARN("Stuck in local maximum. Increasing gradient window size to [%i].", gradient_window_size);
				}
				// If the robot is in a local minimum, there is nothing we can do
				else{
					ROS_WARN("Stuck in local minimum or saddle point. Aborting recovery behavior.");
					return;
				}

			}else{
				gradient_window_size = min_gradient_window_size_;
			}

			double escape_angle = atan2(-cost_gradient_y, -cost_gradient_x);

			tf2::Quaternion escape_quaternion;
			escape_quaternion.setRPY(0, 0, escape_angle);

			geometry_msgs::PoseStamped escape_direction_pose;
			escape_direction_pose.header.stamp = ros::Time::now();
			escape_direction_pose.header.frame_id = local_costmap_->getGlobalFrameID();
			escape_direction_pose.pose.orientation = tf2::toMsg(escape_quaternion);
			escape_direction_pose.pose.position = global_pose.pose.position;
			escape_direction_pub.publish(escape_direction_pose);

			local_costmap_->getRobotPose(global_pose);
			current_angle = tf2::getYaw(global_pose.pose.orientation);

			double distance_to_escape_angle = angles::shortest_angular_distance(current_angle, escape_angle);

			// compute the velocity that will let us escape the lethal cost
			int sgn = (distance_to_escape_angle > 0.0) ? 1 : ((distance_to_escape_angle < 0.0) ? -1 : 0);
			double angular_vel = min_rotational_vel_ * sgn;

			double linear_vel = 0.0;
			if (std::fabs(distance_to_escape_angle) < tolerance_) {
				linear_vel = 0.01;
				angular_vel = 0.0;
			}

			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = linear_vel;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = angular_vel;

			vel_pub.publish(cmd_vel);

			ROS_DEBUG(
				"\n\n"
				"current_cost: %i \n"
				"costs: x_1: %i   x_2: %i      y_1: %i   y_2: %i \n"
				"current_angle: %f   escape_angle: %f \n"
				"distance_to_escape_angle: %f \n"
				"linear_vel: %f   angular_vel: %f \n",
				current_cost,
				cost_x_1, cost_x_2, cost_y_1, cost_y_2,
				current_angle, escape_angle,
				distance_to_escape_angle,
				linear_vel, angular_vel);

			r.sleep();
		}
	}
};  // namespace rotate_recovery
