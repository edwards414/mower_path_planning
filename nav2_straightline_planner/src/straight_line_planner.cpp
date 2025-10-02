/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
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
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

 #include <cmath>
 #include <string>
 #include <memory>
 #include "nav2_util/node_utils.hpp"
 
 #include "nav2_straightline_planner/straight_line_planner.hpp"
 
 namespace nav2_straightline_planner
 {
 
 void StraightLine::configure(
   const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
 {
   node_ = parent.lock();
   if (!node_) {
     RCLCPP_ERROR(rclcpp::get_logger("StraightLine"), "Failed to lock parent node!");
     return;
   }
   RCLCPP_INFO(node_->get_logger(), "StraightLine::configure called");

   name_ = name;
   tf_ = tf;
   if (!costmap_ros) {
     RCLCPP_ERROR(node_->get_logger(), "Costmap ROS pointer is null!");
     return;
   }
   costmap_ = costmap_ros->getCostmap();
   global_frame_ = "map";
 
   // Parameter initialization
   nav2_util::declare_parameter_if_not_declared(
     node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
   node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  
   // 验证参数值
   if (interpolation_resolution_ <= 0.0) {
     RCLCPP_WARN(node_->get_logger(), 
       "Invalid interpolation_resolution: %f, using default 0.1", interpolation_resolution_);
     interpolation_resolution_ = 0.1;
   }
   RCLCPP_INFO(node_->get_logger(), "StraightLine::configure called");
 }
 
 void StraightLine::cleanup()
 {
   RCLCPP_INFO(
     node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
     name_.c_str());
 }
 
 void StraightLine::activate()
 {
   RCLCPP_INFO(
     node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
     name_.c_str());
 }
 
 void StraightLine::deactivate()
 {
   RCLCPP_INFO(
     node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
     name_.c_str());
 }
 
 nav_msgs::msg::Path StraightLine::createPlan(
   const geometry_msgs::msg::PoseStamped & start,
   const geometry_msgs::msg::PoseStamped & goal,
   std::function<bool()> /*cancel_checker*/)
 {
   nav_msgs::msg::Path global_path;
 

   // Checking if the goal and start state is in the global frame
   if (start.header.frame_id != global_frame_) {
     RCLCPP_ERROR(
       node_->get_logger(), "Planner will only accept start position from %s frame",
       global_frame_.c_str());
     return global_path;
   }
 
   if (goal.header.frame_id != global_frame_) {
     RCLCPP_ERROR(
       node_->get_logger(), "Planner will only accept goal position from %s frame",
       global_frame_.c_str());
     return global_path;
   }
 
   global_path.poses.clear();
   global_path.header.stamp = node_->now();
   global_path.header.frame_id = global_frame_;
  
   // 计算距离
   double distance = std::hypot(
     goal.pose.position.x - start.pose.position.x,
     goal.pose.position.y - start.pose.position.y);
  
   // 检查距离是否为零或插值分辨率是否有效
   if (distance < 1e-6 || interpolation_resolution_ <= 0.0) {
     RCLCPP_WARN(node_->get_logger(), 
       "Distance too small (%f) or invalid interpolation resolution (%f), returning goal only", 
       distance, interpolation_resolution_);
     geometry_msgs::msg::PoseStamped goal_pose = goal;
     goal_pose.header.stamp = node_->now();
     goal_pose.header.frame_id = global_frame_;
     global_path.poses.push_back(goal_pose);
     return global_path;
   }
  
   // calculating the number of loops for current value of interpolation_resolution_
   int total_number_of_loop = static_cast<int>(distance / interpolation_resolution_);
  
   // 确保至少有一个循环
   if (total_number_of_loop <= 0) {
     total_number_of_loop = 1;
   }
  
    double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;
      // 根據起點和終點計算正確的朝向
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double yaw = std::atan2(dy, dx);



    //   double cos_yaw = std::cos(yaw);
    //   double sin_yaw = std::sin(yaw);
    // q_.setRPY(0, 0, yaw);
   for (int i = 0; i < total_number_of_loop; ++i) {
     geometry_msgs::msg::PoseStamped pose;
     pose.pose.position.x = start.pose.position.x + x_increment * i;
     pose.pose.position.y = start.pose.position.y + y_increment * i;
     pose.pose.position.z = 0.0;
     pose.pose.orientation.x = 0.0;
     pose.pose.orientation.y = 0.0;
     pose.pose.orientation.z = 0.0;
     pose.pose.orientation.w = 1.0;
     pose.header.stamp = node_->now();
     pose.header.frame_id = global_frame_;
     global_path.poses.push_back(pose);
   }
 
   geometry_msgs::msg::PoseStamped goal_pose = goal;
   goal_pose.header.stamp = node_->now();
   goal_pose.header.frame_id = global_frame_;
   global_path.poses.push_back(goal_pose);
 
   return global_path;
 }
 
 }  // namespace nav2_straightline_planner
 
 #include "pluginlib/class_list_macros.hpp"
 PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)