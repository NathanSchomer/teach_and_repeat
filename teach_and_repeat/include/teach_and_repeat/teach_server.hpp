/**
 * @file teach_server.hpp
 * @brief Declaration of the TeachServer node for the teach_and_repeat package.
 *
 * This node implements the server side of a teach-and-repeat action in ROS 2.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-22
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <fstream>

#include "teach_and_repeat_interfaces/action/teach.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class TeachServer : public rclcpp::Node
{
private:
    using Teach = teach_and_repeat_interfaces::action::Teach;
    using GoalHandleTeach = rclcpp_action::ServerGoalHandle<Teach>;

    rclcpp_action::Server<Teach>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Teach::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTeach> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleTeach> goal_handle);
    void execute_action(const std::shared_ptr<GoalHandleTeach> goal_handle);

    // subscribe to color & depth images
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // file handle for saving keypoints + odometry
    std::ofstream path_file_;

public:
    TeachServer();
};
