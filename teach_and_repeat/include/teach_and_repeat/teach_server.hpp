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

public:
    TeachServer();
};
