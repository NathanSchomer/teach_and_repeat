/**
 * @file teach_server.cpp
 * @brief Implementation of the TeachServer node for the teach_and_repeat package.
 *
 * This node implements the server side of a teach-and-repeat action in ROS 2.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-22
 */

#include "teach_and_repeat/teach_server.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

TeachServer::TeachServer() : Node("teach_server")
{
    // Create the action server.
    action_server_ = rclcpp_action::create_server<Teach>(
        this,
        "teach",
        std::bind(&TeachServer::handle_goal, this, _1, _2),
        std::bind(&TeachServer::handle_cancel, this, _1),
        std::bind(&TeachServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse TeachServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Teach::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received teacher request with path name: %s", goal->path_name.c_str());
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TeachServer::handle_cancel(
    const std::shared_ptr<GoalHandleTeach> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received teach cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TeachServer::handle_accepted(const std::shared_ptr<GoalHandleTeach> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Accepted teach request");

    auto execute_in_thread  = [this, goal_handle]() { return this->execute_action(goal_handle); };
    std::thread{execute_in_thread}.detach();
}

void TeachServer::execute_action(const std::shared_ptr<GoalHandleTeach> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing teach request");

    // Simulate some work being done.
    rclcpp::Rate rate(1);
    for (int i = 0; i < 5; ++i) {
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Teach request canceled");
            auto result = std::make_shared<Teach::Result>();
            result->saved_path = false;
            goal_handle->canceled(result);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Teaching step %d", i);
        rate.sleep();
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Teach request completed successfully");
        auto result = std::make_shared<Teach::Result>();
        result->saved_path = true;
        goal_handle->succeed(result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Teach request failed");
        auto result = std::make_shared<Teach::Result>();
        result->saved_path = false;
        goal_handle->abort(result);
        return;
    }

}

int main(int argc, char **argv)
{
    // Initialize ROS.
    rclcpp::init(argc, argv);

    // Create the server node.
    auto node = std::make_shared<TeachServer>();

    // Spin the node to process callbacks.
    rclcpp::spin(node);

    // Shutdown ROS cleanly.
    rclcpp::shutdown();

    return 0;
}