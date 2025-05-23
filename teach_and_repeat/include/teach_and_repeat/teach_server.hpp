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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <chrono>
#include <memory>
#include <functional>
#include <rmw/qos_profiles.h>

#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "teach_and_repeat_interfaces/action/teach.hpp"
#include "teach_and_repeat/frame.hpp"
#include "DBoW2/DBoW2.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define SYNCRONIZED_SUBS
#define PUB_KEYPOINTS_IMG

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

    // file handle for saving keypoints + odometry
    std::ofstream path_file_;

    // last received color image
    sensor_msgs::msg::Image::SharedPtr last_color_image_;
    
    // last received depth image
    sensor_msgs::msg::Image::SharedPtr last_depth_image_;
    
    // last received odometry
    nav_msgs::msg::Odometry::SharedPtr last_odom_;

    // define parameters 
    rclcpp::Parameter img_color_topic_;
    rclcpp::Parameter img_depth_topic_;
    rclcpp::Parameter camera_info_topic_;
    rclcpp::Parameter odom_topic_;

    std::vector<std::shared_ptr<Frame>> frames_cache_;
    cv::Mat camera_intrinsics_;
    cv::Mat distortion_coeffs_;

#ifdef SYNCRONIZED_SUBS
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img_color_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img_depth_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;

    void SyncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
#else
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> img_color_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> img_depth_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> camera_info_sub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

    void img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
#endif 

#ifdef PUB_KEYPOINTS_IMG
    // keypoints_img_pub
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> keypoints_img_pub_;
#endif

public:
    TeachServer();
};
