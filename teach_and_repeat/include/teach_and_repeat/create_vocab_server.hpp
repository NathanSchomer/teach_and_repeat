/**
 * @file teach_server.hpp
 * @brief Declaration of the TeachServer node for the teach_and_repeat package.
 *
 * This node implements the server side of a teach-and-repeat action in ROS 2.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-26
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
#include <vector>
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

#include <sensor_msgs/msg/image.hpp>

#include "teach_and_repeat_interfaces/action/create_vocab.hpp"
#include "DBoW2/DBoW2.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std;
using namespace std::chrono_literals;
using namespace DBoW2;
using std::placeholders::_1;
using std::placeholders::_2;

class CreateVocabServer : public rclcpp::Node
{
private:
    // Paramters
    rclcpp::Parameter img_color_topic_;

    bool train_vocab_;
    int64_t frame_count_;

    vector<vector<cv::Mat>> features_;
    cv::Ptr<cv::ORB> orb_;

    // last received color image
    sensor_msgs::msg::Image::SharedPtr last_color_image_;

    // Subscribers
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> img_color_sub_;
    void img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Action Stuff
    using CreateVocab = teach_and_repeat_interfaces::action::CreateVocab;
    using GoalHandleCreateVocab = rclcpp_action::ServerGoalHandle<CreateVocab>;

    rclcpp_action::Server<CreateVocab>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CreateVocab::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCreateVocab> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleCreateVocab> goal_handle);
    void execute_action(const std::shared_ptr<GoalHandleCreateVocab> goal_handle);

    bool save_vocabulary(const std::string &path);

    void update_features();

    void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);

public:
    CreateVocabServer();
};