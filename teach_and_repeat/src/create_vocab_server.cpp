/**
 * @file teach_server.cpp
 * @brief Implementation of the TeachServer node for the teach_and_repeat package.
 *
 * This node implements the server side of a teach-and-repeat action in ROS 2.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-26
 */

#include "teach_and_repeat/create_vocab_server.hpp"

CreateVocabServer::CreateVocabServer() : Node("create_vocab_server")
{
    train_vocab_ = false;
    frame_count_ = 0;

    orb_ = cv::ORB::create();

    // get image topic as parameter
    this->declare_parameter("img_color_topic", "/camera_down/color/image_raw");
    this->get_parameter("img_color_topic", img_color_topic_);

    // subscribe to image topic 
    img_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_color_topic_.as_string(), 10,
        std::bind(&CreateVocabServer::img_color_callback, this, _1));

    // Create the action server.
    action_server_ = rclcpp_action::create_server<CreateVocab>(
        this,
        "create_vocab",
        std::bind(&CreateVocabServer::handle_goal, this, _1, _2),
        std::bind(&CreateVocabServer::handle_cancel, this, _1),
        std::bind(&CreateVocabServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse CreateVocabServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CreateVocab::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received teacher request with path name: %s", goal->path_name.c_str());
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CreateVocabServer::handle_cancel(
    const std::shared_ptr<GoalHandleCreateVocab> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received teach cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CreateVocabServer::handle_accepted(const std::shared_ptr<GoalHandleCreateVocab> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Accepted create vocab request");

    auto execute_in_thread  = [this, goal_handle]() { return this->execute_action(goal_handle); };
    std::thread{execute_in_thread}.detach();
}

void CreateVocabServer::img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_color_image_ = msg;
    
}

bool CreateVocabServer::save_vocabulary(const std::string &path)
{
    // branching factor and depth levels 
    // TODO: parameterize these
    const int k = 9;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType scoring = L1_NORM;

    OrbVocabulary voc(k, L, weight, scoring);

    RCLCPP_INFO(this->get_logger(), "Creating vocabulary...");
    voc.create(features_);
    RCLCPP_INFO(this->get_logger(), "... done!");

    // save the vocabulary to disk
    RCLCPP_INFO(this->get_logger(), "Saving vocabulary to %s...", path.c_str());
    voc.save(path);
    return true;
    // TODO: add check to see if it saved
}

void CreateVocabServer::changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  // TODO should probably pre-allocate. This seems like a terrible idea
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

void CreateVocabServer::update_features()
{
    RCLCPP_INFO(this->get_logger(), "Received image for vocabulary training, frame count: %ld", frame_count_);
    frame_count_++;

    cv_bridge::CvImagePtr cv_ptr;

    // convert ROS image to opencv
    try {
        cv_ptr = cv_bridge::toCvCopy(last_color_image_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // convert image to grayscale
    if (cv_ptr->image.channels() == 3) {
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
    }
    
    cv::Mat image = cv_ptr->image;
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_->detectAndCompute(image, mask, keypoints, descriptors);
    features_.push_back(vector<cv::Mat>());
    changeStructure(descriptors, features_.back());  
}

void CreateVocabServer::execute_action(const std::shared_ptr<GoalHandleCreateVocab> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing teach request");

    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<CreateVocab::Feedback>();
    auto result = std::make_shared<CreateVocab::Result>();

    // ROS Loop
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            save_vocabulary("./vocab.yml.gz");  // return True if saved successfully, false otherwise
            goal_handle->canceled(result);
            return;
        }

        update_features();

        feedback->n_keyframes = frame_count_;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Current number of keyframes: %ld", frame_count_);
        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Teach request completed successfully");
        auto result = std::make_shared<CreateVocab::Result>();
        result->saved_path = true;
        goal_handle->succeed(result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Teach request failed");
        auto result = std::make_shared<CreateVocab::Result>();
        result->saved_path = false;
        goal_handle->abort(result);
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreateVocabServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}