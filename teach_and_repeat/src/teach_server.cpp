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

// using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;

TeachServer::TeachServer() : Node("teach_server")
{
    // setup parameters
    this->declare_parameter("img_color_topic", "/camera_down/color/image_raw");
    this->declare_parameter("img_depth_topic", "/camera_down/depth/image_raw");
    this->declare_parameter("camera_intrinsics", "/camera_down/color/camera_info");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("vocabulary_path", "vocab/output.yml.gz");
    
    // get parameters
    this->get_parameter("img_color_topic", img_color_topic_);
    this->get_parameter("img_depth_topic", img_depth_topic_);
    this->get_parameter("camera_intrinsics", camera_info_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("vocabulary_path", vocabulary_path_);

    // Load ORB vocabulary and initialize database
    std::string vocab_path = ament_index_cpp::get_package_share_directory("teach_and_repeat") + "/" + vocabulary_path_.as_string();
    RCLCPP_INFO(this->get_logger(), "Loading vocabulary from: %s", vocab_path.c_str());
    OrbVocabulary vocab(vocab_path);

    RCLCPP_INFO(this->get_logger(), "Vocabulary loaded with %u words", vocab.size());
    orb_db_ = OrbDatabase(vocab, false, 0); // false = do not use direct index

    // subscribe to camera info 
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_.as_string(), 10,
        std::bind(&TeachServer::camera_info_callback, this, _1));

#ifdef SYNCRONIZED_SUBS
    // TODO: broken and idk why :(
    img_color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, img_color_topic_.as_string(), rmw_qos_profile_sensor_data);
    img_depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, img_depth_topic_.as_string(), rmw_qos_profile_sensor_data);

    uint32_t queue_size = 10000;
    sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image>(queue_size),
            *img_color_sub_, *img_depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(2));

    // sync_->setAgePenalty(1000);
    sync_->registerCallback(
        std::bind(&TeachServer::SyncCallback, this, _1, _2));
#else
    img_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_color_topic_.as_string(), 10,
        std::bind(&TeachServer::img_color_callback, this, _1));
    img_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_depth_topic_.as_string(), 10,
        std::bind(&TeachServer::img_depth_callback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_.as_string(), 10,
        std::bind(&TeachServer::odom_callback, this, _1));
#endif

    // Create a publisher for the keypoints image.
    keypoints_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/teach/keypoints_image", 10);

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

void TeachServer::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_intrinsics_ = cv::Mat(3, 3, CV_64F, msg->k.data()).clone();
    distortion_coeffs_ = cv::Mat(1, 5, CV_64F, msg->d.data()).clone();
    RCLCPP_INFO(this->get_logger(), "Received camera intrinsics");
    camera_info_sub_.reset();
}

#ifdef SYNCRONIZED_SUBS
void TeachServer::SyncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received synchronized messages");
    last_color_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(color_msg);
    last_depth_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(depth_msg);
}
#else
void TeachServer::img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_color_image_ = msg;
}

void TeachServer::img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_depth_image_ = msg;
}

void TeachServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_odom_ = msg;
}
#endif

void TeachServer::save_frames_cache_()
{
    RCLCPP_INFO(this->get_logger(), "Saving DBOW2 database to disk.");
    orb_db_.save("dbow2_database.yml.gz");
    // TODO: save database

    // if (frames_cache_.empty()) {
    //     RCLCPP_WARN(this->get_logger(), "No frames to save");
    //     return;
    // }

    // // todo add parameter for save path
    // cv::FileStorage fs("frames.yml", cv::FileStorage::WRITE);
    // fs << "frames" << "[";
    // for (const auto& frame : frames_cache_) {
    //     RCLCPP_INFO(this->get_logger(), "Saving frame %zu", frame->id_);
    //     fs << "{";
    //     teach_and_repeat::saveFrame(*frame, fs);
    //     fs << "}";
    // }
    // fs << "]";
    // fs.release();
}

void TeachServer::execute_action(const std::shared_ptr<GoalHandleTeach> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing teach request");

    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<Teach::Feedback>();
    auto result = std::make_shared<Teach::Result>();
    int64_t frame_count = 0;

    // Initialize ORB feature detector
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat gray_image;
    unsigned int frame_id = -1;

    // ROS Loop
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            save_frames_cache_();
            goal_handle->canceled(result);
            return;
        }

        if (last_color_image_ == nullptr || last_depth_image_ == nullptr || last_odom_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Waiting for images and odometry");
            loop_rate.sleep();
            continue;
        }

        // Convert ROS image message to cv::Mat (BGR8)
        cv::Mat color_image = cv_bridge::toCvCopy(last_color_image_, sensor_msgs::image_encodings::BGR8)->image;

        rclcpp::Time now = this->now();
        double timestamp = static_cast<double>(now.nanoseconds()) / 1e9;

        // calculate ORB bag of words features and add to database
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        orb->detectAndCompute(gray_image, mask, keypoints, descriptors);
        frame_id = orb_db_.add(descriptors);

        // size_t frame_id = static_cast<size_t>(timestamp * 1e6); // convert to microseconds

        // // Create a Frame with the image, timestamp, and camera calibration
        // auto frame = std::make_shared<Frame>(
        //     color_image,
        //     frame_id,
        //     timestamp,
        //     camera_intrinsics_,
        //     distortion_coeffs_,
        //     last_odom_->pose.pose
        // );

        RCLCPP_INFO(this->get_logger(), "Frame %u received at time %.2f", frame_id, timestamp);

        // // Cache frame and odometry
        // frames_cache_.push_back(frame);

#ifdef PUB_KEYPOINTS_IMG
        cv::Mat img_keypoints;
        cv::drawKeypoints(color_image, keypoints, img_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
        auto keypoints_img_msg = cv_bridge::CvImage(
            last_color_image_->header, sensor_msgs::image_encodings::BGR8, img_keypoints).toImageMsg();
        keypoints_img_msg->header.stamp = last_color_image_->header.stamp;
        keypoints_img_msg->header.frame_id = last_color_image_->header.frame_id;
        keypoints_img_pub_->publish(*keypoints_img_msg);
#endif

        // TODO: use camera cal to get 3d points in world frame

        feedback->n_keyframes = ++frame_count;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
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