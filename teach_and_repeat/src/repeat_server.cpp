/**
 * @file repeat_server.cpp
 * @brief Implementation of the RepeatServer node for the teach_and_repeat package.
 *
 * This node implements the server side of a teach-and-repeat action in ROS 2.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-22
 */

#include "teach_and_repeat/repeat_server.hpp"

// using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;

RepeatServer::RepeatServer() : Node("repeat_server")
{
    // setup parameters
    this->declare_parameter("img_color_topic", "/camera_down/color/image_raw");
    this->declare_parameter("img_depth_topic", "/camera_down/depth/image_raw");
    this->declare_parameter("camera_intrinsics", "/camera_down/color/camera_info");
    this->declare_parameter("odom_topic", "/odom");
    // this->declare_parameter("vocabulary_path", "vocab/ORBvoc.yml.gz");
    
    // get parameters
    this->get_parameter("img_color_topic", img_color_topic_);
    this->get_parameter("img_depth_topic", img_depth_topic_);
    this->get_parameter("camera_intrinsics", camera_info_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    // this->get_parameter("vocabulary_path", vocabulary_path_);

    // Load ORB vocabulary and initialize database
    // std::string vocab_path = ament_index_cpp::get_package_share_directory("teach_and_repeat") + "/" + vocabulary_path_.as_string();
    // OrbVocabulary vocab(vocab_path);

    // RCLCPP_INFO(this->get_logger(), "Vocabulary loteachaded with %u words", vocab.size());
    // orb_db_ = OrbDatabase(vocab, false, 0); // false = do not use direct index

    // check that vocab path exists
    // if (!rcpputils::fs::exists(vocab_path)) {
    //     RCLCPP_ERROR(this->get_logger(), "Vocabulary file does not exist: %s", vocab_path.c_str());
    //     throw std::runtime_error("Vocabulary file not found");
    // }
    // RCLCPP_INFO(this->get_logger(), "Loading vocabulary from: %s", vocab_path.c_str());

    // try {
    //     OrbVocabulary vocab(vocab_path);
    //     RCLCPP_INFO(this->get_logger(), "Vocabulary loaded with %u words", vocab.size());
    //     orb_db_ = OrbDatabase(vocab, false, 0);
    // } catch (const std::exception& e) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to load vocabulary: %s", e.what());
    //     throw;
    // }

    // subscribe to camera info 
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_.as_string(), 10,
        std::bind(&RepeatServer::camera_info_callback, this, _1));

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
        std::bind(&RepeatServer::SyncCallback, this, _1, _2));
#else
    img_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_color_topic_.as_string(), 10,
        std::bind(&RepeatServer::img_color_callback, this, _1));
    img_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_depth_topic_.as_string(), 10,
        std::bind(&RepeatServer::img_depth_callback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_.as_string(), 10,
        std::bind(&RepeatServer::odom_callback, this, _1));
#endif

    // Create a publisher for the keypoints image.
    keypoints_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/repeat/keypoints_image", 10);

    // Create the action server.
    action_server_ = rclcpp_action::create_server<Repeat>(
        this,
        "repeat",
        std::bind(&RepeatServer::handle_goal, this, _1, _2),
        std::bind(&RepeatServer::handle_cancel, this, _1),
        std::bind(&RepeatServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse RepeatServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Repeat::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received repeat request with path name: %s", goal->path_name.c_str());
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RepeatServer::handle_cancel(
    const std::shared_ptr<GoalHandleRepeat> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received repeat cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RepeatServer::handle_accepted(const std::shared_ptr<GoalHandleRepeat> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Accepted repeat request");

    load_descriptor_odom_pairs_("teach_path.yml", &saved_descriptors_, &saved_odom_poses_);

    RCLCPP_INFO(this->get_logger(), "Loaded %lu descriptor-odom pairs from file", saved_descriptors_.size());

    auto execute_in_thread  = [this, goal_handle]() { return this->execute_action(goal_handle); };
    std::thread{execute_in_thread}.detach();
}

void RepeatServer::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_intrinsics_ = cv::Mat(3, 3, CV_64F, msg->k.data()).clone();
    distortion_coeffs_ = cv::Mat(1, 5, CV_64F, msg->d.data()).clone();
    RCLCPP_INFO(this->get_logger(), "Received camera intrinsics");
    camera_info_sub_.reset();
}

#ifdef SYNCRONIZED_SUBS
void RepeatServer::SyncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received synchronized messages");
    last_color_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(color_msg);
    last_depth_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(depth_msg);
}
#else
void RepeatServer::img_color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_color_image_ = msg;
}

void RepeatServer::img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_depth_image_ = msg;
}

void RepeatServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_odom_ = msg;
}
#endif

void RepeatServer::load_descriptor_odom_pairs_(
    const std::string& filename, std::vector<cv::Mat>* descriptors,
    std::vector<geometry_msgs::msg::Pose>* odom_poses)
{
    RCLCPP_INFO(this->get_logger(), "Loading descriptor-odom pairs from file: %s", filename.c_str());

    std::vector<std::tuple<cv::Mat, geometry_msgs::msg::Pose>> data;
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open descriptor file: %s", filename.c_str());
        return;
    }

    cv::FileNode frames = fs["pairs"];
    for (const auto& node : frames) {

        cv::Mat curr_descriptors;
        cv::Vec3d position;
        cv::Vec4d orientation;

        node["descriptors"] >> curr_descriptors;
        node["position"] >> position;
        node["orientation"] >> orientation;

        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];

        descriptors->emplace_back(curr_descriptors.clone());
        odom_poses->emplace_back(pose);
    }

    fs.release();
    RCLCPP_INFO(this->get_logger(), "Loaded %lu descriptor-odom pairs", descriptors->size());
    return;
}

void RepeatServer::execute_action(const std::shared_ptr<GoalHandleRepeat> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing repeat request");

    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<Repeat::Feedback>();
    auto result = std::make_shared<Repeat::Result>();
    int64_t frame_count = 0;

    // Initialize ORB feature detector
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat gray_image;
    unsigned int frame_id = -1;

    std::vector<std::pair<cv::Mat, geometry_msgs::msg::Pose>> descriptor_odom_pairs;

    // stack saved descriptors and keep track of indecies
    std::vector<int> descriptor_offsets;
    cv::Mat saved_descriptors_concat;
    int offset = 0;

    for (const auto& desc : saved_descriptors_) {
        descriptor_offsets.push_back(offset);
        saved_descriptors_concat.push_back(desc);
        offset += desc.rows;
    }

    // ROS Loop
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
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
        RCLCPP_INFO(this->get_logger(), "Received color image of size: %dx%d", color_image.cols, color_image.rows);

        rclcpp::Time now = this->now();
        double timestamp = static_cast<double>(now.nanoseconds()) / 1e9;

        // calculate ORB bag of words features and add to database
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        orb->detectAndCompute(gray_image, mask, keypoints, descriptors);

        if (descriptors.empty()) {
            RCLCPP_WARN(this->get_logger(), "No descriptors found in frame %u", frame_id);
            loop_rate.sleep();
            continue;
        }
        RCLCPP_INFO(this->get_logger(), "Frame %u: Detected %lu keypoints", frame_id, keypoints.size());

        // match descriptors with saved descriptors... publish pose of that frame's odometry
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors, saved_descriptors_concat, matches);
        RCLCPP_INFO(this->get_logger(), "Frame %u: Found %lu matches", frame_id, matches.size());

        if (matches.empty()) {
            RCLCPP_WARN(this->get_logger(), "No matches found for frame %u", frame_id);
            loop_rate.sleep();
            continue;
        }

        cv::DMatch best_match;
        float best_distance = std::numeric_limits<float>::max();

        for (const auto& match : matches) {
            if (match.distance < best_distance) {
                best_distance = match.distance;
                best_match = match;
            }
        }

        int trainIdx = best_match.trainIdx;
        int frame_idx = std::upper_bound(descriptor_offsets.begin(), descriptor_offsets.end(), trainIdx) - descriptor_offsets.begin() - 1;
        auto best_odom = saved_odom_poses_[frame_idx];
        RCLCPP_INFO(this->get_logger(), "Best match found at index %lu", frame_idx);

        auto best_match_pose = best_odom;
        RCLCPP_INFO(this->get_logger(), "Best match pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
                    best_match_pose.position.x, best_match_pose.position.y, best_match_pose.position.z,
                    best_match_pose.orientation.x, best_match_pose.orientation.y,
                    best_match_pose.orientation.z, best_match_pose.orientation.w);


#ifdef PUB_KEYPOINTS_IMG
        // TODO: publish image of current image + best match image
#endif
        feedback->n_keyframes = ++frame_count;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Repeat request completed successfully");
        auto result = std::make_shared<Repeat::Result>();
        result->saved_path = true;
        goal_handle->succeed(result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Repeat request failed");
        auto result = std::make_shared<Repeat::Result>();
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
    auto node = std::make_shared<RepeatServer>();

    // Spin the node to process callbacks.
    rclcpp::spin(node);

    // Shutdown ROS cleanly.
    rclcpp::shutdown();

    return 0;
}