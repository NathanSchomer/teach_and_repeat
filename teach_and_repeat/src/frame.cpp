/**
 * @file frame.cpp
 * @brief Implementation of an ORB2-SLAM-style frame class for the teach_and_repeat package.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-23
 */

#include "teach_and_repeat/frame.hpp"
#include <opencv2/imgproc.hpp>  // for cvtColor
#include <opencv2/core.hpp>     // for clone, Mat
#include <opencv2/calib3d.hpp>  // for undistortPoints

Frame::Frame(const cv::Mat& im, size_t id, const double& timeStamp, const cv::Mat& K, const cv::Mat& distCoef, const geometry_msgs::msg::Pose& pose)
    : id_(id), timeStamp_(timeStamp), intrinsicMat_(K.clone()), distCoef_(distCoef.clone()), pose_(pose)
{
    // initialize ORB extractor
    orb_ = cv::ORB::create(nFeatures_, scaleFactor_, nLevels_, edgeThreshold_,
                           firstLevel_, WTA_K_, scoreType_, patchSize_, fastThreshold_);

    // Store ORB scale params
    numScaleLevels_ = orb_->getNLevels();
    fScaleFactor_ = orb_->getScaleFactor();

    // Precompute scale factors
    scaleFactors_.resize(numScaleLevels_);
    invScaleFactors_.resize(numScaleLevels_);
    scaleFactors_[0] = 1.0f;
    invScaleFactors_[0] = 1.0f;
    for (int i = 1; i < numScaleLevels_; ++i) {
        scaleFactors_[i] = scaleFactors_[i - 1] * fScaleFactor_;
        invScaleFactors_[i] = 1.0f / scaleFactors_[i];
    }

    // Extract ORB features
    ExtractORB(im);

    // Undistort keypoints
    UndistortKeyPoints();

    // Compute BoW representation
    ComputeBoW();

    // Odometry pose left uninitialized — user should set externally
}

void Frame::ExtractORB(const cv::Mat &im)
{
    // convert to grayscale if necessary
    cv::Mat gray;
    if (im.type() != CV_8UC1) {
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = im;
    }

    // extract ORB features as keypoints and descriptors
    orb_->detectAndCompute(gray, cv::noArray(), keypoints_, descriptors_);
}

void Frame::UndistortKeyPoints()
{
    if (keypoints_.empty()) return;

    std::vector<cv::Point2f> pts, undistorted_pts;
    for (const auto& kp : keypoints_) {
        pts.push_back(kp.pt);
    }

    cv::undistortPoints(pts, undistorted_pts, intrinsicMat_, distCoef_, cv::noArray(), intrinsicMat_);

    udistortedKeyPoints_.clear();
    for (size_t i = 0; i < undistorted_pts.size(); ++i) {
        cv::KeyPoint kp = keypoints_[i];
        kp.pt = undistorted_pts[i];
        udistortedKeyPoints_.push_back(kp);
    }
}

void Frame::ComputeBoW()
{
    // TODO
    return;
}

bool teach_and_repeat::saveFrame(const Frame& frame, cv::FileStorage& fs)
{
    if (!fs.isOpened())
        return false;

    fs << "Frame" << "{";

    fs << "id" << (int)frame.id_;  // cast to int for YAML friendliness
    fs << "timestamp" << frame.timeStamp_;

    // Save cv::Mat data
    fs << "intrinsicMat" << frame.intrinsicMat_;
    fs << "distCoef" << frame.distCoef_;

    // Save pose (serialize each field manually or create helper function)
    const auto& p = frame.pose_;
    fs << "pose" << "{";
    fs << "position_x" << p.position.x;
    fs << "position_y" << p.position.y;
    fs << "position_z" << p.position.z;
    fs << "orientation_x" << p.orientation.x;
    fs << "orientation_y" << p.orientation.y;
    fs << "orientation_z" << p.orientation.z;
    fs << "orientation_w" << p.orientation.w;
    fs << "}";

    // Save keypoints and descriptors
    fs << "keypoints" << frame.keypoints_;
    fs << "descriptors" << frame.descriptors_;

    fs << "}";

    return true;
}

bool teach_and_repeat::loadFrame(Frame& frame, const cv::FileNode& fn)
{
    if (fn.empty())
        return false;

    int id = 0;
    fn["id"] >> id;
    frame.id_ = static_cast<size_t>(id);

    fn["timestamp"] >> frame.timeStamp_;
    fn["intrinsicMat"] >> frame.intrinsicMat_;
    fn["distCoef"] >> frame.distCoef_;

    auto poseNode = fn["pose"];
    if (!poseNode.empty())
    {
        poseNode["position_x"] >> frame.pose_.position.x;
        poseNode["position_y"] >> frame.pose_.position.y;
        poseNode["position_z"] >> frame.pose_.position.z;

        poseNode["orientation_x"] >> frame.pose_.orientation.x;
        poseNode["orientation_y"] >> frame.pose_.orientation.y;
        poseNode["orientation_z"] >> frame.pose_.orientation.z;
        poseNode["orientation_w"] >> frame.pose_.orientation.w;
    }

    fn["keypoints"] >> frame.keypoints_;
    fn["descriptors"] >> frame.descriptors_;

    return true;
}