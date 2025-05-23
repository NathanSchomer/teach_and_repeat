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
    // if (!orbExtractor_) return; // check extractor is set

    // bowVec_.clear();
    // featVec_.clear();
    // orbExtractor_->ComputeBoW(descriptors_, bowVec_, featVec_);
    return;
}
