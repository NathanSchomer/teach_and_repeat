/**
 * @file frame.hpp
 * @brief Header for implementation of an ORB2-SLAM-style frame class for the teach_and_repeat package.
 *
 * @author Nathan Schomer <nathans@glidance.io>
 * @date 2025-05-23
 */

#pragma once

#include <opencv2/opencv.hpp>  // Fix typo: was "oencv2"
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include "DBoW2/DBoW2.h"

class Frame 
{
public:
    // Constructor
    Frame(const cv::Mat& im, size_t id_, const double& timeStamp, const cv::Mat& K, const cv::Mat& distCoef, const geometry_msgs::msg::Pose& pose);

    // Member functions
    void ExtractORB(const cv::Mat& im);
    void UndistortKeyPoints();
    void ComputeBoW();

    // Frame ID and timestamp
    size_t id_;
    double timeStamp_;

    // Camera calibration
    cv::Mat intrinsicMat_;  // Intrinsic matrix
    cv::Mat distCoef_;      // Distortion coefficients

    // Pose from odometry
    geometry_msgs::msg::Pose pose_;

    // Image Pyramid Levels & Scaling
    int numScaleLevels_;
    float fScaleFactor_;
    std::vector<float> scaleFactors_;
    std::vector<float> invScaleFactors_;

    // Bag of Words (DBoW2)
    DBoW2::BowVector bowVec_;
    DBoW2::FeatureVector featVec_;

    // ORB features
    cv::Ptr<cv::ORB> orb_;
    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::KeyPoint> udistortedKeyPoints_;

    // ORB Parameters (compile-time constants)
    static constexpr int nFeatures_ = 1000;
    static constexpr float scaleFactor_ = 1.2f;
    static constexpr int nLevels_ = 8;
    static constexpr int edgeThreshold_ = 31;
    static constexpr int firstLevel_ = 0;
    static constexpr int WTA_K_ = 2;
    static constexpr cv::ORB::ScoreType scoreType_ = cv::ORB::HARRIS_SCORE;
    static constexpr int patchSize_ = 31;
    static constexpr int fastThreshold_ = 20;
};

namespace teach_and_repeat
{
    bool saveFrame(const Frame& frame, cv::FileStorage& fs);
    bool loadFrame(Frame& frame, const cv::FileNode& fn);
}
