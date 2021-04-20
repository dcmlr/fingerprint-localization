/*
Copyright 2021 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef POSITION_CORRECTION_PUBLISHER_H
#define POSITION_CORRECTION_PUBLISHER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <pole_based_localization/pole_position_correctionConfig.h>

#include <iostream>
#include <fstream>

#include <pole_based_localization/Pole.h>
#include <pole_based_localization/PoleMatchArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

namespace pole_recognition {
class Position_Correction_Publisher : public nodelet::Nodelet{
public:
    Position_Correction_Publisher();
    virtual ~Position_Correction_Publisher();
    virtual void onInit() override;

protected:
    void callbackMatchings(const pole_based_localization::PoleMatchArray& matchings);
    void callbackPosition(const nav_msgs::Odometry& odom);
    void callbackGPSPosition(const nav_msgs::Odometry& odom);
    void callbackPositionDifference(const geometry_msgs::PoseWithCovarianceStamped& odom);
    void callbackReconfigure(pole_based_localization::pole_position_correctionConfig& config, uint32_t level);
    void visualizeMatchings(const pole_based_localization::PoleMatchArray &matchings) const;
    void addYawOffsetToQuaternion(geometry_msgs::Quaternion& quat, double yaw) const;
    void logToCSV(const geometry_msgs::PoseWithCovariance corrected, const geometry_msgs::PoseWithCovariance uncorrected, ros::Time stamp, double posDiffAvg);
    void publishStatusMarker(const geometry_msgs::PoseWithCovariance corrected, const geometry_msgs::PoseWithCovariance uncorrected, double posDiffAvg);

    ros::Subscriber mMatchingSubscriber, mPosSubscriber, mPosDifferenceSubscriber, mGPSSubscriber;
    ros::Publisher mPosCorrectionPublisher, mMatchingMarkerPublisher, mGPSComparePublisher;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<pole_based_localization::pole_position_correctionConfig> > mConfigServer;
    pole_based_localization::pole_position_correctionConfig mConfig;

    std::string mCSVFilename;
    std::ofstream mCSVFile;

    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    geometry_msgs::TransformStamped mMapUTMTransform, mUTMMapTransform;

    geometry_msgs::PoseWithCovarianceStamped mPosCorrection, mLastPosCorrection, mUnalteredPosCorrection;
    geometry_msgs::PoseWithCovariance mLastPose;

    double mAvgPosCorrectionDist = 0.0;
    int mPosCorrectionValCount = 0;

    double mMaxPositionalCorrection = 0.2;
    double mMaxAngularCorrection = 0.00009;
    double mCachedPoseCorrectionDampening = 0.01;

    bool mUseGPS = true, mLogAllWheels = false;
};
}

#endif
