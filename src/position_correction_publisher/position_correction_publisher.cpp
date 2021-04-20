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

#include "position_correction_publisher/position_correction_publisher.h"

#include <sstream>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <pole_based_localization/PoleMatch.h>
#include <pole_based_localization/PoleMatchArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace pole_recognition {

using namespace pole_based_localization;

Position_Correction_Publisher::Position_Correction_Publisher() : mTfListener(mTfBuffer)
{}


Position_Correction_Publisher::~Position_Correction_Publisher()
{
    mCSVFile.close();
}


void Position_Correction_Publisher::onInit()
{
    ros::NodeHandle nh = getNodeHandle();
    dynamic_reconfigure::Server<pole_based_localization::pole_position_correctionConfig>::CallbackType config_callback;

    nh.getParam("/pole_recognition/csv_file", mCSVFilename);
    nh.getParam("/pole_recognition/log_all_wheels", mLogAllWheels);
    mCSVFile.open(mCSVFilename);
    mCSVFile << "timestamp, average positional offset, corrected_x, corrected_y, x, y";
    if(mLogAllWheels)
        mCSVFile << ", frontR_x, frontR_y, backR_x, backR_y, backL_x, backL_y, frontL_x, frontL_y";
    mCSVFile << std::endl;

    mPosCorrectionPublisher = nh.advertise<nav_msgs::Odometry>("/localization/pole_odom",1);
    mMatchingMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/pole_recognition/FeatureMatchingsMarkerArray",1);

    mConfigServer =
        boost::make_shared<dynamic_reconfigure::Server<pole_based_localization::pole_position_correctionConfig> >(getPrivateNodeHandle());
    config_callback = boost::bind(&Position_Correction_Publisher::callbackReconfigure, this, _1, _2);
    mConfigServer->setCallback(config_callback);

    mMatchingSubscriber = nh.subscribe(nh.resolveName("/pole_recognition/PoleMatchings"), 1, &Position_Correction_Publisher::callbackMatchings, this);
    mPosSubscriber = nh.subscribe(nh.resolveName("/localization/odometry/filtered_map"), 1, &Position_Correction_Publisher::callbackPosition, this);
    mPosDifferenceSubscriber = nh.subscribe(nh.resolveName("/pole_recognition/PositionDifference"),1, &Position_Correction_Publisher::callbackPositionDifference, this);
    mGPSSubscriber = nh.subscribe(nh.resolveName("odometry/gps"),1, &Position_Correction_Publisher::callbackGPSPosition, this);
    try{
        mTfBuffer.canTransform("utm", "map", ros::Time::now(), ros::Duration(10.0));
        mMapUTMTransform = mTfBuffer.lookupTransform("utm", "map", ros::Time::now(), ros::Duration(1.0));
        mUTMMapTransform = mTfBuffer.lookupTransform("map", "utm", ros::Time::now(), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Pole based localization problem (missing map-utm transform): %s",ex.what());
    }
}


void Position_Correction_Publisher::callbackReconfigure(pole_based_localization::pole_position_correctionConfig &config, uint32_t level)
{
    mConfig = config;
    mUseGPS = mConfig.use_gps;
    mMaxPositionalCorrection = mConfig.max_positional_correction;
    mMaxAngularCorrection = mConfig.max_angular_correction;
    mCachedPoseCorrectionDampening = mConfig.pose_correction_dampening/100.0;
}


void Position_Correction_Publisher::callbackPosition(const nav_msgs::Odometry &odom)
{
    nav_msgs::Odometry correctedOdom = odom;
    auto lastSpeed = odom.twist.twist.linear;
    double speed = std::sqrt(std::pow(lastSpeed.x, 2.0) + std::pow(lastSpeed.y, 2.0));
    double x_correction = std::max(std::min(mPosCorrection.pose.pose.position.x, std::min(mMaxPositionalCorrection, speed)), std::max(-mMaxPositionalCorrection, -speed));
    double y_correction = std::max(std::min(mPosCorrection.pose.pose.position.y, std::min(mMaxPositionalCorrection, speed)), std::max(-mMaxPositionalCorrection, -speed));

    correctedOdom.pose.pose.position.x -= x_correction;
    correctedOdom.pose.pose.position.y -= y_correction;
    double eulerYawCorrection = std::max(std::min(mPosCorrection.pose.pose.orientation.z, mMaxAngularCorrection), -mMaxAngularCorrection);
    addYawOffsetToQuaternion(correctedOdom.pose.pose.orientation, -eulerYawCorrection);

    if(std::abs(mPosCorrection.pose.pose.position.x) > 0.0 && speed > 0.01){
        ++mPosCorrectionValCount;
        mAvgPosCorrectionDist += (std::sqrt(std::pow(x_correction,2) + std::pow(y_correction,2)) - mAvgPosCorrectionDist) / mPosCorrectionValCount;
    }

    correctedOdom.pose.covariance[0] += mPosCorrection.pose.covariance[0];
    correctedOdom.pose.covariance[7] += mPosCorrection.pose.covariance[7];
    correctedOdom.pose.covariance[35] += mPosCorrection.pose.covariance[35];

    if(!mUseGPS)
        mPosCorrectionPublisher.publish(correctedOdom);

    mLastPose = correctedOdom.pose;
    geometry_msgs::PoseWithCovariance correctedPose;
    correctedPose = odom.pose;
    correctedPose.pose.position.x -= mUnalteredPosCorrection.pose.pose.position.x;
    correctedPose.pose.position.y -= mUnalteredPosCorrection.pose.pose.position.y;    

    // Update position correction
    mPosCorrection.pose.pose.position.x -= mPosCorrection.pose.pose.position.x * mCachedPoseCorrectionDampening;
    mPosCorrection.pose.pose.position.y -= mPosCorrection.pose.pose.position.y * mCachedPoseCorrectionDampening;
    mPosCorrection.pose.pose.orientation.z -= mPosCorrection.pose.pose.orientation.z * mCachedPoseCorrectionDampening;

    // Log to csv
    if(!mCSVFilename.empty())
        logToCSV(correctedPose, odom.pose, odom.header.stamp-ros::Duration(0.02), mAvgPosCorrectionDist);
    publishStatusMarker(correctedPose, odom.pose, mAvgPosCorrectionDist);
}


void Position_Correction_Publisher::logToCSV(const geometry_msgs::PoseWithCovariance corrected, const geometry_msgs::PoseWithCovariance uncorrected, ros::Time stamp, double posDiffAvg)
{
    geometry_msgs::Point correctedPos, uncorrectedPos, frontR, frontL, backR, backL;
    tf2::doTransform(corrected.pose.position, correctedPos, mMapUTMTransform);
    tf2::doTransform(uncorrected.pose.position, uncorrectedPos, mMapUTMTransform);

    if(mLogAllWheels){
        try{
            auto frontRTransform = mTfBuffer.lookupTransform("utm", "wheel_fr", stamp, ros::Duration(0));
            tf2::doTransform(frontR, frontR, frontRTransform);
            auto backRTransform = mTfBuffer.lookupTransform("utm", "wheel_br", stamp, ros::Duration(0));
            tf2::doTransform(backR, backR, backRTransform);
            auto backLTransform = mTfBuffer.lookupTransform("utm", "wheel_bl", stamp, ros::Duration(0));
            tf2::doTransform(backL, backL, backLTransform);
            auto frontLTransform = mTfBuffer.lookupTransform("utm", "wheel_fl", stamp, ros::Duration(0));
            tf2::doTransform(frontL, frontL, frontLTransform);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Wheel transform problem: %s",ex.what());
            return;
        }
    }

    mCSVFile << std::fixed << ros::Time::now().toSec() << ',' << std::fixed << posDiffAvg << ',' << std::fixed << correctedPos.x << ',' << std::fixed << correctedPos.y << ',' << std::fixed << uncorrectedPos.x << ',' << std::fixed << uncorrectedPos.y;
    if(mLogAllWheels){
        mCSVFile << ',' << std::fixed << frontR.x << ',' << std::fixed << frontR.y << ',' << std::fixed << backR.x << ',' << std::fixed << backR.y << ',' << std::fixed << backL.x << ',' << std::fixed << backL.y << ',';
        mCSVFile << std::fixed << frontL.x << ',' << std::fixed << frontL.y;
    }
    mCSVFile << std::endl;
}


void Position_Correction_Publisher::publishStatusMarker(const geometry_msgs::PoseWithCovariance corrected, const geometry_msgs::PoseWithCovariance uncorrected, double posDiffAvg)
{
    // For the RVIZ text marker visualization
    using namespace visualization_msgs;

    // Display as text marker in RVIZ
    // if our offset is 0.0 when can safely assume that we don't have any valid matchings
    bool validOffset = corrected.pose.position.x-uncorrected.pose.position.x!=0.0;
    std::ostringstream markerText;    
    MarkerArray textMarkers;
    Marker textMarker;

    markerText << "Feature loc.: ";
    if(mUseGPS){
        markerText << "GPS\n";
        textMarker.color.r = 1.0;
        textMarker.color.g = 1.0;
        textMarker.color.b = 0.3;
    }
    else{
        markerText << (validOffset ? "ACTIVE\n" : "INACTIVE\n");
        textMarker.color.r = validOffset ? 0.3 : 1.0;
        textMarker.color.g = validOffset ? 1.0 : 0.0;
        textMarker.color.b = validOffset ? 0.3 : 0.0;
    }

    markerText << "Avg. pos. offset: " << std::setprecision(2) << posDiffAvg << "m";


    textMarker.header.frame_id = "base_link";
    textMarker.header.stamp = ros::Time::now();
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.scale.z = 1.0;
    textMarker.ns = "debug_text";
    textMarker.id = 1337;
    textMarker.lifetime = ros::Duration(0.1);
    textMarker.color.a = 0.95;
    textMarker.pose.position.x = -5.0;
    textMarker.pose.position.y = -10.0;
    textMarker.pose.position.z = 0.0;
    textMarker.text = markerText.str();
    textMarkers.markers.push_back(textMarker);
    mMatchingMarkerPublisher.publish(textMarkers);
}


void Position_Correction_Publisher::callbackGPSPosition(const nav_msgs::Odometry &odom)
{
    if(mUseGPS){
        auto odomCopy = odom;
        mPosCorrectionPublisher.publish(odomCopy);
    }
}


void Position_Correction_Publisher::callbackPositionDifference(const geometry_msgs::PoseWithCovarianceStamped &odom)
{
    if(odom.pose.pose.position.x != 0.0 && odom.pose.pose.position.y != 0.0){
        mPosCorrection.header.stamp = ros::Time::now();
        tf2::Quaternion tfquat;
        tf2::convert(odom.pose.pose.orientation, tfquat);
        double yaw = -tf2::getYaw(tfquat);
        mPosCorrection.pose.pose.position.x = odom.pose.pose.position.x;
        mPosCorrection.pose.pose.position.y = odom.pose.pose.position.y;
        mPosCorrection.pose.pose.orientation.z = yaw;

        mPosCorrection.pose.covariance = odom.pose.covariance;

        mLastPosCorrection = mPosCorrection;

        mUseGPS = false; // Switch to feature localization
    }
    else if(!mUseGPS)
            getNodeHandle().getParam("/pole_recognition/position_correction/use_gps", mUseGPS);

    mUnalteredPosCorrection = odom;
}


// Works only in 2D-mode
void Position_Correction_Publisher::addYawOffsetToQuaternion(geometry_msgs::Quaternion &quat, double yaw) const
{
    double siny_cosp = 2.0 * (quat.w * quat.z);
    double cosy_cosp = 1.0 - 2.0 * (std::pow(quat.z, 2.0));
    double old_yaw = std::atan2(siny_cosp, cosy_cosp);
    double new_yaw = old_yaw + yaw;

    double cy = std::cos(new_yaw * 0.5);
    double sy = std::sin(new_yaw * 0.5);
    quat.w = cy;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = sy;
}


void Position_Correction_Publisher::callbackMatchings(const pole_based_localization::PoleMatchArray &matchings)
{
    visualizeMatchings(matchings);
}


void Position_Correction_Publisher::visualizeMatchings(const pole_based_localization::PoleMatchArray &matchings) const
{
    using namespace visualization_msgs;

    // Delete all old markers
    MarkerArray matchingsMarker;
    Marker matchingMarker;
    matchingMarker.header.frame_id = matchings.header.frame_id;
    matchingMarker.header.stamp = matchings.header.stamp;
    matchingMarker.action = matchingMarker.DELETEALL;
    matchingMarker.ns = "matchings";

    geometry_msgs::Point orientationStart, orientationStartUncorrected;
    orientationStartUncorrected.x = mLastPose.pose.position.x;
    orientationStartUncorrected.y = mLastPose.pose.position.y;
    orientationStartUncorrected.z = mLastPose.pose.position.z;
    orientationStart = orientationStartUncorrected;
    orientationStart.x -= mUnalteredPosCorrection.pose.pose.position.x;
    orientationStart.y -= mUnalteredPosCorrection.pose.pose.position.y;
    geometry_msgs::Point orientationEnd;
    orientationEnd = orientationStart;
    geometry_msgs::Point orientationEndUncorrected;
    orientationEndUncorrected = orientationStartUncorrected;

    // extract yaw from quaternion
    double yaw = tf2::getYaw(mLastPose.pose.orientation);//std::atan2(siny_cosp, cosy_cosp);
    double unalteredYaw = -tf2::getYaw(mUnalteredPosCorrection.pose.pose.orientation);
    orientationEnd.x += 50.0 * std::cos(yaw-unalteredYaw);
    orientationEnd.y += 50.0 * std::sin(yaw-unalteredYaw);
    orientationEndUncorrected.x += 50.0 * std::cos(yaw);
    orientationEndUncorrected.y += 50.0 * std::sin(yaw);


    matchingMarker.type = matchingMarker.ARROW;
    matchingMarker.action = matchingMarker.ADD;
    matchingMarker.scale.x = .1;
    matchingMarker.scale.y = .1;
    matchingMarker.scale.z = .1;
    matchingMarker.ns = "orientation";
    matchingMarker.id = 15345;
    matchingMarker.lifetime = ros::Duration(0.1);
    matchingMarker.color.r = 0.0;
    matchingMarker.color.g = mUnalteredPosCorrection.pose.pose.position.x == 0.0 ? 0.1 : 1.0;
    matchingMarker.color.b = 0.0;
    matchingMarker.color.a = 1.0;
    matchingMarker.pose.orientation.x = 0.0;
    matchingMarker.pose.orientation.y = 0.0;
    matchingMarker.pose.orientation.z = 0.0;
    matchingMarker.pose.orientation.w = 1.0;
    matchingMarker.points.push_back(orientationStart);
    matchingMarker.points.push_back(orientationEnd);
    matchingsMarker.markers.push_back(matchingMarker);

    Marker matching2Marker;
    matching2Marker.header.frame_id = "map";
    matching2Marker.header.stamp = matchings.header.stamp;
    matching2Marker.type = matching2Marker.ARROW;
    matching2Marker.action = matching2Marker.ADD;
    matching2Marker.scale.x = .1;
    matching2Marker.scale.y = .1;
    matching2Marker.scale.z = .1;
    matching2Marker.ns = "orientation";
    matching2Marker.id = 15346;
    matching2Marker.lifetime = ros::Duration(0.1);
    matching2Marker.color.r = 1.0;
    matching2Marker.color.g = 0.0;
    matching2Marker.color.b = 0.0;
    matching2Marker.color.a = 1.0;
    matching2Marker.pose.orientation.x = 0.0;
    matching2Marker.pose.orientation.y = 0.0;
    matching2Marker.pose.orientation.z = 0.0;
    matching2Marker.pose.orientation.w = 1.0;
    matching2Marker.points.push_back(orientationStartUncorrected);
    matching2Marker.points.push_back(orientationEndUncorrected);
    matchingsMarker.markers.push_back(matching2Marker);

    int id = 0;

    Marker lineListMarker;
    lineListMarker.header.frame_id = "map";
    lineListMarker.header.stamp = matchings.header.stamp;
    lineListMarker.action = visualization_msgs::Marker::ADD;
    lineListMarker.type = visualization_msgs::Marker::LINE_LIST;
    lineListMarker.scale.x = 0.2;
    lineListMarker.ns = "wallMatchings";
    lineListMarker.id = id++;
    lineListMarker.lifetime = ros::Duration(0.2);
    lineListMarker.pose.orientation.w = 1.0;

    for(const pole_based_localization::PoleMatch& match : matchings.matches){
        if(match.local.type == pole_based_localization::Pole::TYPE_WALL || match.local.type == pole_based_localization::Pole::TYPE_LANELINE){
            geometry_msgs::Point startLocal, endLocal, startRemote, endRemote;
            startLocal.x = match.local.pose.position.x;
            startLocal.y = match.local.pose.position.y;
            startLocal.z = match.local.type == pole_based_localization::Pole::TYPE_LANELINE ? 0.0 : 2.0;            
            endLocal.x = -std::sin(-match.local.pose.orientation.z) * match.local.pose.orientation.x + startLocal.x;
            endLocal.y = std::cos(-match.local.pose.orientation.z) * match.local.pose.orientation.x + startLocal.y;
            endLocal.z = startLocal.z;
            tf2::doTransform(startLocal, startLocal, mUTMMapTransform);
            tf2::doTransform(endLocal, endLocal, mUTMMapTransform);

            startRemote.x = match.remote.pose.position.x;
            startRemote.y = match.remote.pose.position.y;
            startRemote.z = startLocal.z;
            endRemote.x = -std::sin(-match.remote.pose.orientation.z) * match.remote.pose.orientation.x + startRemote.x;
            endRemote.y = std::cos(-match.remote.pose.orientation.z) * match.remote.pose.orientation.x + startRemote.y;
            endRemote.z = startRemote.z;
            tf2::doTransform(startRemote, startRemote, mUTMMapTransform);
            tf2::doTransform(endRemote, endRemote, mUTMMapTransform);

            std_msgs::ColorRGBA colorLocal, colorRemote;
            colorLocal.r = 0.0;
            colorLocal.g = 0.0;
            colorLocal.b = 1.0;
            colorLocal.a = 1.0;
            colorRemote.r = 1.0;
            colorRemote.g = 0.0;
            colorRemote.b = 0.0;
            colorRemote.a = 1.0;

            lineListMarker.colors.push_back(colorLocal);
            lineListMarker.colors.push_back(colorLocal);
            lineListMarker.colors.push_back(colorRemote);
            lineListMarker.colors.push_back(colorRemote);            
            lineListMarker.points.push_back(startLocal);
            lineListMarker.points.push_back(endLocal);
            lineListMarker.points.push_back(startRemote);
            lineListMarker.points.push_back(endRemote);
        }

        Marker marker;
        marker.header.frame_id = match.local.header.frame_id;
        marker.header.stamp = matchings.header.stamp;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = .4;
        marker.scale.y = .4;
        marker.scale.z = 4.0;
        marker.ns = "matchings";
        marker.id = id++;
        marker.lifetime = ros::Duration(0.3);
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position = match.local.pose.position;
        marker.pose.position.z = 2.0;
        matchingsMarker.markers.push_back(marker);

        Marker marker_remote;
        marker_remote.header.frame_id = match.remote.header.frame_id;
        marker_remote.header.stamp = matchings.header.stamp;
        marker_remote.action = visualization_msgs::Marker::ADD;
        marker_remote.type = visualization_msgs::Marker::CYLINDER;
        marker_remote.scale.x = .4;
        marker_remote.scale.y = .4;
        marker_remote.scale.z = 4.0;
        marker_remote.ns = "matchings";
        marker_remote.id = id++;
        marker_remote.lifetime = ros::Duration(0.3);
        marker_remote.color.r = 1.0;
        marker_remote.color.g = 0.0;
        marker_remote.color.b = 0.0;
        marker_remote.color.a = 1.0;
        marker_remote.pose.orientation.w = 1.0;
        marker_remote.pose.position = match.remote.pose.position;
        marker_remote.pose.position.z = 2.0;
        matchingsMarker.markers.push_back(marker_remote);
    }

    if(!lineListMarker.points.empty())
        matchingsMarker.markers.push_back(lineListMarker);
    mMatchingMarkerPublisher.publish(matchingsMarker);
}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pole_recognition::Position_Correction_Publisher, nodelet::Nodelet);
