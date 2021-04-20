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

#include "SimplePoleDetector.h"

#include <pole_based_localization/Pole.h>
#include <pole_based_localization/PoleArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <chrono>
#include <string>
#include <random>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/ColorRGBA.h>

#include <numeric>



/* -------------------------------------------------------------------------- */

/** Simple fast pole/wall detector and tracker
 **
 ** @ingroup @@
 */

namespace simple_pole_detector{

SimplePoleDetector::SimplePoleDetector() : mTrackingEnabled(true), mMaxPairingDist(0.4), mMaxPairingAngle(0.05), mDetectCorners(true), mDetectWalls(true), mLogFeatures(false), mTfListener(mTfBuffer)
{
    std::srand(std::time(nullptr));
}


SimplePoleDetector::~SimplePoleDetector()
{
    if(mLogFeatures)
        mCSVFile.close();
}


void SimplePoleDetector::onInit()
{
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<pole_based_localization::pole_detectionConfig>::CallbackType config_callback;

    std::string topic = nh.resolveName("/sensors/velodyne_points");
    mPoleArrayPublisher = nh.advertise<pole_based_localization::PoleArray>("/pole_recognition/trackedPolesArray",1);
    mMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/pole_recognition/pole_marker", 1);

    nh.getParam("/pole_recognition/log_features", mLogFeatures);

    if(mLogFeatures)
        mCSVFile.open("csv/poles.csv");

    mConfigServer =
        boost::make_shared<dynamic_reconfigure::Server<pole_based_localization::pole_detectionConfig> >(getPrivateNodeHandle());
    config_callback = boost::bind(&SimplePoleDetector::callbackReconfigure, this, _1, _2);
    mConfigServer->setCallback(config_callback);

    mSubscriber = nh.subscribe(topic, 1, &SimplePoleDetector::callback, this);
    mLastPosSubscriber = nh.subscribe(nh.resolveName("/localization/odometry/filtered_map"), 1, &SimplePoleDetector::callbackPos, this);
}


void SimplePoleDetector::callbackPos(const nav_msgs::OdometryConstPtr& odom)
{
    mLastPos = *odom;
}


void SimplePoleDetector::callbackMatchings(const pole_based_localization::PoleMatchArray &matchings)
{
    mMatchedFeatures.clear();
    for(auto match : matchings.matches)
        mMatchedFeatures.push_back(std::stol(match.local.ID));
}


void SimplePoleDetector::callbackReconfigure(pole_based_localization::pole_detectionConfig &config, uint32_t level)
{
    mConfig = config;
    mDetectCorners = config.detect_corners;
    mDetectWalls = config.detect_walls;
    mInitialConfidence = config.initial_confidence;
    mRedetectionConfidenceInc = config.redetection_confidence_increase;
    mNoRedetectionConfidenceDec = config.no_redetection_confidence_decrease;
}


void SimplePoleDetector::sendVisualizationMarker(ros::Time timestamp, std::basic_string<char> frame_id, const std::vector<std::tuple<long, PCLPoint, int, int> > &trackedPoles) const
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;
    for(auto pole : trackedPoles)
    {
        long poleID;
        PCLPoint polePos;
        int poleLife, poleDetections;
        std::tie(poleID, polePos, poleLife, poleDetections) = pole;
        geometry_msgs::Point p;
        p.x = polePos.x;
        p.y = polePos.y;
        p.z = 2.0f; //fixed z...

        std_msgs::ColorRGBA c;
        c.g = 1.0f;
        c.r = 1.0f;
        c.a = std::min(1.0f, poleLife/80.0f);

        //marker.points.push_back(p);
        //marker.colors.push_back(c);

        marker.header.frame_id = frame_id;
        marker.header.stamp = timestamp;
        marker.ns = "poles";
        marker.id = poleID;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = p;
        marker.lifetime = ros::Duration(1.0);;
        marker.color = c;
        marker.scale.x = .3f;
        marker.scale.y = .3f;
        marker.scale.z = 4.0f;

        markerArray.markers.push_back(marker);
    }

    mMarkerPublisher.publish(markerArray);
}


void SimplePoleDetector::logPolesToCsv(const std::vector<std::tuple<long, PCLPoint, int, int> > &trackedPoles)
{
    static std::set<long> loggedAlready;
    for(auto pole : trackedPoles){
        long poleID;
        PCLPoint polePos;
        int poleLife, poleDetections;
        std::tie(poleID, polePos, poleLife, poleDetections) = pole;
        polePos = map2UTM(polePos, ros::Time::now());
        if(poleLife >= 80 && loggedAlready.insert(poleID).second)
            mCSVFile << std::fixed << polePos.x << "," << std::fixed << polePos.y << std::endl;
    }
}


void SimplePoleDetector::sendWallVizMarker(ros::Time timestamp, std::basic_string<char> frame_id, const std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > &trackedWalls) const
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;
    for(auto pole : trackedWalls)
    {
        long wallID;
        std::pair<PCLPoint, PCLPoint> wallPos;
        int wallLife, wallDetections;
        std::tie(wallID, wallPos, wallLife, wallDetections) = pole;
        geometry_msgs::Point start, end;
        start.x = wallPos.first.x;
        start.y = wallPos.first.y;
        start.z = 2.0f; //fixed z...
        end.x = wallPos.second.x;
        end.y = wallPos.second.y;
        end.z = 2.0f; //fixed z...

        std::vector<geometry_msgs::Point> points;
        points.push_back(start);
        points.push_back(end);

        std_msgs::ColorRGBA c;
        c.b = (wallID % 3)/3.0;
        c.r = (wallID % 5)/5.0;
        c.g = (wallID % 7)/7.0;
        c.a = std::min(1.0f, (float)wallLife/80.0f);

        marker.header.frame_id = frame_id;
        marker.header.stamp = timestamp;
        marker.ns = "poles";
        marker.id = wallID%2147483647; // marker.id is just int32 and wallID is long, so we clamp it.
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points = points;
        marker.lifetime = ros::Duration(1.0);
        marker.color = c;
        marker.scale.x = .33f;

        markerArray.markers.push_back(marker);
    }
    mMarkerPublisher.publish(markerArray);
}


void SimplePoleDetector::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    auto start = std::chrono::system_clock::now();
    static double avg_time = 0.0;
    static unsigned int time_vals = 0;
    pcl::PointCloud<PCLPoint>::Ptr cloud (new pcl::PointCloud<PCLPoint>);
    pcl::fromROSMsg (*cloud_msg, *cloud);

    const static unsigned int ring_count = detectRingCount(cloud);
    static unsigned int curRing = ring_count >= 64 ? 52 : 17, curCornerRing = ring_count >= 64 ? 45 : 22, curWallRing = curCornerRing;

    float maxWidth = 3.2;
    float minWidth = 0.03;
    float minSpacing = 1.0;

    double angleTol = M_PI/20.0;
    float minCornerWalllength = 1.0f;
    float minWalllength = 5.0f;
    float maxInlierDist = 0.8f;

    std::vector<unsigned int> rings, cornerRings, wallRings;
    if(ring_count == 128){
        if(curRing > 98) curRing = 20;
        if(curCornerRing > 122) curCornerRing = 60;
        if(curWallRing > 122) curWallRing = 60;

        rings = {curRing, curRing+3, curRing+6, curRing+9, curRing+12, curRing+15, curRing+18, curRing+21};
        cornerRings = {curCornerRing, curCornerRing+1, curCornerRing+2, curCornerRing+3};
        wallRings = {curWallRing, curWallRing+1, curWallRing+2, curWallRing+3};
    }
    else if(ring_count == 64){
        // 64 and 32
        if(curRing > 54) curRing = 10;
        // 16
//        if(curRing > 52) curRing = 30;
//        if(curCornerRing > 60) curCornerRing = 30;
        // 64
        if(curWallRing > 60) curWallRing = 45;
//        if(curWallRing > 60) curWallRing = 44;

        // 64
        rings = {curRing, curRing+2, curRing+4, curRing+6, curRing+8, curRing+9, curRing+10, curRing+11};
        // 32
//        rings = {curRing, curRing+2, curRing+4, curRing+6, curRing+8};
        //16
        //rings = {curRing, curRing+4, curRing+8, curRing+12};
        // 64
        cornerRings = {curCornerRing, curCornerRing+1, curCornerRing+2, curCornerRing+3};
        // 32
//        cornerRings = {curCornerRing, curCornerRing+2, curCornerRing+4};
        // 16
//        cornerRings = {curCornerRing, curCornerRing+4};
        // 64
        wallRings = {curWallRing, curWallRing+1, curWallRing+2, curWallRing+3};
        // 32
//        wallRings = {curWallRing, curWallRing+2, curWallRing+4};
        // 16
//        wallRings = {curWallRing, curWallRing+4};
    }
    else{ // 32 beams
        if(curRing > 27) curRing = 5;
        if(curCornerRing > 30) curCornerRing = 22;
        if(curWallRing > 30) curWallRing = 22;

        rings = {curRing, curRing+1, curRing+2, curRing+3};
        cornerRings = {curCornerRing, curCornerRing+1};
        wallRings = {curWallRing, curWallRing+1};
    }

    auto generatedHypos = genHypotheses(cloud, rings, minWidth, maxWidth, minSpacing);
    auto generatedCorners = mDetectCorners ? findCorners(cloud, cornerRings, angleTol, minCornerWalllength, maxInlierDist) : std::vector<std::vector<PCLPoint> >();
    std::vector<std::pair<PCLPoint, PCLPoint> > generatedWalls = mDetectWalls ? findWalls(cloud, wallRings, angleTol, minWalllength, maxInlierDist) : std::vector<std::pair<PCLPoint, PCLPoint> >();
    std::vector<std::pair<int, PCLPoint> > hypotheses = mergeHypotheses(generatedHypos);
    std::vector<std::pair<int, PCLPoint> > cornerHypotheses = mergeHypotheses(generatedCorners);
    std::vector<std::pair<PCLPoint, PCLPoint> > wallHypotheses = mergeWallHypotheses(generatedWalls);

    // transform detections into odom frame
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = mTfBuffer.lookupTransform("odom", "base_link", cloud_msg->header.stamp, ros::Duration(0.05));
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return; // give up on that detection
    }

    for(std::pair<int, PCLPoint>& hypo : hypotheses)
        hypo = std::make_pair(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"base_link",transformStamped));

    for(std::pair<int, PCLPoint>& hypo : cornerHypotheses)
        hypo = std::make_pair(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"base_link",transformStamped));

    std::vector<std::pair<PCLPoint, PCLPoint> > transformedWalls;
    for(auto hypo : wallHypotheses){
        PCLPoint p1 = transformPclPoint(std::get<0>(hypo), cloud_msg->header.stamp,"base_link",transformStamped);
        PCLPoint p2 = transformPclPoint(std::get<1>(hypo), cloud_msg->header.stamp,"base_link",transformStamped);

        // sort wall hypotheses by x
        std::pair<PCLPoint, PCLPoint> wallHypo = p1.x < p2.x ? std::make_pair(p1, p2) : std::make_pair(p2, p1);
        transformedWalls.push_back(wallHypo);
    }
    wallHypotheses = transformedWalls;

    PCLPoint origin;
    origin.x = 0;
    origin.y = 0;
    origin = transformPclPoint(origin, cloud_msg->header.stamp, "base_link", transformStamped);

    curRing += (rings.back()-rings.front())/2;
    curCornerRing += (cornerRings.back()-cornerRings.front())/2;
    curWallRing += (wallRings.back()-wallRings.front())/2;

    // 16 beams
//    curRing += 4;
//    curCornerRing += 4;
//    curWallRing += 4;

    auto tracked = trackPoles(hypotheses);
    auto trackedCorners = trackCorners(cornerHypotheses);
    auto trackedWalls = trackWalls(wallHypotheses, origin);

    // transform via base_link to map
    try{
        transformStamped = mTfBuffer.lookupTransform("base_link", "odom", cloud_msg->header.stamp, ros::Duration(0.05));
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return; // give up on that detection
    }

    for(std::tuple<long, PCLPoint, int, int>& hypo : tracked)
        hypo = std::make_tuple(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"odom",transformStamped), std::get<2>(hypo), std::get<3>(hypo));

    for(std::tuple<long, PCLPoint, int, int>& hypo : trackedCorners)
        hypo = std::make_tuple(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"odom",transformStamped), std::get<2>(hypo), std::get<3>(hypo));

    for(auto& hypo : trackedWalls){
        PCLPoint p1 = transformPclPoint(std::get<0>(std::get<1>(hypo)), cloud_msg->header.stamp,"odom",transformStamped);
        PCLPoint p2 = transformPclPoint(std::get<1>(std::get<1>(hypo)), cloud_msg->header.stamp,"odom",transformStamped);

        // sort wall hypotheses by x
        std::pair<PCLPoint, PCLPoint> wallHypo = p1.x < p2.x ? std::make_pair(p1, p2) : std::make_pair(p2, p1);
        hypo = std::make_tuple(std::get<0>(hypo), wallHypo, std::get<2>(hypo), std::get<3>(hypo));
    }

    try{
        transformStamped = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.05));
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return; // give up on that detection
    }

    for(std::tuple<long, PCLPoint, int, int>& hypo : tracked)
        hypo = std::make_tuple(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"base_link",transformStamped), std::get<2>(hypo), std::get<3>(hypo));

    for(std::tuple<long, PCLPoint, int, int>& hypo : trackedCorners)
        hypo = std::make_tuple(std::get<0>(hypo), transformPclPoint(std::get<1>(hypo),cloud_msg->header.stamp,"base_link",transformStamped), std::get<2>(hypo), std::get<3>(hypo));

    for(auto& hypo : trackedWalls){
        PCLPoint p1 = transformPclPoint(std::get<0>(std::get<1>(hypo)), cloud_msg->header.stamp,"base_link",transformStamped);
        PCLPoint p2 = transformPclPoint(std::get<1>(std::get<1>(hypo)), cloud_msg->header.stamp,"base_link",transformStamped);

        // sort wall hypotheses by x
        std::pair<PCLPoint, PCLPoint> wallHypo = p1.x < p2.x ? std::make_pair(p1, p2) : std::make_pair(p2, p1);
        hypo = std::make_tuple(std::get<0>(hypo), wallHypo, std::get<2>(hypo), std::get<3>(hypo));
    }

    if(mPoleArrayPublisher.getNumSubscribers() > 0)
    {
        pole_based_localization::PoleArray poleArray;
        poleArray.header.stamp = cloud_msg->header.stamp;
        poleArray.header.frame_id = "map";
        poleArray.poles.reserve(tracked.size()+trackedCorners.size()+trackedWalls.size());

        for(auto entry : tracked)
        {
            pole_based_localization::Pole pole;
            long poleID;
            PCLPoint polePos;
            int poleLife, poleDetections;
            std::tie(poleID, polePos, poleLife, poleDetections) = entry;
            pole.header.stamp = cloud_msg->header.stamp;
            pole.header.frame_id = "map";
            pole.ID = std::to_string(poleID);
            pole.type = pole_based_localization::Pole::TYPE_POLE;
            pole.pose.position.x = polePos.x;
            pole.pose.position.y = polePos.y;
            pole.pose.position.z = polePos.z;
            pole.life = poleLife;
            poleArray.poles.push_back(pole);
        }

        for(auto entry : trackedCorners)
        {
            pole_based_localization::Pole pole;
            long poleID;
            PCLPoint polePos;
            int poleLife, poleDetections;
            std::tie(poleID, polePos, poleLife, poleDetections) = entry;
            pole.header.stamp = cloud_msg->header.stamp;
            pole.header.frame_id = "map";
            pole.ID = std::to_string(poleID);
            pole.type = pole_based_localization::Pole::TYPE_CORNER;
            pole.pose.position.x = polePos.x;
            pole.pose.position.y = polePos.y;
            pole.pose.position.z = polePos.z;
            pole.life = poleLife;
            poleArray.poles.push_back(pole);
        }

        for(auto entry : trackedWalls)
        {
            pole_based_localization::Pole pole;
            long wallID;
            std::pair<PCLPoint, PCLPoint> wallPoints;
            int wallLife, wallDetections;
            std::tie(wallID, wallPoints, wallLife, wallDetections) = entry;
            pole.header.stamp = cloud_msg->header.stamp;
            pole.header.frame_id = "map";
            pole.ID = std::to_string(wallID);
            pole.type = pole_based_localization::Pole::TYPE_WALL;
            pole.pose.position.x = wallPoints.first.x;
            pole.pose.position.y = wallPoints.first.y;
            pole.pose.position.z = wallPoints.first.z;
            std::pair<PCLPoint, PCLPoint> utmWallPoints;
            utmWallPoints.first = map2UTM(wallPoints.first, pole.header.stamp);
            utmWallPoints.second = map2UTM(wallPoints.second, pole.header.stamp);
            pole.pose.orientation.z = calcAngle(utmWallPoints); // We need to calculate the angle in utm to avoid meridian convergence problems
            pole.pose.orientation.x = std::sqrt(std::pow(wallPoints.second.x-wallPoints.first.x, 2.0) + std::pow(wallPoints.second.y-wallPoints.first.y, 2.0)); // Since we're operating only in 2d the x-value is free to be abused for the wall length
            pole.life = wallLife;
            poleArray.poles.push_back(pole);
        }

        mPoleArrayPublisher.publish(poleArray);
    }

    //publish debug marker
    if(mMarkerPublisher.getNumSubscribers() > 0){
        sendVisualizationMarker(ros::Time::now(), "map", tracked);
        sendVisualizationMarker(ros::Time::now(), "map", trackedCorners);
        sendWallVizMarker(ros::Time::now(), "map", trackedWalls);
    }

    if(mLogFeatures)
        logPolesToCsv(tracked);

    auto endDetection = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = endDetection-start;
    const double milliseconds = elapsed_seconds.count() * 1000;
    avg_time = (milliseconds + time_vals * avg_time)/(time_vals+1);
    ++time_vals;
    ROS_DEBUG_STREAM("Found " << hypotheses.size() << " (ring " << curRing << ") and tracked " << tracked.size() << " poles in " << milliseconds << "ms (avg " << avg_time << ")");
}


unsigned int SimplePoleDetector::detectRingCount(const pcl::PointCloud<PCLPoint>::Ptr cloud) const
{
    unsigned int ring_count = 0;
    for(size_t i = 0; i < cloud->points.size(); ++i)
        if(cloud->points[i].ring > ring_count) ring_count = cloud->points[i].ring;

    return ring_count+1;
}


double SimplePoleDetector::calcDist(const PCLPoint point1, const PCLPoint point2) const
{
    return sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0));// + pow(point1.z - point2.z, 2.0));no z right now
}


double SimplePoleDetector::calcDist(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) const
{
    return sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0));// + pow(point1.z - point2.z, 2.0)); no z right now
}


double SimplePoleDetector::calcAngle(const std::pair<PCLPoint, PCLPoint> &direction) const
{
    double xDir = direction.second.x - direction.first.x;
    double yDir = direction.second.y - direction.first.y;
    if(xDir < 0.0){
        xDir *= -1.0;
        yDir *= -1.0;
    }
    return std::acos(yDir/std::hypot(xDir, yDir));
}


SimplePoleDetector::PCLPoint SimplePoleDetector::calcAvg(const std::vector<PCLPoint> points) const
{
    double curX, curY, curZ;
    curX = curY = curZ = 0.0;
    for(auto point : points)
    {
        curX += point.x;
        curY += point.y;
        curZ += point.z;
    }

    unsigned int len = points.size();
    PCLPoint result;
    result.x = curX/len;
    result.y = curY/len;
    result.z = curZ/len;

    return result;
}


double SimplePoleDetector::calcLineDist(const std::vector<PCLPoint>& linePoints, const PCLPoint& point) const
{
    if(linePoints.empty()) return 0;
    if(linePoints.size() == 1) return calcDist(linePoints[0], point);

    return calcLineDist(std::make_pair(linePoints.front(), linePoints.back()), point);
}


double SimplePoleDetector::calcLineDist(const std::pair<PCLPoint, PCLPoint> &linePoints, const PCLPoint& point) const
{
    const PCLPoint& first = linePoints.first;
    const PCLPoint& last = linePoints.second;

    return std::abs(((double)last.y-(double)first.y) * (double)point.x - ((double)last.x-(double)first.x) * (double)point.y + (double)last.x*(double)first.y - (double)last.y*(double)first.x)/calcDist(first, last);
}


double SimplePoleDetector::calcLineAngle(const std::pair<PCLPoint, PCLPoint>& linePoints1, const std::pair<PCLPoint, PCLPoint>& linePoints2) const
{
    double x1 = linePoints1.second.x-linePoints1.first.x;
    double y1 = linePoints1.second.y-linePoints1.first.y;
    double x2 = linePoints2.second.x-linePoints2.first.x;
    double y2 = linePoints2.second.y-linePoints2.first.y;

    double l1 = std::sqrt(std::pow(x1, 2.0) + std::pow(y1, 2.0));
    double l2 = std::sqrt(std::pow(x2, 2.0) + std::pow(y2, 2.0));

    // always measured in position x direction
    if(x1 < 0.0){        
        x1 *= -1;
        y1 *= -1;        
    }
    if(x2 < 0.0){
        x2 *= -1;
        y2 *= -1;
    }

    double result = (x1*x2+y1*y2)/(l1 * l2);
    result = std::max(result, -1.0);
    result = std::min(result, 1.0);
    result = std::acos(result);

    return result;
}


double SimplePoleDetector::calcLineAngle(const std::vector<PCLPoint>& linePoints1, const std::vector<PCLPoint>& linePoints2) const
{
    return calcLineAngle(std::make_pair(linePoints1.front(), linePoints1.back()), std::make_pair(linePoints2.front(), linePoints2.back()));
}


std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > SimplePoleDetector::findWalls(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int> &rings, double angleTolerance, float minWallLength, float maxInlierDist) const
{
    std::vector<std::vector<PCLPoint>> curLine;
    std::vector<std::pair<PCLPoint, PCLPoint>> hypotheses;
    auto ring_count = rings.size();
    curLine.resize(ring_count);

    // initialization
    for(int i=0; i < ring_count; ++i){
        curLine[i] = std::vector<PCLPoint>();
    }

    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
        PCLPoint point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = 0.0; // cloud.points[i].z; z value is currently ignored
        point.ring = cloud->points[i].ring;
        int ring_id = -1;

        for(int j=0; j<ring_count; ++j)
            if(rings[j] == cloud->points[i].ring)
            {
                ring_id = j;
                break;
            }

        if(ring_id > -1)
        {
            int first = curLine[ring_id].size() > 3 ? std::rand()%((curLine[ring_id].size()-1)/2) : 0; // I know it's biased but it doesn't matter here
            int second = curLine[ring_id].size() > 3 ? (curLine[ring_id].size()/2)+std::rand()%((curLine[ring_id].size()-1)/2) : curLine[ring_id].size()-1;
            if(curLine[ring_id].size() > 0 && (calcLineDist(std::make_pair(curLine[ring_id][first],curLine[ring_id][second]), point) > maxInlierDist || calcDist(curLine[ring_id].back(), point) > maxInlierDist*2.0))
            {
                if(curLine[ring_id].size() > 1 && calcDist(curLine[ring_id].front(), curLine[ring_id].back()) > minWallLength)
                    hypotheses.push_back(ransacLineFit(curLine[ring_id]));
                curLine[ring_id].clear();
            }
            curLine[ring_id].push_back(point);
        }
    }

    return hypotheses;
}


std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> SimplePoleDetector::ransacLineFit(std::vector<SimplePoleDetector::PCLPoint> pointsToFit) const
{
    if(pointsToFit.size() < 2) return std::pair<PCLPoint, PCLPoint>();

    int N = 20; // sample size, iteration count
    double inlierThreshold = 0.15;
    std::vector<PCLPoint> inlier;

    std::vector<PCLPoint> cInlier;
    std::random_device rd;
    std::mt19937 g(rd());

    for(int i=0; i<N; ++i)
    {
        std::shuffle(pointsToFit.begin(), pointsToFit.end(), g);

        std::pair<PCLPoint, PCLPoint> model = std::make_pair(pointsToFit.front(), pointsToFit[1]);

        for(auto pointsIt = pointsToFit.begin(); pointsIt != pointsToFit.end(); ++pointsIt)
            if(calcLineDist(model, *pointsIt) < inlierThreshold) cInlier.push_back(*pointsIt);

        if(cInlier.size() > inlier.size())
        {
            inlier.clear();
            inlier.insert(inlier.begin(), cInlier.begin(), cInlier.end());
        }

        cInlier.clear();
    }        

    if(inlier.size() >= pointsToFit.size()-(pointsToFit.size()/20.0)){
        // use x dimension to get the points with the biggest distance
        PCLPoint minX = inlier[0], maxX = inlier[0];
        for(auto in : inlier){
            if(in.x < minX.x) minX = in;
            if(in.x > maxX.x) maxX = in;
        }
        return std::make_pair(minX, maxX);
    }
    else return ransacLineFit(inlier); // refit with inliers
}


std::vector<std::vector<SimplePoleDetector::PCLPoint> >  SimplePoleDetector::findCorners(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int>& rings, double angleTolerance, float minWallLength, float maxInlierDist) const
{
    std::vector<std::vector<PCLPoint> > curLine;
    std::vector<std::vector<PCLPoint> > lastLine;
    std::vector<std::vector<PCLPoint> > hypotheses;
    auto ring_count = rings.size();
    curLine.resize(ring_count);
    lastLine.resize(ring_count);

    // initialization
    for(int i=0; i < ring_count; ++i)
    {
        curLine[i] = std::vector<PCLPoint>();
        lastLine[i] = std::vector<PCLPoint>();
        hypotheses.push_back(std::vector<PCLPoint>());
    }

    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
        PCLPoint point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = 0.0; // cloud.points[i].z; z value is currently ignored
        point.ring = cloud->points[i].ring;
        int ring_id = -1;

        for(int j=0; j<ring_count; ++j)
            if(rings[j] == cloud->points[i].ring)
            {
                ring_id = j;
                break;
            }

        if(ring_id > -1)
        {
            if(calcLineDist(curLine[ring_id], point) > maxInlierDist){ // start of a new line
                if(!lastLine[ring_id].empty())
                    lastLine[ring_id].erase(lastLine[ring_id].begin());
                lastLine[ring_id].push_back(curLine[ring_id].front());
                curLine[ring_id].clear();
            }


            curLine[ring_id].push_back(point);
            if(curLine[ring_id].size() < 2 || calcDist(curLine[ring_id][0], point) < minWallLength)
                    continue;

            if(!lastLine[ring_id].empty() && calcDist(lastLine[ring_id].back(), curLine[ring_id].front()) > maxInlierDist) // Don't allow too big gaps between the corner lines
                lastLine[ring_id].clear();

            if(lastLine[ring_id].size() >= 2 && calcDist(lastLine[ring_id].front(), lastLine[ring_id].back()) >= minWallLength)
            {
                double angle = calcLineAngle(curLine[ring_id], lastLine[ring_id]);
                if((angle > M_PI_2 - angleTolerance && angle < M_PI_2 + angleTolerance) || (angle > -M_PI_2 - angleTolerance && angle < -M_PI_2 + angleTolerance))
                {
                    hypotheses[ring_id].push_back(curLine[ring_id].front());
                    lastLine[ring_id] = curLine[ring_id];
                    curLine[ring_id].clear();
                }
                else
                {
                    lastLine[ring_id].erase(lastLine[ring_id].begin());
                    lastLine[ring_id].push_back(curLine[ring_id].front());
                    curLine[ring_id].erase(curLine[ring_id].begin());
                }
            }
            else
            {
                lastLine[ring_id].push_back(curLine[ring_id].front());
                curLine[ring_id].erase(curLine[ring_id].begin());
            }
        }
    }

    return hypotheses;
}


std::vector<std::vector<SimplePoleDetector::PCLPoint> >  SimplePoleDetector::genHypotheses(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int>& rings, double minWidth, double maxWidth, double minSpacing) const
{
    std::vector<std::vector<PCLPoint> > result = std::vector<std::vector<PCLPoint> >();
    std::vector<PCLPoint> lastPoint;
    std::vector<std::vector<PCLPoint> > recorded;
    std::vector<float> curWidth;
    auto ring_count = rings.size();
    std::vector<bool> first;

    lastPoint.resize(ring_count);
    recorded.resize(ring_count);
    curWidth.resize(ring_count);
    first.resize(ring_count);

    // initialization
    for(int i=0; i < ring_count; ++i)
    {
        lastPoint[i] = PCLPoint();
        recorded[i] = std::vector<PCLPoint>();
        curWidth[i] = 0.0;
        first[i] = false;
        result.push_back(std::vector<PCLPoint>());
    }

    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
        PCLPoint point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = 0.0; // cloud.points[i].z; z value is currently ignored
        point.ring = cloud->points[i].ring;
        int ring_id = -1;

        for(int j=0; j<ring_count; ++j)
            if(rings[j] == cloud->points[i].ring)
            {
                ring_id = j;
                break;
            }

        double dist = std::hypot(point.x, point.y);
        if(ring_id > -1 && (dist > 2.0))
        {
            if(first[ring_id])
            {
                first[ring_id] = false;
                lastPoint[ring_id] = point;
                recorded[ring_id].clear();
            }
            else if(recorded[ring_id].empty() && calcDist(lastPoint[ring_id], point) < minSpacing)
            {
                lastPoint[ring_id] = point;
                curWidth[ring_id] = 0.0f;
            }
            else if(curWidth[ring_id] < maxWidth)
            {
                float dist = calcDist(lastPoint[ring_id], point);
                if(recorded[ring_id].empty())
                {
                    lastPoint[ring_id] = point;
                    recorded[ring_id].push_back(point);
                }
                else if(dist + curWidth[ring_id] < maxWidth && dist <= minSpacing)
                {
                    curWidth[ring_id] += dist;
                    lastPoint[ring_id] = point;
                    recorded[ring_id].push_back(point);
                }
                else if(curWidth[ring_id] > minWidth && dist >= minSpacing)
                {
                    PCLPoint surfaceCenter = recorded[ring_id][recorded[ring_id].size()/2];
                    PCLPoint intersection = getClosestPointOnLine(std::make_pair(recorded[ring_id].front(), recorded[ring_id].back()), surfaceCenter);
                    PCLPoint direction;
                    direction.x = intersection.x - surfaceCenter.x;
                    direction.y = intersection.y - surfaceCenter.y;
                    // check if pole is concave. Then we have to project the center in the other direction
                    if(std::pow(intersection.x - mLastPos.pose.pose.position.x, 2) + std::pow(intersection.y - mLastPos.pose.pose.position.y, 2) < std::pow(surfaceCenter.x - mLastPos.pose.pose.position.x, 2) + std::pow(surfaceCenter.y - mLastPos.pose.pose.position.y, 2)){
                        direction.x *= -1;
                        direction.y *= -1;
                    }
                    // transform surface center to pole center
                    surfaceCenter.x += direction.x * curWidth[ring_id] * 1.5;
                    surfaceCenter.y += direction.y * curWidth[ring_id] * 1.5;

                    result[ring_id].push_back(surfaceCenter);
                    first[ring_id] = true;
                    curWidth[ring_id] = 0.0f;
                }
                else
                {
                    first[ring_id] = true;
                    curWidth[ring_id] = 0.0f;
                }
            }
        }
    }

    return result;
}


SimplePoleDetector::PCLPoint SimplePoleDetector::getClosestPointOnLine(const std::pair<PCLPoint, PCLPoint> &line, const PCLPoint& point) const
{
    PCLPoint lineVector, pointVector, result;
    lineVector.x = line.second.x - line.first.x;
    lineVector.y = line.second.y - line.first.y;
    pointVector.x = point.x - line.first.x;
    pointVector.y = point.y - line.first.y;

    double dot = lineVector.x * pointVector.x + lineVector.y * pointVector.y;
    dot /= std::pow(lineVector.x, 2.0) + std::pow(lineVector.y, 2.0);

    result.x = line.first.x + lineVector.x * dot;
    result.y = line.first.y + lineVector.y * dot;
    result.z = (line.first.z + line.second.z)/2.0;
    result.intensity = (line.first.intensity + line.second.intensity)/2.0;

    return result;
}


std::vector<std::pair<int, SimplePoleDetector::PCLPoint> > SimplePoleDetector::mergeHypotheses(const std::vector<std::vector<PCLPoint> >& hypotheses) const
{
    std::vector<std::pair<int, PCLPoint> > result;
    std::vector<PCLPoint> hypothesesAllRings;

    // Put everything in one vector
    for(auto hypos : hypotheses)
        for(auto hypo : hypos)
            hypothesesAllRings.push_back(hypo);

    // sort it by angle
    std::sort(hypothesesAllRings.begin(), hypothesesAllRings.end(),  [](PCLPoint a, PCLPoint b) {
        double angleA = std::acos(a.x / std::hypot(a.x, a.y));
        double angleB = std::acos(b.x / std::hypot(b.x, b.y));
        angleA = a.y < 0 ? 2 * M_PI - angleA : angleA;
        angleB = b.y < 0 ? 2 * M_PI - angleB : angleB;
        return angleA > angleB;
    });

    // Merge closeby hypotheses
    for(auto hypo = hypothesesAllRings.begin(); hypo != hypothesesAllRings.end(); ++hypo){
        std::vector<PCLPoint> currentHypos;
        PCLPoint currentHypo = *hypo;
        while(++hypo != hypothesesAllRings.end() && calcDist(currentHypo, *hypo) < mMaxPairingDist){
            currentHypos.push_back(currentHypo);
            currentHypo = *hypo;
        }

        if(currentHypos.size() > 1){
            currentHypos.push_back(currentHypo);
            result.push_back(std::make_pair(currentHypos.size(), calcAvg(currentHypos)));
        }

        if(hypo == hypothesesAllRings.end())
            break;
    }

    return result;
}


std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > SimplePoleDetector::mergeWallHypotheses(const std::vector<std::pair<PCLPoint, PCLPoint> >& hypotheses) const
{
    std::vector<std::pair<PCLPoint, PCLPoint> > result;
    std::vector<std::pair<PCLPoint, PCLPoint> > hypothesesAllRings;

    // Make sure the lines are sorted by x
    for(auto hypo : hypotheses)
        hypothesesAllRings.push_back(sortByX(hypo));

    // sort the start points by observation angle
    std::sort(hypothesesAllRings.begin(), hypothesesAllRings.end(),  [](std::pair<PCLPoint, PCLPoint> a, std::pair<PCLPoint, PCLPoint> b) {
        PCLPoint startA = std::get<0>(a), startB = std::get<0>(b);
        double angleA = std::acos(startA.x / std::hypot(startA.x, startA.y));
        double angleB = std::acos(startB.x / std::hypot(startB.x, startB.y));
        angleA = startA.y < 0 ? 2 * M_PI - angleA : angleA;
        angleB = startB.y < 0 ? 2 * M_PI - angleB : angleB;
        return angleA > angleB;
    });

    // Merge closeby hypotheses
    for(auto hypo = hypothesesAllRings.begin(); hypo != hypothesesAllRings.end(); ++hypo){
        std::vector<std::pair<PCLPoint, PCLPoint> > currentHypos;
        std::pair<PCLPoint, PCLPoint> currentHypo = *hypo;
        while(++hypo != hypothesesAllRings.end() && linesPairable(currentHypo, *hypo, mMaxPairingDist*2.0, mMaxPairingAngle))
            currentHypos.push_back(*hypo);

        if(currentHypos.size() > 0){
            currentHypos.push_back(currentHypo);
            result.push_back(mergeLines(currentHypos));
        }

        if(hypo == hypothesesAllRings.end())
            break;
    }

    return result;
}


const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> >& SimplePoleDetector::trackPoles(const std::vector<std::pair<int, PCLPoint> >& hypotheses) const
{
    static std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > trackedHypos = std::vector<std::tuple<long, PCLPoint, int, int> >();
    trackedHypos = updateTracking(trackedHypos, hypotheses);
    trackedHypos = mergeTracked(trackedHypos);
    return trackedHypos;
}


const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> >& SimplePoleDetector::trackCorners(const std::vector<std::pair<int, PCLPoint> >& hypotheses) const
{
    static std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > trackedHypos = std::vector<std::tuple<long, PCLPoint, int, int> >();
    trackedHypos = updateTracking(trackedHypos, hypotheses);
    trackedHypos = mergeTracked(trackedHypos);
    return trackedHypos;
}


const std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> >& SimplePoleDetector::trackWalls(const std::vector<std::pair<PCLPoint, PCLPoint> >& hypotheses, const PCLPoint origin) const
{
    static std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > trackedHypos = std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> >();
    trackedHypos = updateWallTracking(hypotheses, trackedHypos);
    trackedHypos = mergeTrackedWalls(trackedHypos, origin);
    return trackedHypos;
}


std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > SimplePoleDetector::updateWallTracking(const std::vector<std::pair<PCLPoint, PCLPoint>>& newHypos, std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >& trackedHypos) const
{
    std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > result;

    for(auto tracked : trackedHypos)
    {
        auto ID = std::get<0>(tracked);
        auto hypo = std::get<1>(tracked);
        auto lifes = std::get<2>(tracked);
        auto detectionCount = std::get<3>(tracked);
        auto partners = findPairableLines(hypo, newHypos, mMaxPairingDist*2.0, mMaxPairingAngle);
        if(!partners.empty())
        {
            partners.push_back(hypo);
            result.push_back(std::make_tuple(ID, mergeLines(partners), std::min(100, lifes + mRedetectionConfidenceInc * 2), detectionCount+1));
        }
        else if(lifes > 0) // no redetection
            result.push_back(std::make_tuple(ID, hypo, lifes - mNoRedetectionConfidenceDec, std::max(detectionCount-1,1)));
    }

    for(auto newHypo : newHypos)
    {
        std::vector<std::pair<PCLPoint, PCLPoint> > tracked = std::vector<std::pair<PCLPoint, PCLPoint> >(trackedHypos.size());
        std::transform(trackedHypos.begin(), trackedHypos.end(), tracked.begin(), [](std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int>& p){ return std::get<1>(p);});
        if(findPairableLines(newHypo, tracked, mMaxPairingDist*2.0, mMaxPairingAngle).empty())
            result.push_back(std::make_tuple(std::hash<unsigned int>{}(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()), newHypo, 2*mInitialConfidence, 1));
    }

    return result;
}


std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > SimplePoleDetector::updateTracking(std::vector<std::tuple<long, PCLPoint, int, int> > &trackedHypos, const std::vector<std::pair<int, PCLPoint> >& newHypos) const
{
    std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > result = std::vector<std::tuple<long, PCLPoint, int, int> >();
    std::vector<PCLPoint> newHyposUnpacked;
    newHyposUnpacked.reserve(newHypos.size());

    for(auto hypo : newHypos)
        newHyposUnpacked.push_back(std::get<1>(hypo));

    for(auto tracked : trackedHypos)
    {
        auto ID = std::get<0>(tracked);
        auto hypo = std::get<1>(tracked);        
        auto lifes = std::get<2>(tracked);
        auto detectionCount = std::get<3>(tracked);
        auto partners = pairable(hypo, newHyposUnpacked, mMaxPairingDist*2.0);
        if(!partners.empty())
        {
            for(int i=0; i<detectionCount-1; ++i)
                partners.push_back(hypo);

            result.push_back(std::make_tuple(ID, calcAvg(partners), std::min(100, lifes + mRedetectionConfidenceInc), detectionCount+1));
        }
        else if(lifes > 0) // no redetection
            result.push_back(std::make_tuple(ID, hypo, lifes - mNoRedetectionConfidenceDec, std::max(detectionCount-1, 1)));
    }

    for(auto newHypo : newHypos)
    {
        std::vector<PCLPoint> tracked = std::vector<PCLPoint>(trackedHypos.size());
        std::transform(trackedHypos.begin(), trackedHypos.end(), tracked.begin(), [](std::tuple<long, PCLPoint, int, int>& p){ return std::get<1>(p);});
        if(pairable(std::get<1>(newHypo), tracked, mMaxPairingDist*2.0).empty())
            result.push_back(std::make_tuple(std::hash<unsigned int>{}(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()), std::get<1>(newHypo), std::min(100, mInitialConfidence * (std::get<0>(newHypo)-1)), 1));
    }

    return result;
}


std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > SimplePoleDetector::mergeTrackedWalls(std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > &trackedHypos, const PCLPoint& origin) const
{
    std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > result = std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >();
    std::vector<long> toDel;

    for(auto tracked : trackedHypos)
    {
        long ID = std::get<0>(tracked);
        if(std::find(toDel.begin(), toDel.end(), ID) != toDel.end())
            continue; // skip wall

        auto partnersUnfiltered = findPairableWalls(tracked, trackedHypos, mMaxPairingDist*2.0, mMaxPairingAngle);
        std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > partners;

        for(auto partner : partnersUnfiltered) // filter already merged features
            if(std::find(toDel.begin(), toDel.end(), std::get<0>(partner)) == toDel.end())
                partners.push_back(partner);

        if(!partners.empty())
        {
            std::vector<long> ids = std::vector<long>(partners.size());
            std::transform(partners.begin(), partners.end(), ids.begin(), [](std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int>& p){return std::get<0>(p);});
            if(std::find(mMatchedFeatures.begin(), mMatchedFeatures.end(), ID) == mMatchedFeatures.end()){
                bool partnerMatched = false;
                for(auto partner : partners)
                    if(std::find(mMatchedFeatures.begin(), mMatchedFeatures.end(), std::get<0>(partner)) != mMatchedFeatures.end()){
                        partnerMatched = true;
                        break;
                    }
                if(partnerMatched) continue; // if one of the partners was already matched, use this one for merging instead.
            }
            toDel.insert(toDel.end(), ids.begin(), ids.end());
        }
        if(partners.empty())
            ROS_ERROR("Could not pair wall with itself!");

        std::vector<int> lifes = std::vector<int>(partners.size());
        std::transform(partners.begin(), partners.end(), lifes.begin(), [](std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int>& p){return std::get<2>(p);});
        int newlife = std::min(std::accumulate(lifes.begin(), lifes.end(), 0), 100);
        std::vector<int> detections = std::vector<int>(partners.size());
        std::transform(partners.begin(), partners.end(), detections.begin(), [](std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int>& p){return std::get<3>(p);});
        int newDetections = std::accumulate(detections.begin(), detections.end(), 0);
        std::vector<std::pair<PCLPoint, PCLPoint> > hypoPoints = std::vector<std::pair<PCLPoint, PCLPoint> >(partners.size());
        std::transform(partners.begin(), partners.end(), hypoPoints.begin(), [](std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int>& p){return std::get<1>(p);});
        if(!hypoPoints.empty())
            result.push_back(std::make_tuple(ID, mergeLines(hypoPoints), newlife, newDetections));
    }

    return result;
}


std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > SimplePoleDetector::mergeTracked(std::vector<std::tuple<long, PCLPoint, int, int> > &trackedHypos) const
{
    std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > result = std::vector<std::tuple<long, PCLPoint, int, int> >();
    std::vector<long> toDel;

    for(auto tracked : trackedHypos)
    {
        long ID = std::get<0>(tracked);
        int detectionCount = std::get<3>(tracked);
        if(std::find(toDel.begin(), toDel.end(), ID) != toDel.end())
            continue; // skip pole

        auto partners = hypoPairable(tracked, trackedHypos, mMaxPairingDist);

        if(!partners.empty())
        {
            std::vector<long> ids = std::vector<long>(partners.size());
            std::transform(partners.begin(), partners.end(), ids.begin(), [](std::tuple<long, PCLPoint, int, int>& p){return std::get<0>(p);});
            if(std::find(mMatchedFeatures.begin(), mMatchedFeatures.end(), ID) == mMatchedFeatures.end()){
                bool partnerMatched = false;
                for(auto partner : partners)
                    if(std::find(mMatchedFeatures.begin(), mMatchedFeatures.end(), std::get<0>(partner)) != mMatchedFeatures.end()){
                        partnerMatched = true;
                        break;
                    }
                if(partnerMatched) continue; // if one of the partners was already matched, use this one for merging instead.
            }
                toDel.insert(toDel.begin(), ids.begin(), ids.end());
        }

        partners.push_back(tracked);
        std::vector<PCLPoint> hypoPoints = std::vector<PCLPoint>(partners.size());
        std::transform(partners.begin(), partners.end(), hypoPoints.begin(), [](std::tuple<long, PCLPoint, int, int>& p){return std::get<1>(p);});
        for(int i=0; i<detectionCount-1; ++i) hypoPoints.push_back(std::get<1>(tracked));
        result.push_back(std::make_tuple(ID, calcAvg(hypoPoints), std::get<2>(tracked), detectionCount)); // TODO add partners life
    }

    return result;
}


const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > SimplePoleDetector::hypoPairable(const std::tuple<long, PCLPoint, int, int> hypothesis, const std::vector<std::tuple<long, PCLPoint, int, int> >& otherHypotheses, float maxPairingDist) const
{
    std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > result;
    long id = std::get<0>(hypothesis);
    for(auto otherHypo : otherHypotheses)
        if(std::get<0>(otherHypo) != id && calcDist(std::get<1>(hypothesis), std::get<1>(otherHypo)) < maxPairingDist)
            result.push_back(otherHypo);

    return result;
}


std::vector<SimplePoleDetector::PCLPoint> SimplePoleDetector::pairable(const PCLPoint hypothesis, const std::vector<PCLPoint>& otherHypotheses, float maxPairingDist) const
{
    std::vector<PCLPoint> result;
    for(const PCLPoint otherHypo : otherHypotheses)
        if(calcDist(hypothesis, otherHypo) < maxPairingDist)
            result.push_back(otherHypo);

    return result;
}


bool SimplePoleDetector::linesPairable(const std::pair<PCLPoint, PCLPoint> hypothesis, const std::pair<PCLPoint, PCLPoint>& otherHypothesis, float maxPairingDist, float maxPairingAngle) const
{
    PCLPoint vecHypo = pointsToVector(hypothesis);
    if(calcDist(hypothesis.first, hypothesis.second) >= 50.0) // Don't allow too long walls
        return false;

    PCLPoint otherVec = pointsToVector(otherHypothesis);
    double hypoLength = std::sqrt(std::pow(vecHypo.x, 2.0) + std::pow(vecHypo.y, 2.0));
    double otherLength = std::sqrt(std::pow(otherVec.x, 2.0) + std::pow(otherVec.y, 2.0));    
    const double epsilon = 0.00000001; // acos is undefined for 1.0, so we reduce the value by this epsilon
    float angle = std::acos((vecHypo.x/hypoLength*otherVec.x/otherLength + vecHypo.y/hypoLength*otherVec.y/otherLength) - epsilon);
    if(angle > -maxPairingAngle && angle < maxPairingAngle && std::max(calcLineDist(hypothesis, otherHypothesis.first), calcLineDist(hypothesis, otherHypothesis.second)) < maxPairingDist*2.0){
        bool hypoLeft = hypothesis.first.x <= otherHypothesis.first.x;
        return ((hypoLeft && otherHypothesis.first.x-hypothesis.second.x <= maxPairingDist) || (!hypoLeft && hypothesis.first.x-otherHypothesis.second.x <= maxPairingDist));
    }

    return false;
}


std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > SimplePoleDetector::findPairableLines(const std::pair<PCLPoint, PCLPoint> hypothesis, const std::vector<std::pair<PCLPoint, PCLPoint> >& otherHypotheses, float maxPairingDist, float maxPairingAngle) const
{
    std::vector<std::pair<PCLPoint, PCLPoint> > result;
    for(auto other : otherHypotheses){
        if(linesPairable(hypothesis, other, maxPairingDist, maxPairingAngle))
            result.push_back(other);
    }
    return result;
}


std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > SimplePoleDetector::findPairableWalls(const std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> trackedHypothesis, const std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >& otherTrackedHypos, float maxPairingDist, float maxPairingAngle) const
{
    std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > result;
    std::pair<PCLPoint, PCLPoint> hypothesis = std::get<1>(trackedHypothesis);
    PCLPoint vecHypo = pointsToVector(hypothesis);

    if(calcDist(hypothesis.first, hypothesis.second) >= 50.0){ // Don't allow too long walls
        result.push_back(trackedHypothesis);
        return result;
    }

    for(const auto otherHypo : otherTrackedHypos){
        PCLPoint otherVec = pointsToVector(std::get<1>(otherHypo));
        double hypoLength = std::sqrt(std::pow(vecHypo.x, 2.0) + std::pow(vecHypo.y, 2.0));
        double otherLength = std::sqrt(std::pow(otherVec.x, 2.0) + std::pow(otherVec.y, 2.0));
        const double epsilon = 0.00000001; // acos is undefined for 1.0, so we reduce the value by this epsilon
        float angle = std::acos((vecHypo.x/hypoLength * otherVec.x/otherLength + vecHypo.y/hypoLength * otherVec.y/otherLength) - epsilon);

        if(angle > -maxPairingAngle && angle < maxPairingAngle && std::max(calcLineDist(hypothesis, std::get<1>(otherHypo).first), calcLineDist(hypothesis, std::get<1>(otherHypo).second)) < maxPairingDist){
            auto hypoSorted = sortByX(hypothesis);
            auto otherSorted = sortByX(std::get<1>(otherHypo));
            bool hypoLeft = hypoSorted.first.x <= otherSorted.first.x;            
            if((hypoLeft && otherSorted.first.x-hypoSorted.second.x <= maxPairingDist) || (!hypoLeft && hypoSorted.first.x-otherSorted.second.x <= maxPairingDist))
                result.push_back(otherHypo);
        }
    }

    return result;
}


std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> SimplePoleDetector::sortByX(const std::pair<PCLPoint, PCLPoint>& line) const
{
    if(line.first.x <= line.second.x)
        return std::make_pair(line.first, line.second);
    else
        return std::make_pair(line.second, line.first);
}


SimplePoleDetector::PCLPoint SimplePoleDetector::pointsToVector(const std::pair<PCLPoint, PCLPoint>& points) const
{
    PCLPoint vec;
    if(points.first.x <= points.second.x){
        vec.x = points.second.x-points.first.x;
        vec.y = points.second.y-points.first.y;
    }
    else{
        vec.x = points.first.x-points.second.x;
        vec.y = points.first.y-points.second.y;
    }
    return vec;
}


std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> SimplePoleDetector::mergeLines(const std::vector<std::pair<PCLPoint, PCLPoint> > &lines) const
{
    if(lines.empty())
        ROS_ERROR("Got empty vector in mergeLines!"); // This is fatal!

    if(lines.size() == 1) // Just one line? Nothing to merge...
        return lines.front();

    // Assumptions: lines is not empty and the point pairs are sorted ascending by the x coordinate    
    PCLPoint start=lines.front().first, end=lines.front().second;

    for(auto line : lines){
        if(start.x > line.first.x) start = line.first;
        if(end.x < line.second.x) end = line.second;
    }

    std::pair<PCLPoint, PCLPoint> merged = std::make_pair(start, end);
    return merged;
}


SimplePoleDetector::PCLPoint SimplePoleDetector::transformPclPoint(const PCLPoint &point, const ros::Time &stamp, const std::string sourceFrameId, const geometry_msgs::TransformStamped& transform) const
{
    geometry_msgs::Point gmp;
    gmp.x = point.x;
    gmp.y = point.y;
    gmp.z = point.z;
    return transformPoint(gmp, stamp, sourceFrameId, transform);
}


SimplePoleDetector::PCLPoint SimplePoleDetector::transformPoint(const geometry_msgs::Point &point, const ros::Time& stamp, const std::string sourceFrameId, const geometry_msgs::TransformStamped &transform) const
{
    std_msgs::Header header;
    PCLPoint result;
    header.stamp = stamp;
    header.frame_id = sourceFrameId;

    geometry_msgs::PointStamped psIn, psOut;
    psIn.header = header;
    psIn.point = point;

    tf2::doTransform(psIn, psOut, transform);

    result.x = psOut.point.x;
    result.y = psOut.point.y;
    result.z = psOut.point.z;
    return result;
}


SimplePoleDetector::PCLPoint SimplePoleDetector::map2UTM(const PCLPoint &point, ros::Time stamp) const
{
    std_msgs::Header header;
    PCLPoint result;
    header.stamp = stamp;
    header.frame_id = "map";
    static geometry_msgs::TransformStamped transformStamped; // map2utm is a static transform


    if(transformStamped.header.stamp.sec == 0.0)
    {
        try{
            mTfBuffer.canTransform("utm", "map", stamp, ros::Duration(5.0));
            transformStamped = mTfBuffer.lookupTransform("utm", "map", stamp, ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return point;
        }
    }


    geometry_msgs::PointStamped psIn, psOut;
    psIn.header = header;
    psIn.point.x = point.x;
    psIn.point.y = point.y;
    psIn.point.z = point.z;

    tf2::doTransform(psIn, psOut, transformStamped);

    result.x = psOut.point.x;
    result.y = psOut.point.y;
    result.z = psOut.point.z;

    return result;
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simple_pole_detector::SimplePoleDetector, nodelet::Nodelet);
