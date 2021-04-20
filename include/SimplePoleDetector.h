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

#ifndef SIMPLE_POLE_DETECTOR_H
#define SIMPLE_POLE_DETECTOR_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "velodyne_pointcloud/point_types.h"
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pole_based_localization/PoleMatchArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <pole_based_localization/pole_detectionConfig.h>

#include <iostream>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>


namespace simple_pole_detector {
class SimplePoleDetector : public nodelet::Nodelet{
public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;
    SimplePoleDetector();
    virtual ~SimplePoleDetector();
    virtual void onInit() override;

protected:
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void callbackPos(const nav_msgs::OdometryConstPtr& odom);
    void callbackMatchings(const pole_based_localization::PoleMatchArray& matchings);
    void callbackReconfigure(pole_based_localization::pole_detectionConfig& config, uint32_t level);

    unsigned int detectRingCount(const pcl::PointCloud<PCLPoint>::Ptr cloud) const;
    std::vector<std::vector<SimplePoleDetector::PCLPoint> > genHypotheses(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int>& rings, double minWidth, double maxWidth, double minSpacing) const;
    std::vector<std::vector<SimplePoleDetector::PCLPoint> > findCorners(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int>& rings, double angleTolerance, float minWallLength, float maxInlierDist) const;
    std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > findWalls(const pcl::PointCloud<PCLPoint>::Ptr cloud, const std::vector<unsigned int>& rings, double angleTolerance, float minWallLength, float maxInlierDist) const;
    std::vector<std::pair<int, PCLPoint> > mergeHypotheses(const std::vector<std::vector<PCLPoint> >& hypotheses) const;
    std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > mergeWallHypotheses(const std::vector<std::pair<PCLPoint, PCLPoint> >& hypotheses) const;
    std::vector<SimplePoleDetector::PCLPoint> pairable(const PCLPoint hypothesis, const std::vector<PCLPoint>& otherHypotheses, float maxPairingDist) const;
    bool linesPairable(const std::pair<PCLPoint, PCLPoint> hypothesis, const std::pair<PCLPoint, PCLPoint>& otherHypothesis, float maxPairingDist, float maxPairingAngle) const;
    std::vector<std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> > findPairableLines(const std::pair<PCLPoint, PCLPoint> hypothesis, const std::vector<std::pair<PCLPoint, PCLPoint> > &otherHypotheses, float maxPairingDist, float maxPairingAngle) const;
    std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > findPairableWalls(const std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> trackedHypothesis, const std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > &otherTrackedHypos, float maxPairingDist, float maxPairingAngle) const;
    SimplePoleDetector::PCLPoint pointsToVector(const std::pair<PCLPoint, PCLPoint>& points) const;
    const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > hypoPairable(const std::tuple<long, PCLPoint, int, int> hypothesis, const std::vector<std::tuple<long, PCLPoint, int, int> >& otherHypotheses, float maxPairingDist) const;
    const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> >& trackPoles(const std::vector<std::pair<int, PCLPoint> >& hypotheses) const;
    const std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> >& trackCorners(const std::vector<std::pair<int, PCLPoint> > &hypotheses) const;
    const std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> >& trackWalls(const std::vector<std::pair<PCLPoint, PCLPoint> >& hypotheses, const PCLPoint origin) const;
    std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> > updateWallTracking(const std::vector<std::pair<PCLPoint, PCLPoint>>& newHypos, std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >& trackedHypos) const;
    std::vector<std::tuple<long, PCLPoint, int, int> > updateTracking(std::vector<std::tuple<long, PCLPoint, int, int> >& trackedHypos, const std::vector<std::pair<int, PCLPoint> > &newHypos) const;
    std::vector<std::tuple<long, SimplePoleDetector::PCLPoint, int, int> > mergeTracked(std::vector<std::tuple<long, PCLPoint, int, int> > &trackedHypos) const;
    std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> ransacLineFit(std::vector<SimplePoleDetector::PCLPoint> pointsToFit) const;
    void sendVisualizationMarker(ros::Time timestamp, std::basic_string<char> frame_id, const std::vector<std::tuple<long, PCLPoint, int, int> >& trackedPoles) const;
    void logPolesToCsv(const std::vector<std::tuple<long, PCLPoint, int, int> >& trackedPoles);
    std::vector<std::tuple<long, std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>, int, int> > mergeTrackedWalls(std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >& trackedHypos, const PCLPoint& origin) const;
    void sendWallVizMarker(ros::Time timestamp, std::basic_string<char> frame_id, const std::vector<std::tuple<long, std::pair<PCLPoint, PCLPoint>, int, int> >& trackedPoles) const;
    SimplePoleDetector::PCLPoint map2UTM(const PCLPoint &point, ros::Time stamp) const;
    SimplePoleDetector::PCLPoint transformPclPoint(const PCLPoint &point, const ros::Time &stamp, const std::string sourceFrameId, const geometry_msgs::TransformStamped& transform) const;
    SimplePoleDetector::PCLPoint transformPoint(const geometry_msgs::Point &point, const ros::Time& stamp, const std::string sourceFrameId, const geometry_msgs::TransformStamped &transform) const;


    double calcDist(const PCLPoint point1, const PCLPoint point2) const;
    double calcDist(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) const;
    double calcAngle(const std::pair<PCLPoint, PCLPoint>& direction) const;
    double calcLineDist(const std::vector<SimplePoleDetector::PCLPoint>& linePoints, const SimplePoleDetector::PCLPoint& point) const;
    double calcLineDist(const std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint>& linePoints, const SimplePoleDetector::PCLPoint& point) const;
    double calcLineAngle(const std::vector<SimplePoleDetector::PCLPoint>& linePoints1, const std::vector<SimplePoleDetector::PCLPoint>& linePoints2) const;
    double calcLineAngle(const std::pair<PCLPoint, PCLPoint>& linePoints1, const std::pair<PCLPoint, PCLPoint>& linePoints2) const;
    SimplePoleDetector::PCLPoint getClosestPointOnLine(const std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> &line, const SimplePoleDetector::PCLPoint& point) const;
    SimplePoleDetector::PCLPoint calcAvg(const std::vector<PCLPoint> points) const;
    std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> sortByX(const std::pair<PCLPoint, PCLPoint>& line) const;
    std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> mergeLines(const std::vector<std::pair<PCLPoint, PCLPoint> >& lines) const;
    std::pair<SimplePoleDetector::PCLPoint, SimplePoleDetector::PCLPoint> cutLineToSphere(const std::pair<PCLPoint, PCLPoint> line, const PCLPoint center, double radius) const;

    ros::Subscriber mSubscriber, mLastPosSubscriber;
    ros::Publisher mPolePublisher, mMarkerPublisher, mPoleArrayPublisher;

    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<pole_based_localization::pole_detectionConfig> > mConfigServer;
    pole_based_localization::pole_detectionConfig mConfig;

    nav_msgs::Odometry mLastPos;

    std::vector<long> mMatchedFeatures;
    std::ofstream mCSVFile;

    bool mTrackingEnabled, mDetectCorners, mDetectWalls, mLogFeatures;
    const float mMaxPairingDist, mMaxPairingAngle;
    int mInitialConfidence = 50, mRedetectionConfidenceInc = 50, mNoRedetectionConfidenceDec = 2;
};
}

#endif

