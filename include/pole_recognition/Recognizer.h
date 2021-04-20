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


#ifndef POLE_RECOGNIZER_H
#define POLE_RECOGNIZER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <pole_based_localization/pole_recognitionConfig.h>

#include <pole_based_localization/Pole.h>
#include <pole_based_localization/PoleArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <fstream>

#include <sqlite3.h>

namespace pole_recognition {
class Recognizer : public nodelet::Nodelet{
public:
    typedef std::map<long, std::map<long, std::pair<double, double> > > FingerprintMap;
    Recognizer();
    virtual ~Recognizer();
    virtual void onInit() override;

protected:
    void cleanPolesNPrints(std::map<long, pole_based_localization::Pole> &poleMap, FingerprintMap &fingerprints) const;
    void updateFingerprints(const pole_based_localization::PoleArray &poles, FingerprintMap &fingerprints, std::map<long, pole_based_localization::Pole> &poleMap) const;
    void callback(const pole_based_localization::PoleArray& poles);
    void callbackPosition(const nav_msgs::Odometry& odom);
    void callbackReconfigure(pole_based_localization::pole_recognitionConfig& config, uint32_t level);

    void visualizeFingerprints(const FingerprintMap &fingerprints, const std::map<long, pole_based_localization::Pole>& poleMap, bool rainbow=true) const;
    void visualizeMatchedFingerprints(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > & matched, const FingerprintMap& globalFp, const std::map<long, pole_based_localization::Pole>& poleMap) const;

    std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > associate(const FingerprintMap &fingerprints, const std::map<long, pole_based_localization::Pole> &poleMap) const;
    std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > > dbAssociation(const std::vector<long>::const_iterator fpbegin, const std::vector<long>::const_iterator fpend, const FingerprintMap& fpMap, const std::map<long, pole_based_localization::Pole> &poleMap, sqlite3* db, sqlite3_stmt* poleStmt, sqlite3_stmt* wallStmt) const;
    double calcDist(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2) const;
    double calcDist(const pole_based_localization::Pole &pole1, const pole_based_localization::Pole &pole2) const;
    double calcDistSquared(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2) const;
    double calcRangeDistSquared(const pole_based_localization::Pole &feat1, const pole_based_localization::Pole &feat2) const;
    double calcLineDist(const pole_based_localization::Pole &line, const geometry_msgs::Point & point) const;
    double calcAngle(const pole_based_localization::Pole &pole1, const pole_based_localization::Pole &pole2) const;
    double calcAngle(const tf2::Vector3& direction) const;
    double calcPointEdgeAngle(const pole_based_localization::Pole &pole1, const pole_based_localization::Pole &pole2) const;
    std::pair<geometry_msgs::Point, geometry_msgs::Point> edgeToPoints(const pole_based_localization::Pole& edge) const;
    geometry_msgs::Point getClosestPointOnLine(const pole_based_localization::Pole &line, const geometry_msgs::Point & point) const;
    std::vector<long> getPolesInRange(const pole_based_localization::Pole &origin, const std::map<long, pole_based_localization::Pole>& poleMap, double range) const;
    geometry_msgs::Point map2UTM(const geometry_msgs::Point &point, ros::Time stamp) const;
    geometry_msgs::Point UTM2map(const geometry_msgs::Point &point, ros::Time stamp) const;
    bool isEdgeType(const pole_based_localization::Pole &feature) const;

    std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > selectBestMatchings(const std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > > &allMatches) const;
    std::vector<std::pair<double, std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > > > genPopulation(const std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > > &allMatches, const short popSize) const;
    double cost(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> >& pop) const;
    void adaptOldBest(std::pair<double, std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > >& oldBest, const std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > >& allMatches) const;
    void permutate(std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > &matching, float prob, const std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > >& allMatches) const;
    std::array<double, 3> calcVariances(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> >& matchings) const;
    geometry_msgs::Pose calculateOffset(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> >& associations) const;
    std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > rejectOutliers(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > associations) const;

    struct {
        bool operator()(const std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double>& i, const std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double>& j) const
        {
            // squared x,y dist is enough...
            double iDist = std::pow(std::get<1>(i)-std::get<4>(i).pose.position.x, 2.0) + std::pow(std::get<2>(i)-std::get<4>(i).pose.position.y, 2.0);
            double jDist = std::pow(std::get<1>(j)-std::get<4>(j).pose.position.x, 2.0) + std::pow(std::get<2>(j)-std::get<4>(j).pose.position.y, 2.0);
            unsigned int distCounti = std::get<5>(i), distCountj = std::get<5>(j);
            // if dist count equal sort by distance (close to far)
            if(distCounti == distCountj)
                return iDist < jDist;

            return std::get<5>(i) > std::get<5>(j);
        }
    } distCompare;

    struct {
        bool operator()(const std::pair<double, std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > >& i, const std::pair<double, std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > >& j) const
        {
            // just extract the cost and compare
            return i.first < j.first;
        }
    } costCompare;


    // ros stuff
    ros::Subscriber mSubscriber, mPosSubscriber;
    ros::Publisher mPositionDifferencePublisher, mPoleMatchingPublisher, mFingerprintsMarkerPublisher;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<pole_based_localization::pole_recognitionConfig> > mConfigServer;
    pole_based_localization::pole_recognitionConfig mConfig;

    static const unsigned short mThreadCount = 8, mMaxDists = 125;
    float mMinAngleTolerance = 0.01f;
    float mMinDistTolerance = 0.01f;
    bool mDetectWalls = true;
    float mMaxDistTolerance = 0.05f, mMaxDistAbsTolerance = 0.8f, mMinDistAbsTolerance = 0.2;
    float mMaxAngleTolerance = 0.02f;
    float mMaxWallAngleTolerance = 0.02f;
    float mMaxSearchRadius = 2.5f;
    double mDatabaseMinX = 0.0, mDatabaseMaxX = 0.0, mDatabaseMinY = 0.0, mDatabaseMaxY = 0.0;
    bool mInDatabaseBounds = false, mHasMatched = false;
    geometry_msgs::Point mLastPos, mLastUtmPos;
    geometry_msgs::Vector3 mLastSpeed;
    std::string mDatabaseName = "pole_based_localization/sqlite/spatia_poles.sqlite";
    sqlite3* mDatabase[mThreadCount];
    sqlite3_stmt* mStatements[mThreadCount], *mWallStatements[mThreadCount], *mDebugStmt, *mDebugWallStmt;
    std::ofstream mCSVFile;
};
}

#endif
