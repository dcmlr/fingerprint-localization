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

#include "pole_recognition/Recognizer.h"

#include <cmath>
#include <chrono>
#include <stdio.h>
#include <random>
#include <algorithm>
#include <numeric>
#include <string>
#include <thread>
#include <future>

#include <spatialite.h>

#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pole_based_localization/PoleMatch.h>
#include <pole_based_localization/PoleMatchArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/registration/transformation_estimation_svd.h>

namespace pole_recognition {

using namespace pole_based_localization;

Recognizer::Recognizer() : mTfListener(mTfBuffer)
{}

Recognizer::~Recognizer()
{
    for(auto stmt : mStatements)
        sqlite3_finalize(stmt);
    for(auto stmt : mWallStatements)
        sqlite3_finalize(stmt);
    for(auto db : mDatabase)
        sqlite3_close(db);

    mCSVFile.close();
}


static int tracer(unsigned, void*, void *p, void*)
{
    sqlite3_stmt *stmt = (sqlite3_stmt*)p;
    char *sql = sqlite3_expanded_sql(stmt);
    ROS_DEBUG_STREAM("sql: " << sql);
    sqlite3_free(sql);
    return 0;
}

void Recognizer::onInit()
{
    ros::NodeHandle nh = getNodeHandle();
    std::string topic = nh.resolveName("/pole_recognition/trackedPolesArray");    
    dynamic_reconfigure::Server<pole_based_localization::pole_recognitionConfig>::CallbackType config_callback;

    mPositionDifferencePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pole_recognition/PositionDifference",1);
    mPoleMatchingPublisher = nh.advertise<PoleMatchArray>("/pole_recognition/PoleMatchings",1);
    mFingerprintsMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/pole_recognition/FingerprintsMarkerArray",1);

    mConfigServer =
        boost::make_shared<dynamic_reconfigure::Server<pole_based_localization::pole_recognitionConfig> >(getPrivateNodeHandle());
    config_callback = boost::bind(&Recognizer::callbackReconfigure, this, _1, _2);
    mConfigServer->setCallback(config_callback);

    nh.getParam("/pole_recognition/max_search_radius", mMaxSearchRadius);
    nh.getParam("/pole_recognition/max_dist_tolerance", mMaxDistTolerance);
    nh.getParam("/pole_recognition/max_angle_tolerance", mMaxAngleTolerance);
    nh.getParam("/pole_recognition/database_name", mDatabaseName);
    nh.getParam("/pole_recognition/detect_walls", mDetectWalls);

    ROS_INFO_STREAM("Using database " << mDatabaseName);

    sqlite3_config(SQLITE_CONFIG_MULTITHREAD);

    for(int i=0; i<mThreadCount; ++i){
        sqlite3* db;
        // Open database connection
        if(sqlite3_open_v2(mDatabaseName.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL))
            ROS_ERROR_STREAM("Can't open database: " << mDatabaseName);

        // Open Spatialite extension
        sqlite3_enable_load_extension(db, 1);
        if(sqlite3_exec(db, "SELECT load_extension('mod_spatialite.so')", NULL, NULL, NULL))
            ROS_ERROR_STREAM("Can't open database: " << sqlite3_errmsg(db));

        sqlite3_exec(db, "PRAGMA TEMP_STORE = MEMORY", NULL, NULL, NULL);
        sqlite3_exec(db, "PRAGMA query_only = TRUE", NULL, NULL, NULL);

        mDatabase[i] = db;

        // Retrieve database extends
        sqlite3_stmt *stmt = NULL;
        sqlite3_prepare_v2(db, "SELECT MIN(x), MIN(y), MAX(x), MAX(y) FROM fingerprints", -1, &stmt, NULL);        
        if(sqlite3_step(stmt) == SQLITE_ROW)
        {
            mDatabaseMinX = sqlite3_column_double(stmt, 0);
            mDatabaseMinY = sqlite3_column_double(stmt, 1);
            mDatabaseMaxX = sqlite3_column_double(stmt, 2);
            mDatabaseMaxY = sqlite3_column_double(stmt, 3);
        }
        else
            ROS_ERROR_STREAM("Could not get database dimensions, falling back to GPS.");

        sqlite3_finalize(stmt);

        // Build prepared statements
        std::ostringstream query, wallquery, debug_query, debug_wall_query;
        query << "SELECT count(poleID) as c, poleID, type as why, x, y, angle FROM (SELECT poleID, dist, angle, type, x, y FROM fingerprints WHERE fingerprints.ROWID IN (SELECT ROWID FROM SpatialIndex WHERE f_table_name='fingerprints' AND search_frame=BuildMbr(?,?,?,?,32633)) AND type = ? AND ((dist BETWEEN ? AND ?) AND (angle BETWEEN ? AND ?)";
        wallquery << "SELECT count(id) as c, id, type as why, x1, y1, angle, x2, y2 FROM (SELECT id, dist, angle, type, x1, y1, x2, y2 FROM fingerprintsWalls WHERE fingerprintsWalls.ROWID IN (SELECT ROWID FROM SpatialIndex WHERE f_table_name='fingerprintsWalls' AND search_frame=BuildMbr(?,?,?,?,32633)) AND type = ? AND ((dist BETWEEN ? AND ? AND angle BETWEEN ? AND ? AND (oX BETWEEN ? AND ? AND oY BETWEEN ? AND ?))";
        debug_query << "SELECT otherId,dist,angle FROM fingerprints where poleId = ? AND ((dist BETWEEN ? AND ? AND angle BETWEEN ? AND ?)";
        debug_wall_query << "SELECT otherId,dist,angle,oX,oY FROM fingerprintsWalls where id = ? AND ((dist BETWEEN ? AND ? AND angle BETWEEN ? AND ? AND oX BETWEEN ? AND ? AND oY BETWEEN ? AND ?)";

        for(int i=1; i < mMaxDists; ++i){
            query << " OR (dist BETWEEN ? AND ? AND angle BETWEEN ? AND ?)";
            wallquery << " OR (dist BETWEEN ? AND ? AND angle BETWEEN ? AND ? AND oX BETWEEN ? AND ? AND oY BETWEEN ? AND ?)";
            debug_query << " OR (dist BETWEEN ? AND ? AND angle BETWEEN ? AND ?)";
            debug_wall_query << " OR (dist BETWEEN ? AND ? AND angle BETWEEN ? AND ? AND oX BETWEEN ? AND ? AND oY BETWEEN ? AND ?)";
       }

        query << ") GROUP BY poleID, dist, angle, x, y, type) as t1 GROUP BY poleID, type, x, y HAVING count(poleID) > 3 ORDER BY c DESC LIMIT 10;";
        wallquery << ") GROUP BY id, dist, angle, x1, y1, x2, y2, type) as t1 GROUP BY id, type, x1, y1, x2, y2 HAVING count(id) > 3 ORDER BY c DESC LIMIT 10;";
        debug_query << ")";
        debug_wall_query << ")";

        sqlite3_stmt* poleStmt = NULL, *wallStmt = NULL;
        if((sqlite3_prepare_v3(db, query.str().c_str(), -1, SQLITE_PREPARE_PERSISTENT, &poleStmt, 0)))
            ROS_ERROR_STREAM("ERROR while preparing statment: " << sqlite3_errmsg(db));

        if(mDetectWalls && (sqlite3_prepare_v3(db, wallquery.str().c_str(), -1, SQLITE_PREPARE_PERSISTENT, &wallStmt, 0))){            
            mDetectWalls = false;
            ROS_ERROR_STREAM("ERROR while preparing statment: " << sqlite3_errmsg(db));
        }

        if(i==0 && (sqlite3_prepare_v3(db, debug_query.str().c_str(), -1, SQLITE_PREPARE_PERSISTENT, &mDebugStmt, 0)))
            ROS_ERROR_STREAM("ERROR while preparing statment: " << sqlite3_errmsg(db));

        if(i==0 && (sqlite3_prepare_v3(db, debug_wall_query.str().c_str(), -1, SQLITE_PREPARE_PERSISTENT, &mDebugWallStmt, 0)))
            ROS_ERROR_STREAM("ERROR while preparing statment: " << sqlite3_errmsg(db));

        mStatements[i] = poleStmt;
        mWallStatements[i] = wallStmt;
    }


    mCSVFile.open("pole_recognizer_debug.csv");
    mCSVFile << "timestamp," << "lastPosX," << "lastPosY," << "searchRadius," << "matchingCount," << "diffx," << "diffy," << "diffyaw," << "varX," << "varY," << "varYaw" << std::endl;

    mSubscriber = nh.subscribe(topic, 1, &Recognizer::callback, this);
    mPosSubscriber = nh.subscribe(nh.resolveName("/localization/odometry/filtered_map"), 1, &Recognizer::callbackPosition, this);

    // Debugging
    //sqlite3_trace_v2(mDatabase[0], SQLITE_TRACE_STMT, tracer, NULL);
}


void Recognizer::callbackPosition(const nav_msgs::Odometry &odom)
{    
    mLastPos = odom.pose.pose.position;
    mLastSpeed = odom.twist.twist.linear;
    mLastUtmPos = map2UTM(odom.pose.pose.position, odom.header.stamp);

    if(mHasMatched && mInDatabaseBounds != (mDatabaseMinX < mLastUtmPos.x && mDatabaseMinY < mLastUtmPos.y && mDatabaseMaxX > mLastUtmPos.x && mDatabaseMaxY > mLastUtmPos.y))
    {
        mInDatabaseBounds = !mInDatabaseBounds;
        getNodeHandle().setParam("/pole_recognition/position_correction/use_gps", !mInDatabaseBounds); // Switch GPS
    }
}

void Recognizer::callback(const PoleArray &poles)
{
    static FingerprintMap fingerprints;
    static std::map<long, Pole> poleMap;
    auto start = std::chrono::system_clock::now();
    updateFingerprints(poles, fingerprints, poleMap);
    auto afterPrints = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = afterPrints-start;    
    auto matchings = associate(fingerprints, poleMap);

    auto endDetection = std::chrono::system_clock::now();
    elapsed_seconds = endDetection-afterPrints;
    //ROS_DEBUG_STREAM("Found " << matchings.size() << " matchings in " << elapsed_seconds.count() * 1000 << "ms\n");

    geometry_msgs::Pose offset;

    if(matchings.size() >= 1){
       offset = calculateOffset(matchings);
       mHasMatched = true;
    }
    //ROS_DEBUG_STREAM("Position difference is: (" << offset.position.x << ", " << offset.position.y << ")");
    auto var = calcVariances(matchings);

    mCSVFile << std::fixed << ros::Time::now().toSec() << "," << std::fixed << mLastUtmPos.x << "," << std::fixed << mLastUtmPos.y << "," << std::fixed << mMaxSearchRadius << "," << matchings.size() << "," << std::fixed << offset.position.x << "," << std::fixed << offset.position.y << "," << std::fixed << offset.orientation.z << "," << std::fixed << var[0] << "," << std::fixed << var[1] << "," << std::fixed << var[2] << std::endl;

    // adjust search radius
    if(mInDatabaseBounds && std::pow(mLastSpeed.x, 2.0) + std::pow(mLastSpeed.y, 2.0) > 1.0)
    {
        if(offset.position.x == 0.0 || offset.position.y == 0.0 || matchings.size() < 4){
            mMaxSearchRadius += 0.01;
            mMaxAngleTolerance += 0.001;
            mMaxDistTolerance += 0.001;
        }
        else if((std::pow(offset.position.x*2, 2.0) + std::pow(offset.position.y*2, 2.0)) < std::pow(mMaxSearchRadius, 2.0)){
            mMaxSearchRadius = std::max(mMaxSearchRadius - 0.02, 1.0);
            mMaxAngleTolerance = std::max(mMaxAngleTolerance - 0.002f, mMinAngleTolerance);
            mMaxDistTolerance = std::max(mMaxDistTolerance - 0.002f, mMinDistTolerance);
        }
    }

    // Publish position difference
    if(mPositionDifferencePublisher.getNumSubscribers())
    {
        geometry_msgs::PoseWithCovarianceStamped ps;
        boost::array<double, 36> cov;
        for(int i=0; i<36; ++i) cov[i] = 0.0;
        cov[0] = var[0];
        cov[7] = var[1];
        cov[35] = var[2];
        ps.header.stamp = poles.header.stamp;
        ps.header.frame_id = "map";
        ps.pose.pose = offset;
        ps.pose.covariance = cov;
        mPositionDifferencePublisher.publish(ps);
    }    

    // Publish pole match array
    if(mPoleMatchingPublisher.getNumSubscribers())
    {
        PoleMatchArray matchArray;
        matchArray.header.frame_id = "map";
        matchArray.header.stamp = ros::Time::now();
        for(auto match : matchings)
        {
            auto now = ros::Time::now();
            PoleMatch poleMatch;
            poleMatch.header.frame_id = "utm";
            poleMatch.header.stamp = poles.header.stamp;
            poleMatch.local = match.first;
            poleMatch.remote = match.second;
            poleMatch.local.header.frame_id = "utm";
            poleMatch.local.header.stamp = now;
            poleMatch.remote.header.frame_id = "utm";
            poleMatch.remote.header.stamp = now;
            matchArray.matches.push_back(poleMatch);
        }
        mPoleMatchingPublisher.publish(matchArray);
    }

    // Publish fingerprints marker array
    if(mFingerprintsMarkerPublisher.getNumSubscribers())
        visualizeFingerprints(fingerprints, poleMap, false);

    //ROS_DEBUG_STREAM("fp association of " << matchings.size() << " took " << es)
}


void Recognizer::visualizeMatchedFingerprints(const std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > &matched, const FingerprintMap &globalFp,  const std::map<long, Pole>& poleMap) const
{
    FingerprintMap localFp, localWallFp;
    std::map<long, Pole> wallPoleMap; // Create a special pole map for wall prints to fill it with the correct positions
    sqlite3_stmt* stmt;
    int id = 0, wallPoleId = 0;
    float angleTolerance = mMaxAngleTolerance;

    for(auto match : matched){
        bool isEdge = isEdgeType(match.first);
        if(isEdge){
            stmt = mDebugWallStmt;
            angleTolerance = mMaxWallAngleTolerance;
        }
        else{
            stmt = mDebugStmt;
        }

        sqlite3_bind_text(stmt, 1, match.second.ID.c_str(), -1, NULL);

        int i=1;
        auto dists = globalFp.at(std::stol(match.first.ID));

        for(auto it = dists.begin(); it != dists.end(); ++it)
        {
            sqlite3_bind_double(stmt, ++i, it->second.first - it->second.first*mMaxDistTolerance);
            sqlite3_bind_double(stmt, ++i, it->second.first + it->second.first*mMaxDistTolerance);
            double minAngle = it->second.second - std::abs(it->second.second*angleTolerance), maxAngle = it->second.second + std::abs(it->second.second*angleTolerance);
            if(minAngle < -M_PI) minAngle += 2*M_PI;
            if(maxAngle > M_PI) maxAngle -= 2*M_PI;
            sqlite3_bind_double(stmt, ++i, minAngle);
            sqlite3_bind_double(stmt, ++i, maxAngle);

            if(isEdge){
                const Pole& other = poleMap.at(it->first);
                auto otherCoords = map2UTM(other.pose.position, other.header.stamp);
                double oX = isEdgeType(other) ? 0.0 : otherCoords.x;
                double oY = isEdgeType(other) ? 0.0 : otherCoords.y;
                sqlite3_bind_double(stmt, ++i, oX - mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oX + mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oY - mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oY + mMaxSearchRadius);
            }
        }

        ++i;
        int paramsPerPrint = isEdge ? 8 : 4;
        for(; i<1+mMaxDists*paramsPerPrint; ++i){
            sqlite3_bind_double(stmt, i, -5.0);
        }

        double x = 0.0, y = 0.0;
        std::map<long, std::pair<double, double> > prints;
        while(sqlite3_step(stmt) == SQLITE_ROW){
            if(isEdge){
                x = sqlite3_column_double(stmt, 3);
                y = sqlite3_column_double(stmt, 4);
                if(x == 0.0 && y == 0.0) continue;
                Pole wallPole = poleMap.at(std::stol(match.first.ID));
                prints[id++] = std::make_pair(sqlite3_column_double(stmt, 1), sqlite3_column_double(stmt, 2));
                wallPole.ID = wallPoleId;
                wallPole.header.frame_id = "map";
                wallPole.pose.position.x = x;
                wallPole.pose.position.y = y;
                wallPole.pose.position = UTM2map(wallPole.pose.position, ros::Time::now());
                localWallFp[wallPoleId] = prints;
                wallPoleMap[wallPoleId] = wallPole;
                wallPoleId++;
                prints.clear();
            }
            else{
                prints[id++] = std::make_pair(sqlite3_column_double(stmt, 1), sqlite3_column_double(stmt, 2));
            }
        }

        if(!isEdge)
            localFp[std::stol(match.first.ID)] = prints;
        // reset statement
        sqlite3_clear_bindings(stmt);
        sqlite3_reset(stmt);
    }

    visualizeFingerprints(localFp, poleMap);
    visualizeFingerprints(localWallFp, wallPoleMap);
}


void Recognizer::visualizeFingerprints(const FingerprintMap &fingerprints, const std::map<long, Pole>& poleMap, bool rainbow) const
{
    visualization_msgs::MarkerArray markerArray;
    for(auto entry : fingerprints)
    {
        visualization_msgs::Marker marker;

        Pole pole = poleMap.at(entry.first);        
        pole.pose.position.z = 2.0f;

        std_msgs::ColorRGBA c;
        c.r = 1.0f;
        c.a = 1.0f;

        /*marker.header.frame_id = pole.header.frame_id;
        marker.header.stamp = pole.header.stamp;
        marker.ns = !rainbow ? "current" : "matched";
        marker.id = entry.first;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = pole.pose.position;
        marker.color = c;
        marker.scale.x = .4f;
        marker.scale.y = .4f;
        marker.scale.z = 4.0f;
        marker.lifetime = ros::Duration(2.0);*/

        //markerArray.markers.push_back(marker);

        visualization_msgs::Marker printMarker;
        printMarker.header.frame_id = pole.header.frame_id;
        printMarker.header.stamp = pole.header.stamp;
        printMarker.type = printMarker.LINE_LIST;
        printMarker.action = printMarker.ADD;
        printMarker.scale.x = 0.05;
        //printMarker.scale.y = 0.05;
        //printMarker.scale.z = 0.05;
        printMarker.ns = !rainbow ? "current" : "matched";
        printMarker.id = entry.first;
        printMarker.lifetime = ros::Duration(2.0);        
        printMarker.pose.orientation.w = 1.0;

        bool edgeThis = isEdgeType(pole);

        for(auto prints : entry.second)
        {
            Pole otherPole = pole;
            bool edgeOther = isEdgeType(otherPole);
            bool isInPoleMap = poleMap.find(prints.first) != poleMap.end();
            double angle = prints.second.second;
            double dist = prints.second.first;
            double sin = std::sin(-angle);
            double cos = std::cos(-angle);

            if(isInPoleMap){
                otherPole = poleMap.at(prints.first);
                edgeOther = isEdgeType(otherPole);
                if(edgeThis)
                    pole = otherPole; //HACK: since we do not store the closest point to a pole on a wall, we draw the fingerprint from the pole to the wall and not the other around
            }

            otherPole.pose.position.z = rainbow ? 2.0 : 0.0;

            if(edgeThis){
                otherPole.pose.position.x = otherPole.pose.position.x - (dist * -sin);
                otherPole.pose.position.y = otherPole.pose.position.y - (dist * cos);
                // Switch the direction
                Pole third = pole;
                pole = otherPole;
                otherPole = third;
            }
            else if(!isInPoleMap){
                otherPole.pose.position.x = pole.pose.position.x - (dist * sin);
                otherPole.pose.position.y = pole.pose.position.y - (dist * -cos);
            }
            else if(edgeOther){
                otherPole.pose.position.x = pole.pose.position.x - (dist * sin);
                otherPole.pose.position.y = pole.pose.position.y - (dist * -cos);
            }

            std_msgs::ColorRGBA pc;

            angle += M_PI;
            angle *= 243.5;
            angle = std::floor(angle);

            pc.a = rainbow ? 0.8 : 0.15;

            if(rainbow){
                switch((int)angle/255){
                case 0:
                    pc.b = 1.0;
                    pc.r = ((int)angle % 255) / 255.0;
                    pc.g = 0.0;
                    break;
                case 1:
                    pc.r = 1.0;
                    pc.b = 1.0 - ((int)angle % 255) / 255.0;
                    pc.g = 0.0;
                    break;
                case 2:
                    pc.r = 1.0;
                    pc.g = ((int)angle % 255) / 255.0;
                    pc.b = 0.0;
                    break;
                case 3:
                    pc.g = 1.0;
                    pc.r = 1.0 - ((int)angle % 255) / 255.0;
                    pc.b = 0.0;
                    break;
                case 4:
                    pc.g = 1.0;
                    pc.b = ((int)angle % 255) / 255.0;
                    pc.r = 0.0;
                    break;
                case 5:
                    pc.b = 1.0;
                    pc.g = 1.0 - ((int)angle % 255) / 255.0;
                    pc.r = 0.0;
                    break;

                default:
                    pc.b = 0.0;
                    pc.g = 0.0;
                    pc.r = 0.0;
                }
            }
            else{
                pc.r = 0.8;
                pc.g = 0.8;
                pc.b = 0.8;
            }

            printMarker.colors.push_back(pc);
            pc.a = 0.0;
            printMarker.colors.push_back(pc);
            printMarker.points.push_back(pole.pose.position);
            printMarker.points.push_back(otherPole.pose.position);
        }

        markerArray.markers.push_back(printMarker);
    }

    mFingerprintsMarkerPublisher.publish(markerArray);
}


void Recognizer::updateFingerprints(const PoleArray &poles, FingerprintMap& fingerprints, std::map<long, Pole>& poleMap) const
{       
    // add new poles
    for(Pole pole : poles.poles){
        if((pole.life >= 60 || poleMap.find(std::stol(pole.ID)) != poleMap.end()))
            poleMap[std::stol(pole.ID)] = pole;
    }

    // remove old poles and prints
    cleanPolesNPrints(poleMap, fingerprints);

    // update fingerprints for all affected poles
    for(Pole pole : poles.poles)
    {
        if(pole.life <= 60 || calcDistSquared(pole.pose.position, mLastPos) > 10000.0)
            continue;


        std::vector<long> polesInRange = getPolesInRange(pole, poleMap, 100.0);
        for(long otherID : polesInRange)
        {
            if(otherID == std::stol(pole.ID)) continue; // Don't fingerprint self

            Pole other = poleMap.at(otherID);
            double dist = calcDist(pole, other);
            double angle = calcAngle(pole, other);
            double otherAngle = calcAngle(other, pole);

            fingerprints[std::stol(pole.ID)][otherID] = std::make_pair(dist, angle);
            fingerprints[otherID][std::stol(pole.ID)] = std::make_pair(dist, otherAngle);
        }
    }
}


void Recognizer::cleanPolesNPrints(std::map<long, Pole> &poleMap, FingerprintMap &fingerprints) const
{
    std::vector<long> toDelete;

    for(auto entry : poleMap)
    {
        Pole pole = entry.second;

        if(pole.life <= 0 || calcDistSquared(pole.pose.position, mLastPos) > 16900.0 || ros::Time::now() - pole.header.stamp > ros::Duration(15))
        {
            if(calcDistSquared(pole.pose.position, mLastPos) > 90000.0)
                toDelete.push_back(entry.first);
            fingerprints.erase(entry.first);
        }
    }

    for(long ID : toDelete)
        poleMap.erase(ID);
}


std::vector<long> Recognizer::getPolesInRange(const Pole &origin, const std::map<long, Pole> &poleMap, double range) const
{
    std::vector<long> result;
    for(auto entry : poleMap){
        Pole feat = entry.second;
        if(calcRangeDistSquared(origin, feat) < (range*range))
            result.push_back(entry.first);
    }

    return result;
}


double Recognizer::calcAngle(const Pole& pole1, const Pole& pole2) const
{
    if(isEdgeType(pole1) && isEdgeType(pole2)) return pole1.pose.orientation.z; // for walls just use their own angle


    geometry_msgs::Point other, p1UTM = pole1.pose.position, p2UTM = pole2.pose.position;
    if(pole1.header.frame_id == "map")
        p1UTM = map2UTM(pole1.pose.position, pole1.header.stamp);
    if(pole2.header.frame_id == "map")
        p2UTM = map2UTM(pole2.pose.position, pole2.header.stamp);
    other.x = p2UTM.x - p1UTM.x;
    other.y = p2UTM.y - p1UTM.y;

    Pole pole1UTM = pole1, pole2UTM = pole2;
    pole1UTM.pose.position = p1UTM;
    pole2UTM.pose.position = p2UTM;

    if(isEdgeType(pole1) || isEdgeType(pole2))
        return calcPointEdgeAngle(pole1UTM, pole2UTM);

    double result = std::acos(other.y/(std::sqrt(std::pow(other.x, 2.0) + std::pow(other.y, 2.0))));
    if(other.x < 0.0) result *= -1.0;
    return result;
}


double Recognizer::calcAngle(const tf2::Vector3& direction) const{
    auto adjustedDir = direction.x() < 0.0 ? tf2::Vector3(direction.x()*-1.0,direction.y()*-1.0,direction.z()*-1.0) : direction;
    return std::acos(adjustedDir.y()/std::hypot(adjustedDir.x(), adjustedDir.y()));
}


double Recognizer::calcPointEdgeAngle(const pole_based_localization::Pole& pole1, const pole_based_localization::Pole& pole2) const
{
    const Pole& line = isEdgeType(pole1) ? pole1 : pole2;
    const Pole& point= isEdgeType(pole1) ? pole2 : pole1;

    auto linePoints = edgeToPoints(line);
    tf2::Vector3 lineVec(linePoints.second.x - linePoints.first.x, linePoints.second.y - linePoints.first.y, 0.0);
    tf2::Vector3 pointVec(point.pose.position.x - linePoints.first.x, point.pose.position.y - linePoints.first.y, 0.0);

    lineVec.normalize();

    auto proj = pointVec.dot(lineVec);
    tf2::Vector3 v_proj(proj * lineVec.x() + linePoints.first.x, proj * lineVec.y() + linePoints.first.y, 0.0);
    tf2::Vector3 rej(v_proj.x() - point.pose.position.x, v_proj.y() - point.pose.position.y, 0.0);

    double angle = std::acos(rej.y()/rej.length());
    if(!isEdgeType(pole2)){ // Check
        angle -= M_PI;
        angle = angle < -M_PI ? angle + 2*M_PI : angle;
    }

    return rej.x() < 0 ? angle * -1 : angle;
}


std::pair<geometry_msgs::Point, geometry_msgs::Point> Recognizer::edgeToPoints(const pole_based_localization::Pole& edge) const
{
    geometry_msgs::Point p1 = edge.pose.position, p2;

    p2.x = p1.x + (edge.pose.orientation.x * -std::sin(-edge.pose.orientation.z));
    p2.y = p1.y + (edge.pose.orientation.x * std::cos(-edge.pose.orientation.z));

    if(p1.x > p2.x){
        geometry_msgs::Point p3 = p1;
        p1 = p2;
        p2 = p3;
    }

    return std::make_pair(p1,p2);
}

static void sqliteCallbackFunc(void *foo, const char* statement) {
    ROS_INFO_STREAM("executed statement: " << statement);
}



std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > > Recognizer::dbAssociation(const std::vector<long>::const_iterator fpbegin, const std::vector<long>::const_iterator fpend, const FingerprintMap& fpMap, const std::map<long, Pole> &poleMap, sqlite3* db, sqlite3_stmt* poleStmt, sqlite3_stmt* wallStmt) const
{
    auto start = std::chrono::system_clock::now();    
    std::chrono::duration<double> elapsed_seconds;
    sqlite3_stmt *polePos = NULL, *matchedPrints = NULL, *stmt = NULL;

    std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > > allMatches; // don't ask...

    float angleTolerance = mMaxAngleTolerance;

    auto afterStep = start;

    for(auto it=fpbegin; it != fpend; ++it){
        long curId = *it;        
        auto trackedPole = poleMap.at(curId);        
        bool isEdge = isEdgeType(trackedPole);

        if(ros::Time::now() - trackedPole.header.stamp > ros::Duration(.5)) continue; // ignore dead poles (older than .5 seconds)

        if(isEdge){            
            if(!mDetectWalls) continue; // Skip, if wall detection is disabled

            // switch the statement for the wall statement
            stmt = wallStmt;
            angleTolerance = mMaxWallAngleTolerance;
        }
        else stmt = poleStmt;

        std::map<long, std::pair<double, double> > dists = fpMap.at(curId);
        auto coords = map2UTM(trackedPole.pose.position, trackedPole.header.stamp);

        if(dists.size() < 10) // min 10 prints per pole
            continue;

        sqlite3_bind_double(stmt, 1, coords.x - mMaxSearchRadius);
        sqlite3_bind_double(stmt, 2, coords.y - mMaxSearchRadius);
        sqlite3_bind_double(stmt, 3, coords.x + mMaxSearchRadius);
        sqlite3_bind_double(stmt, 4, coords.y + mMaxSearchRadius);
        sqlite3_bind_int(stmt, 5, trackedPole.type);

        int i=5;
        for(auto it = dists.begin(); it != dists.end(); ++it)
        {
            if(isEdge && poleMap.find(it->first) == poleMap.end())
                continue;

            double dist_tol = std::max(std::min(it->second.first*mMaxDistTolerance, (double)mMaxDistAbsTolerance), (double)mMinDistAbsTolerance);
            sqlite3_bind_double(stmt, ++i, it->second.first - dist_tol);
            sqlite3_bind_double(stmt, ++i, it->second.first + dist_tol);

            double minAngle = it->second.second - ((angleTolerance*10.0)/it->second.first), maxAngle = it->second.second + ((angleTolerance*10.0)/it->second.first);
            sqlite3_bind_double(stmt, ++i, minAngle);
            sqlite3_bind_double(stmt, ++i, maxAngle);

            if(isEdge){
                const Pole& other = poleMap.at(it->first);
                auto otherCoords = map2UTM(other.pose.position, other.header.stamp);
                double oX = isEdgeType(other) ? 0.0 : otherCoords.x;
                double oY = isEdgeType(other) ? 0.0 : otherCoords.y;
                sqlite3_bind_double(stmt, ++i, oX - mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oX + mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oY - mMaxSearchRadius);
                sqlite3_bind_double(stmt, ++i, oY + mMaxSearchRadius);
            }
        }

        ++i;

        std::string curPole = "";
        unsigned int distCount = 0;
        std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > results;

        while(sqlite3_step(stmt) == SQLITE_ROW)
        {
            elapsed_seconds = std::chrono::system_clock::now()-afterStep;
            distCount = sqlite3_column_int(stmt, 0);
            double minDistCount = mMaxSearchRadius*5.0;
            if(isEdge) minDistCount *= 2.0;
            if(curPole.compare(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1))) && distCount > minDistCount)
            {
                curPole = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
                double prec = distCount/(double)dists.size();
                //double rec  = distCount/(double)distCount; // recall calculation disabled
                Pole pole;
                pole.ID = curPole;
                pole.pose.position.x = sqlite3_column_double(stmt, 3);
                pole.pose.position.y = sqlite3_column_double(stmt, 4);
                pole.pose.position.z = 0.0;
                if(isEdgeType(trackedPole)){
                    tf2::Vector3 wallVec(sqlite3_column_double(stmt, 6)-pole.pose.position.x, sqlite3_column_double(stmt, 7) - pole.pose.position.y, 0.0);
                    pole.pose.orientation.x = wallVec.length();
                    pole.pose.orientation.z = calcAngle(wallVec);
                    if(std::abs(trackedPole.pose.orientation.z-pole.pose.orientation.z) > M_PI_2/2.0) // 45 degrees orientation error? I don't think so.
                        continue;
                }
                pole.type = sqlite3_column_int(stmt, 2);
                pole.life = std::min(100.0,prec*100);
                pole.mappingConfidence = pole.life/100.0;
                results.push_back(std::make_tuple(pole.life, pole.pose.position.x, pole.pose.position.y, pole.pose.position.z, pole, distCount, 0)); //last entry = dbFingerprint count
            }
        }

        // reset statement
        sqlite3_clear_bindings(stmt);
        sqlite3_reset(stmt);

        if(!results.empty())
        {
            Pole pole;
            pole.pose.position = coords;
            pole.pose.orientation = trackedPole.pose.orientation;

            pole.ID = std::to_string(curId);
            pole.type = trackedPole.type;
            pole.header.stamp = trackedPole.header.stamp;
            pole.header.frame_id = "utm";
            pole.life = trackedPole.life;
            std::sort(results.begin(), results.end(), distCompare);
            allMatches.push_back(std::make_tuple(pole, results));
        }
        afterStep = std::chrono::system_clock::now();
    }

    if(polePos) sqlite3_finalize(polePos);
    if(matchedPrints) sqlite3_finalize(matchedPrints);

    auto afterClose = std::chrono::system_clock::now();
    elapsed_seconds = afterClose-start;
    //ROS_ERROR_STREAM("Database thread association took " << elapsed_seconds.count() * 1000 << "ms");

    return allMatches;
}


std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > Recognizer::associate(const FingerprintMap& fingerprints, const std::map<long, Pole> &poleMap) const
{    
    auto start = std::chrono::system_clock::now();
    static double avg_time = 0.0;
    static unsigned int time_vals = 0;

    static std::map<long, ros::Time> trackedMatches;
    static std::map<long, std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > > lastMatches;
    std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > > allMatches;

    std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > associated;

    std::vector<std::future<std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > > > > results;

    std::vector<long> toMatch;    

    for(auto entry : fingerprints){
        if(trackedMatches.find(entry.first) == trackedMatches.end() || ros::Time::now() - trackedMatches[entry.first] > ros::Duration(std::get<5>(std::get<1>(lastMatches[entry.first]).front())/(2*mMaxDists)+0.05))
            toMatch.push_back(entry.first);
        else{
            Pole trackedPole = poleMap.at(entry.first);
            trackedPole.header.frame_id = "utm";
            trackedPole.pose.position = map2UTM(trackedPole.pose.position, trackedPole.header.stamp);
            allMatches.push_back(std::make_tuple(trackedPole, std::get<1>(lastMatches[entry.first])));
        }
    }

    const int partitionSize = toMatch.size()/mThreadCount;
    for(int i=0; i<mThreadCount-1; ++i){
        auto fpStart = toMatch.cbegin();
        for(int j=0; j<partitionSize * i; ++j)
            ++fpStart;
        auto fpEnd = fpStart;
        for(int j=0; j<partitionSize; ++j)
            ++fpEnd;
        results.push_back(std::async(std::launch::async, std::bind(&Recognizer::dbAssociation, this, fpStart, fpEnd, std::cref(fingerprints), std::cref(poleMap), mDatabase[i], mStatements[i], mWallStatements[i])));
    }


    auto fpStart = toMatch.cbegin();
    for(int i=0; i<partitionSize * (mThreadCount-1); ++i)
        ++fpStart;

    auto fpEnd = fpStart;
    for(int i=0; i<partitionSize; ++i)
        ++fpEnd;

    for(auto elem : dbAssociation(fpStart, fpEnd, fingerprints, poleMap, mDatabase[mThreadCount-1], mStatements[mThreadCount-1], mWallStatements[mThreadCount-1]))
        allMatches.push_back(elem);

    for(int i=0; i<results.size(); ++i){
        for(auto elem : results[i].get())
            allMatches.push_back(elem);
    }

    if(allMatches.size() >= 1){
        associated = selectBestMatchings(allMatches);
        associated = rejectOutliers(associated);
    }

    auto afterMatching = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = afterMatching-start;
    const double milliseconds = elapsed_seconds.count() * 1000;
    avg_time = (milliseconds + time_vals * avg_time)/(time_vals+1);
    ++time_vals;
    ROS_DEBUG_STREAM("Database matching took " << milliseconds << "ms (avg " << avg_time << ") got " << associated.size() << " pole associations - search radius " << mMaxSearchRadius);

    for(auto match : allMatches){
        long localID = std::stol(std::get<0>(match).ID);
        if(std::find(toMatch.begin(), toMatch.end(), localID) != toMatch.end()){
            trackedMatches[localID] = ros::Time::now();
            lastMatches[localID] = match;
        }
    }

    if(mFingerprintsMarkerPublisher.getNumSubscribers() > 0)
        visualizeMatchedFingerprints(associated, fingerprints, poleMap);

    return associated;
}


geometry_msgs::Pose Recognizer::calculateOffset(const std::vector<std::pair<Pole, Pole> >& associations) const
{
    ros::Duration d(0.06);
    auto stamp = ros::Time::now() - d;

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = mTfBuffer.lookupTransform("velodyne_base_link", "utm", stamp, ros::Duration(0.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remote(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wall_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wall_remote(new pcl::PointCloud<pcl::PointXYZ>);

    int wallCount = 0;

    double angleDifference = 0.0;

    for(const std::pair<Pole, Pole>& match : associations){
        const Pole& poleLocal = match.first;
        const Pole& poleRemote = match.second;


        if(poleLocal.type == pole_based_localization::Pole::TYPE_WALL || poleLocal.type == pole_based_localization::Pole::TYPE_LANELINE){
            ++wallCount;
            angleDifference += poleLocal.pose.orientation.z - poleRemote.pose.orientation.z;
            const float wallPointDist = 5.0f;

            for(int i=1; i<std::ceil(poleLocal.pose.orientation.x/wallPointDist); ++i){
                geometry_msgs::Point wallLocalPoint;
                wallLocalPoint.x = poleLocal.pose.position.x + i*wallPointDist * std::sin(poleLocal.pose.orientation.z);
                wallLocalPoint.y = poleLocal.pose.position.y + i*wallPointDist * std::cos(-poleLocal.pose.orientation.z);

                geometry_msgs::Point wallRemotePoint = getClosestPointOnLine(poleRemote, wallLocalPoint);

                geometry_msgs::PointStamped psIn, psOut;
                psIn.header = poleLocal.header;
                psIn.point = wallLocalPoint;
                psIn.header.stamp = stamp;
                tf2::doTransform(psIn, psOut, transformStamped);
                auto pLocal = psOut.point;
                psIn.point = wallRemotePoint;
                tf2::doTransform(psIn, psOut, transformStamped);
                wallRemotePoint = psOut.point;
                pcl::PointXYZ pl(pLocal.x, pLocal.y, 0);
                pcl::PointXYZ pr(wallRemotePoint.x, wallRemotePoint.y, 0);

                if(calcDistSquared(pLocal,wallRemotePoint) > (mMaxSearchRadius*mMaxSearchRadius)/8.0)
                    continue; // don't add points that are too far

                cloud_wall_local->push_back(pl);
                cloud_wall_remote->push_back(pr);
                cloud_local->push_back(pl);
                cloud_remote->push_back(pr);
            }
            continue;
        }

        geometry_msgs::PointStamped psIn, psOut;
        psIn.header = poleLocal.header;
        psIn.point = poleLocal.pose.position;
        psIn.header.stamp = stamp;
        tf2::doTransform(psIn, psOut, transformStamped);
        auto pLocal = psOut.point;
        pcl::PointXYZ pl(pLocal.x, pLocal.y, 0);
        cloud_local->push_back(pl);                        

        psIn.point = poleRemote.pose.position;
        tf2::doTransform(psIn, psOut, transformStamped);
        auto pRemote = psOut.point;
        pcl::PointXYZ pr(pRemote.x, pRemote.y, 0);
        cloud_remote->push_back(pr);
    }

    geometry_msgs::Pose result;
    if(cloud_remote->empty() && cloud_wall_remote->empty())
        return result;

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double> pclSVD;
    pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ, double>::Matrix4 trans, transwall;
    pclSVD.estimateRigidTransformation(*cloud_local, *cloud_remote, trans);
    double rz = 0.0;
    result.position.x = 0.0;
    result.position.y = 0.0;
    result.position.z = 0.0;

    if(!std::isnan(trans(1)) && !std::isnan(trans(12)) && !std::isnan(trans(13)) && std::abs(trans(12)) < mMaxSearchRadius*2.0 && std::abs(trans(13)) < mMaxSearchRadius*2.0){
        result.position.x = trans(12);
        result.position.y = trans(13);
        rz = trans(1);
    }

    result.orientation.x = 0.0;
    result.orientation.y = 0.0;
    result.orientation.z = 0.0;
    result.orientation.w = 1.0;

    try{
        transformStamped = mTfBuffer.lookupTransform("map", "velodyne_base_link", stamp, ros::Duration(0.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return result; // Couldn't get transform...
    }

    geometry_msgs::PoseStamped psIn, psOut, zero, zeroOut;
    psIn.header.frame_id = "velodyne_base_link";
    psIn.pose = result;
    psIn.header.stamp = ros::Time::now();
    tf2::doTransform(psIn, psOut, transformStamped);
    zero.header = psIn.header;
    result = psOut.pose;
    tf2::doTransform(zero, zeroOut, transformStamped);
    result.position.x -= zeroOut.pose.position.x;
    result.position.y -= zeroOut.pose.position.y;
    result.orientation.z = -rz;
    result.orientation.x = 0.0;
    result.orientation.y = 0.0;
    result.orientation.w = 1.0;

    result.position.x *= -1.0;
    result.position.y *= -1.0;

    tf2::Matrix3x3 tf3d;
    tf3d.setValue(trans(0), trans(4), trans(8), trans(1), trans(5), trans(9), trans(2), trans(6), trans(10));
    tf2::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf2::convert(tfqt, result.orientation);

    return result;
}

std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > Recognizer::rejectOutliers(const std::vector<std::pair<Pole, Pole> > associations) const
{
    std::vector<std::pair<Pole, Pole> > result;
    if(associations.empty()) return result;

    // calculate all dists and angles
    std::vector<double> dists, wallDists, angles, wallAngles;
    for(auto association : associations){
        if(association.first.type == pole_based_localization::Pole::TYPE_WALL || association.first.type == pole_based_localization::Pole::TYPE_LANELINE){
            wallDists.push_back(calcLineDist(association.second, association.first.pose.position));
            wallAngles.push_back(std::abs(association.first.pose.orientation.z-association.second.pose.orientation.z));
        }
        else{
            dists.push_back(calcDistSquared(association.first.pose.position, association.second.pose.position));
            angles.push_back(calcAngle(association.second, association.first));
        }
    }

    // calculate median displacement vectors    
    std::sort(wallDists.begin(), wallDists.end());
    std::sort(wallAngles.begin(), wallAngles.end());
    std::sort(dists.begin(), dists.end());
    std::sort(angles.begin(), angles.end());

    double medDist, medAngle, medWallDist, medWallAngle;
    medDist = dists.empty() ? 0 : dists[dists.size()/2];
    medWallDist = wallDists.empty() ? 0 : std::min(wallDists[wallDists.size()/2], medDist);
    medWallAngle = wallAngles.empty() ? 0 : wallAngles[wallAngles.size()/2];


    for(auto association : associations){
        if(association.first.type == pole_based_localization::Pole::TYPE_WALL || association.first.type == pole_based_localization::Pole::TYPE_LANELINE){
            if(std::abs(association.first.pose.orientation.z-association.second.pose.orientation.z) <= medWallAngle+mMaxWallAngleTolerance && calcLineDist(association.second, association.first.pose.position) <= mMaxSearchRadius/4.0 + medWallDist)
                result.push_back(association);            
        }
        else if(calcDistSquared(association.first.pose.position, association.second.pose.position) - medDist <= std::pow(mMaxSearchRadius/4.0, 2.0))
            result.push_back(association);
    }

    return result;
}


// Simple randomized optimization of matchings. Reduces localization error about 10% and can be skipped in order to save computational time
std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > Recognizer::selectBestMatchings(const std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > >& allMatches) const
{
    const short its = 10, popSize = 20;
    const float prob = 0.1f, thres = 100.1 - allMatches.size();
    auto pop = genPopulation(allMatches, popSize);
    static std::pair<double, std::vector<std::pair<Pole, Pole> > > currentBest = std::make_pair(-1.0, std::vector<std::pair<Pole, Pole> >());

    if(currentBest.second.size() > 0)
    {
        adaptOldBest(currentBest, allMatches);
        pop.insert(pop.begin(), currentBest);
    }

    for(int i=0; i<its; ++i)
    {
        std::vector<std::pair<double, std::vector<std::pair<Pole, Pole> > > > newPop;
        if (currentBest.first >= 0.0)
            newPop.insert(newPop.begin(), pop.begin(), pop.begin()+(pop.size()/2));

        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(pop.begin(), pop.begin()+(pop.size()-(pop.size()/2)), g); // We throw away the worst 50 percent

        for(auto it = pop.begin(); it != pop.end()-(pop.size()/2); ++it)
        {            
            auto c = it->second;
            permutate(c, prob, allMatches);
            auto cc = std::make_pair(cost(c), c);
            newPop.push_back(cc);
            if(currentBest.first < 0.0 || currentBest.first > cc.first)
            {
                currentBest = cc;
                if(thres > cc.first)
                    return currentBest.second;                
            }
        }
        std::sort(newPop.begin(), newPop.end(), costCompare);        
        pop = newPop;
    }

    return currentBest.second;
}


void Recognizer::permutate(std::vector<std::pair<Pole, Pole> > &matching, float prob, const std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > >& allMatches) const
{
    std::map<std::string, std::string> matchedReverse;

    // build matched map
    for(auto entry : matching)
        matchedReverse[entry.second.ID] = entry.first.ID;

    for(int i=0; i<matching.size(); ++i)
        if(rand() % 100 < 100*prob)
        {
            for(auto entry : allMatches)
            {
                if(std::get<0>(entry).ID == matching[i].first.ID)
                {
                    int r = 0;
                    int tries = 0;
                    do{
                        r = rand() % std::get<1>(entry).size();
                    }while(matchedReverse.find(std::get<4>(std::get<1>(entry)[r]).ID) != matchedReverse.end() && ++tries <= 20);

                    if(tries <= 20)
                    {
                        matchedReverse.erase(matching[i].second.ID);
                        matching[i].second = std::get<4>(std::get<1>(entry)[r]);
                        matchedReverse[matching[i].second.ID] = matching[i].first.ID;
                        break;
                    }
                }
            }
        }
}


void Recognizer::adaptOldBest(std::pair<double, std::vector<std::pair<Pole, Pole> > >& oldBest, const std::vector<std::tuple<Pole, std::vector<std::tuple<double, double, double, double, Pole, unsigned int, double> > > >& allMatches) const
{        
    std::map<std::string, std::string> matched, matchedReverse;
    std::vector<std::pair<Pole, Pole>> newMatches;

    // build matched map
    for(auto entry : oldBest.second)
    {
        matched[entry.first.ID] = entry.second.ID;
        matchedReverse[entry.second.ID] = entry.first.ID;
    }

    // remove disappeared matches
    for(auto entry : oldBest.second)
        for(auto match : allMatches)
            if(entry.first.ID == std::get<0>(match).ID)
            {                                
                // update position
                newMatches.push_back(std::make_pair(std::get<0>(match), entry.second));
                break;
            }

    oldBest.second.clear();

    for(auto entry : newMatches)
        oldBest.second.push_back(entry);

    // add new matches
    for(auto match : allMatches)

    {
        if(matched.find(std::get<0>(match).ID) == matched.end())
        {
            int r = 0;
            int tries = 0;
            do{
                r = rand() % std::get<1>(match).size();
            }while(matchedReverse.find(std::get<4>(std::get<1>(match)[r]).ID) != matchedReverse.end() && ++tries <= 20);

            if(tries <= 20){
                oldBest.second.push_back(std::make_pair(std::get<0>(match), std::get<4>(std::get<1>(match)[r])));
                matched[std::get<0>(match).ID] = std::get<4>(std::get<1>(match)[r]).ID;
            }
        }
    }

    // update cost
    oldBest.first = cost(oldBest.second);
}


std::vector<std::pair<double, std::vector<std::pair<pole_based_localization::Pole, pole_based_localization::Pole> > > > Recognizer::genPopulation(const std::vector<std::tuple<pole_based_localization::Pole, std::vector<std::tuple<double, double, double, double, pole_based_localization::Pole, unsigned int, double> > > >& allMatches, const short popSize) const
{
    std::vector<std::pair<double, std::vector<std::pair<Pole, Pole> > > > result;
    for(int i=0; i<popSize; ++i)
    {
        std::vector<std::pair<Pole, Pole> > entry;
        std::set<std::string> dbMatched;

        for(auto match : allMatches)
        {
            auto matches = std::get<1>(match);                        

            for(int j=0; j<matches.size(); ++j)
            {                
                int r;
                int tries = 0;
                do
                {
                    r = rand() % (i+1);
                    r = (r+j) % (matches.size() > 1 ? (matches.size()-1) : 1);
                }while(dbMatched.find(std::get<4>(matches[r]).ID) != dbMatched.end() && tries++ < 15);

                if(tries <= 15)
                {
                    dbMatched.insert(std::get<4>(matches[r]).ID);
                    entry.push_back(std::make_pair(std::get<0>(match), std::get<4>(matches[r])));
                    break;
                }
            }
        }
        result.push_back(std::make_pair(cost(entry), entry));       
    }

    std::sort(result.begin(), result.end(), costCompare);
    return result;
}


std::array<double, 3> Recognizer::calcVariances(const std::vector<std::pair<Pole, Pole> >& matchings) const
{
    double varZ = 0.1;
    double measurementUncertainty = 0.5;
    geometry_msgs::Point p;
    geometry_msgs::Point avg = std::accumulate(matchings.begin(), matchings.end(), p, [](geometry_msgs::Point p, std::pair<Pole, Pole> m) {
        geometry_msgs::Point res;
        res.x = p.x + (m.first.pose.position.x - m.second.pose.position.x);
        res.y = p.y + (m.first.pose.position.y - m.second.pose.position.y);
        return res;
        });

    avg.x /= matchings.size() + 1.0;
    avg.y /= matchings.size() + 1.0;
    avg.z = 0.0; // no z

    geometry_msgs::Point var = std::accumulate(matchings.begin(), matchings.end(), p, [avg](geometry_msgs::Point p, std::pair<Pole, Pole> m) {
            geometry_msgs::Point res;
            res.x = m.first.pose.position.x - m.second.pose.position.x;
            res.y = m.first.pose.position.y - m.second.pose.position.y;
            res.z = 0.0; // no z

            p.x = std::pow(res.x - avg.x, 2.0);
            p.y = std::pow(res.y - avg.y, 2.0);
            p.z = 0.0; // no z
            return p;
            });

    var.x /= matchings.size()+1.0;
    var.y /= matchings.size()+1.0;
    varZ = (var.x + var.y)/2.0;


    std::array<double, 3> res;
    res[0] = var.x + measurementUncertainty;
    res[1] = var.y + measurementUncertainty;
    res[2] = std::min(0.01, varZ + measurementUncertainty/100.0);
    return res;
}


double Recognizer::cost(const std::vector<std::pair<Pole, Pole> >& pop) const
{
    if(pop.empty()) return 100.0;
    geometry_msgs::Point p;
    p.x = p.y = p.z = 0.0;

    geometry_msgs::Point avg = std::accumulate(pop.begin(), pop.end(), p, [this](geometry_msgs::Point p, std::pair<Pole, Pole> m) {
            geometry_msgs::Point res;
            if(m.first.type == pole_based_localization::Pole::TYPE_WALL || m.first.type == pole_based_localization::Pole::TYPE_LANELINE){
                res.x = 0.0;
                res.y = calcLineDist(m.second, m.first.pose.position);
                res.z = 0.0;
            }
            else{
                res.x = p.x + (m.first.pose.position.x - m.second.pose.position.x);
                res.y = p.y + (m.first.pose.position.y - m.second.pose.position.y);
                res.z = 0.0; //p.z + (m.first.pose.position.z - m.second.pose.position.z);
            }
            return res;
});


    avg.x /= pop.size();
    avg.y /= pop.size();
    avg.z = 0.0; // no z

    double var = std::accumulate(pop.begin(), pop.end(), 0, [this, avg](double c, std::pair<Pole, Pole> m) {
            geometry_msgs::Point res;
            if(m.first.type == pole_based_localization::Pole::TYPE_WALL || m.first.type == pole_based_localization::Pole::TYPE_LANELINE){
                res.x = 0.0;
                res.y = std::pow(calcLineDist(m.second, m.first.pose.position), 2.0);
                res.z = 0.0;
            }
            else{
                res.x = m.first.pose.position.x - m.second.pose.position.x;
                res.y = m.first.pose.position.y - m.second.pose.position.y;
                res.z = 0.0; //m.first.pose.position.z - m.second.pose.position.z; no z
            }

            return c + calcDistSquared(res, avg);
            });

    var /= pop.size();

    double dist = std::accumulate(pop.begin(), pop.end(), 0, [this](double c, std::pair<Pole, Pole> m) {
        return c + calcDistSquared(m.first.pose.position, m.second.pose.position);
    });    

    return 100-rejectOutliers(pop).size()+dist/pop.size()+var*var;
}


geometry_msgs::Point Recognizer::map2UTM(const geometry_msgs::Point &point, ros::Time stamp) const
{
    std_msgs::Header header;
    geometry_msgs::Point result;
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
    psIn.point = point;

    tf2::doTransform(psIn, psOut, transformStamped);

    result.x = psOut.point.x;
    result.y = psOut.point.y;
    result.z = psOut.point.z;    

    return result;
}


geometry_msgs::Point Recognizer::UTM2map(const geometry_msgs::Point &point, ros::Time stamp) const
{
    std_msgs::Header header;
    geometry_msgs::Point result;
    header.stamp = stamp;
    header.frame_id = "utm";
    static geometry_msgs::TransformStamped transformStamped; // map2utm is a static transform


    if(transformStamped.header.stamp.sec == 0.0)
    {
        try{
            mTfBuffer.canTransform("map", "utm", stamp, ros::Duration(5.0));
            transformStamped = mTfBuffer.lookupTransform("map", "utm", stamp, ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return point;
        }
    }


    geometry_msgs::PointStamped psIn, psOut;
    psIn.header = header;
    psIn.point = point;

    tf2::doTransform(psIn, psOut, transformStamped);

    result.x = psOut.point.x;
    result.y = psOut.point.y;
    result.z = psOut.point.z;

    return result;
}


double Recognizer::calcDist(const Pole &pole1, const Pole &pole2) const
{
    if(pole1.type != pole_based_localization::Pole::TYPE_WALL && pole2.type != pole_based_localization::Pole::TYPE_WALL &&
            pole1.type != pole_based_localization::Pole::TYPE_LANELINE && pole2.type != pole_based_localization::Pole::TYPE_LANELINE) // no walls or lines
        return calcDist(pole1.pose.position, pole2.pose.position);

    // two walls: For two walls their fingerprint dist is actually their angle (I'm sorry)
    if(pole1.type == pole2.type)
        return std::abs(pole1.pose.orientation.z - pole2.pose.orientation.z);

    // wall to pole
    if(pole1.type == pole_based_localization::Pole::TYPE_WALL || pole1.type == pole_based_localization::Pole::TYPE_LANELINE)
        return calcLineDist(pole1, pole2.pose.position);
    else // pole to wall (actually the same as wall to pole)
        return calcLineDist(pole2, pole1.pose.position);
}


double Recognizer::calcDist(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2) const
{    
    return std::sqrt(std::pow(point1.x - point2.x, 2.0) + std::pow(point1.y - point2.y, 2.0));// + pow(point1.z - point2.z, 2.0)); no z right now
}


double Recognizer::calcDistSquared(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2) const
{
    return std::pow(point1.x - point2.x, 2.0) + std::pow(point1.y - point2.y, 2.0);// + pow(point1.z - point2.z, 2.0)); no z right now
}


double Recognizer::calcRangeDistSquared(const Pole &feat1, const Pole &feat2) const
{
    // Any walls or lines involved?
    if(feat1.type == pole_based_localization::Pole::TYPE_WALL || feat2.type == pole_based_localization::Pole::TYPE_WALL ||
       feat1.type == pole_based_localization::Pole::TYPE_LANELINE || feat2.type == pole_based_localization::Pole::TYPE_LANELINE){
        // Both walls
        if(feat1.type == feat2.type){
            geometry_msgs::Point feat1Start = feat1.pose.position;
            geometry_msgs::Point feat1End = feat1.pose.position;
            feat1End.x += feat1.pose.orientation.x * -std::sin(-feat1.pose.orientation.z);
            feat1End.y += feat1.pose.orientation.x * std::cos(-feat1.pose.orientation.z);

            geometry_msgs::Point feat2Start = feat2.pose.position;
            geometry_msgs::Point feat2End = feat2.pose.position;
            feat2End.x += feat2.pose.orientation.x * -std::sin(-feat2.pose.orientation.z);
            feat2End.y += feat2.pose.orientation.x * std::cos(-feat2.pose.orientation.z);

            double feat1StartDist = std::min(calcDistSquared(feat1Start, feat2Start), calcDistSquared(feat1Start, feat2End));
            double feat1EndDist = std::min(calcDistSquared(feat1End, feat2Start), calcDistSquared(feat1End, feat2End));

            return std::min(feat1StartDist, feat1EndDist);
        }
        else{
            // only one wall/line
            const Pole& wall = feat1.type == pole_based_localization::Pole::TYPE_WALL || feat1.type == pole_based_localization::Pole::TYPE_LANELINE ? feat1 : feat2;
            const Pole& feat = feat1.type == pole_based_localization::Pole::TYPE_WALL || feat1.type == pole_based_localization::Pole::TYPE_LANELINE ? feat2 : feat1;

            geometry_msgs::Point wallStart = wall.pose.position;
            geometry_msgs::Point wallEnd = wall.pose.position;
            double wallLength = wall.pose.orientation.x;
            wallEnd.x += wallLength * -std::sin(-wall.pose.orientation.z);
            wallEnd.y += wallLength * std::cos(-wall.pose.orientation.z);

            double startDist = calcDistSquared(feat.pose.position, wallStart);
            double endDist = calcDistSquared(feat.pose.position, wallEnd);

            // If the feature is in between the wall start and end, take the line dist
            if(startDist < wallLength && endDist < wallLength)
                return calcLineDist(wall, feat.pose.position);

            // otherwise take the smaller dist to wall start or end
            return std::min(startDist, endDist);
        }
    }

    return calcDistSquared(feat1.pose.position, feat2.pose.position);
}


double Recognizer::calcLineDist(const Pole &line, const geometry_msgs::Point & point) const
{
    if(line.type != pole_based_localization::Pole::TYPE_WALL && line.type != pole_based_localization::Pole::TYPE_LANELINE){
        ROS_ERROR("Got a non line feature as line in calcLineDist!");
        return 0.0;
    }
    geometry_msgs::Point lineStart = line.pose.position;
    geometry_msgs::Point lineEnd = line.pose.position;
    lineEnd.x += -std::sin(-line.pose.orientation.z);
    lineEnd.y += std::cos(-line.pose.orientation.z);

    return std::abs((lineEnd.y-lineStart.y)*point.x - (lineEnd.x-lineStart.x)*point.y + lineEnd.x*lineStart.y - lineEnd.y*lineStart.x)/std::sqrt(std::pow(lineEnd.y-lineStart.y, 2.0) + std::pow(lineEnd.x-lineStart.x, 2.0));
}


geometry_msgs::Point Recognizer::getClosestPointOnLine(const Pole &line, const geometry_msgs::Point &point) const
{
    geometry_msgs::Point result;
    if(line.type != Pole::TYPE_WALL && line.type != Pole::TYPE_LANELINE){
        ROS_ERROR("Got a non line feature as line in getClosestPointOnLine!");
        return result;
    }
    geometry_msgs::Point lineStart = line.pose.position;
    geometry_msgs::Point lineEnd = line.pose.position;
    lineEnd.x += -std::sin(-line.pose.orientation.z);
    lineEnd.y += std::cos(-line.pose.orientation.z);

    geometry_msgs::Point lineVector, pointVector;
    lineVector.x = lineEnd.x-lineStart.x;
    lineVector.y = lineEnd.y-lineStart.y;
    pointVector.x = point.x-lineStart.x;
    pointVector.y = point.y-lineStart.y;
    double dot = lineVector.x*pointVector.x + lineVector.y*pointVector.y;
    dot /= lineVector.x*lineVector.x + lineVector.y*lineVector.y;

    result.x = lineStart.x + lineVector.x*dot;
    result.y = lineStart.y + lineVector.y*dot;

    return result;
}

bool Recognizer::isEdgeType(const pole_based_localization::Pole& feature) const
{
    switch (feature.type) {
    case Pole::TYPE_POLE:
    case Pole::TYPE_CORNER:
        return false;

    case Pole::TYPE_WALL:
    case Pole::TYPE_LANELINE:
        return true;

    default:
        ROS_ERROR("Encountered unknown pole type. Can not decide if edge type!");
        return false;
    }
}

void Recognizer::callbackReconfigure(pole_based_localization::pole_recognitionConfig &config, uint32_t level)
{
    mConfig = config;
    mMaxAngleTolerance = mConfig.max_abs_angle_tolerance;
    mMaxDistTolerance = mConfig.max_rel_dist_tolerance/100.0;
    mMinAngleTolerance = mConfig.min_abs_angle_tolerance;
    mMinDistTolerance = mConfig.min_rel_dist_tolerance/100.0;
    mMaxDistAbsTolerance = mConfig.max_abs_dist_tolerance;
    mMaxSearchRadius = mConfig.max_search_radius;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pole_recognition::Recognizer, nodelet::Nodelet);
