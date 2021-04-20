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

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#include <geodesy/utm.h>
#pragma GCC diagnostic pop


void set_origin(geographic_msgs::GeoPoint map_origin)
{
    geodesy::UTMPoint utm_origin;
    geodesy::fromMsg(map_origin, utm_origin);
    double utm_meridian = (utm_origin.zone-1) * 6 - 177;
    double meridian_convergence = std::atan(std::tan((map_origin.longitude-utm_meridian) * M_PI/180.0) * std::sin(map_origin.latitude * M_PI/180.0));
    ROS_INFO_STREAM("utm origin and meridian convergence " << utm_origin.easting << ", " << utm_origin.northing << ", " << utm_origin.altitude << ", " << meridian_convergence);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "utm";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = utm_origin.easting;
    static_transformStamped.transform.translation.y = utm_origin.northing;
    static_transformStamped.transform.translation.z = 0.0;//utm_origin.altitude;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, meridian_convergence);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}


int main(int argc, char **argv){
    ros::init(argc,argv, "map_utm_transform_broadcaster");
    ros::NodeHandle nh("utm_map_transform_broadcaster");
    ros::Subscriber map_origin_sub = nh.subscribe(nh.resolveName("/localization/map_origin"), 1, &set_origin);

    ROS_INFO("Spinning until killed publishing map to utm transform");
    ros::spin();
    return 0;
}
