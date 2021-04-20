#!/usr/bin/env python
# Copyright 2021 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
# endorse or promote products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import math
import numpy
import time
import sqlite3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf


def visualize_fingerprints(): 
	global markerPub, currentPos, tfBuffer
	rospy.init_node('db_fingerprint_viz', anonymous=False)
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	markerPub = rospy.Publisher('/pole_recognition/db_fingerprints', MarkerArray, queue_size=20)
	rate = rospy.Rate(0.2)

	currentPos = Point()

	# we need to wait until the node is fully initialized, otherwise we cannot publish (seems to be a bug in rospy)
	time.sleep(1)

	rospy.Subscriber('/localization/odometry/filtered_map', Odometry, callbackPosition, queue_size=50)

	while not rospy.is_shutdown():
		sendMarker()
		rate.sleep()


def callbackPosition(pos):
	global currentPos, mapToUTM, tfBuffer
	utmTrans = tfBuffer.lookup_transform('map', 'utm', pos.header.stamp, rospy.Duration(.03))
	mapToUTM = tf.TransformerROS(True, rospy.Duration(1.0))
	mapToUTM.setTransform(utmTrans)

	ps = PointStamped()
	ps.header = pos.header
	ps.point = pos.pose.pose.position
	currentPos=mapToUTM.transformPoint('utm', ps).point

def sendMarker():
	global markerPub, tfBuffer
	conn = sqlite3.connect(rospy.get_param('/pole_recognition/database_name'))
	conn.enable_load_extension(True)
	conn.execute('SELECT load_extension("mod_spatialite.so")')
	markerArray = MarkerArray()
	mid = 0

	print 'start collecting prints'

	stamp = rospy.Time.now()
	utmToMap = tf.TransformerROS(True, rospy.Duration(1.0))
	mapTrans = tfBuffer.lookup_transform('utm', 'map', stamp, rospy.Duration(.03))
	utmToMap.setTransform(mapTrans)

	marker = None

	with conn:
		cursor = conn.execute('SELECT x,y,dist,angle from (SELECT distinct x, y, dist,angle FROM fingerprints WHERE fingerprints.ROWID IN (SELECT ROWID FROM SpatialIndex WHERE f_table_name=\'fingerprints\' AND search_frame=BuildMbr(?,?,?,?,32633)))where dist > 0',  (currentPos.x-60, currentPos.y-60, currentPos.x+60, currentPos.y+60))
		for row in cursor.fetchall():
			if marker == None or len(marker.points) > 10000:
				if mid > 0:
					markerArray.markers.append(marker)
				mid += 1
				print 'adding new marker ', mid
				marker = Marker()
				marker.type = Marker.LINE_LIST
				marker.id = mid
				marker.lifetime = rospy.Duration(6)
				marker.header.stamp = rospy.Time.now()
				marker.header.frame_id = 'map'
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.04
				marker.ns = 'poles'
	
			point = PointStamped()
			point.header.stamp = stamp
			point.header.frame_id = 'utm'
			point.point.x = row[0]
			point.point.y = row[1]
			point.point.z = 4.0		

			sint = math.sin(-row[3])
			cost = math.cos(-row[3])
			dist = row[2]
			other = PointStamped()
			other.header = point.header
			other.point.x = point.point.x - (dist * sint);
			other.point.y = point.point.y - (dist * -cost);
			other.point.z = 2.0

			marker.points.append(utmToMap.transformPoint('map', point).point)
			color = ColorRGBA()
			color.r = 0.0
			color.g = 0.0
			color.b = 1.0
			color.a = 0.15
			marker.colors.append(color)

			marker.points.append(utmToMap.transformPoint('map', other).point)
			newcolor = ColorRGBA()
			newcolor.r = 0.0
			newcolor.g = 0.0
			newcolor.b = 1.0
			newcolor.a = 0.0
			marker.colors.append(newcolor)

		if mid > 0:
			markerArray.markers.append(marker)
			print len(marker.points)

		cursor.close()

		marker = None

		cursor = conn.execute('SELECT ox,oy,dist,angle from (SELECT distinct ox, oy, dist,angle,id,otherid FROM fingerprintsWalls WHERE fingerprintsWalls.ROWID IN (SELECT ROWID FROM SpatialIndex WHERE f_table_name=\'fingerprintsWalls\' AND search_frame=BuildMbr(?,?,?,?,32633))) where ox!=0.0 AND oy!=0.0 AND dist > 0',  (currentPos.x-60, currentPos.y-60, currentPos.x+60, currentPos.y+60))
		for row in cursor.fetchall():
			if marker == None or len(marker.points) > 10000:
				if marker != None and len(marker.points) > 0:
					markerArray.markers.append(marker)
				mid += 1
				print 'adding new marker ', mid
				marker = Marker()
				marker.type = Marker.LINE_LIST
				marker.id = mid
				marker.lifetime = rospy.Duration(6)
				marker.header.stamp = rospy.Time.now()
				marker.header.frame_id = 'map'
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.04
				marker.ns = 'wall'
	
			point = PointStamped()
			point.header.stamp = stamp
			point.header.frame_id = 'utm'
			point.point.x = row[0]
			point.point.y = row[1]
			point.point.z = 4.0	

			angle = row[3]
			sint = math.sin(-angle)
			cost = math.cos(-angle)
			dist = row[2]
			other = PointStamped()
			other.header = point.header
			other.point.x = point.point.x - (dist * -sint);
			other.point.y = point.point.y - (dist * cost);
			other.point.z = 2.0

			marker.points.append(utmToMap.transformPoint('map', other).point)
			color = ColorRGBA()
			color.r = 1.0
			color.g = 0.0
			color.b = 0.0
			color.a = 0.15
			marker.colors.append(color)

			marker.points.append(utmToMap.transformPoint('map', point).point)
			newcolor = ColorRGBA()
			newcolor.r = 1.0
			newcolor.g = 0.0
			newcolor.b = 0.0
			newcolor.a = 0.0
			marker.colors.append(newcolor)

		if mid > 0:
			if marker != None:
				markerArray.markers.append(marker)
				print len(marker.points)

	print 'sending prints'

	markerPub.publish(markerArray)


if __name__ == '__main__':
	try:
		visualize_fingerprints()
	except rospy.ROSInterruptException:
		pass


