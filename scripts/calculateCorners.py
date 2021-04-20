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

import csv
import math
from geometry_msgs.msg import Point
import geojson as gj


def calcLineDist(linePoints, point):
	if len(linePoints) == 0: return 0
	if len(linePoints) == 1: return 0
	l1 = (linePoints[0].x-linePoints[len(linePoints)-1].x, linePoints[0].y-linePoints[len(linePoints)-1].y)
	p1 = (linePoints[len(linePoints)-1].x-point.x, linePoints[len(linePoints)-1].y-point.y)
	return norm(np.cross(l1, p1))/norm(l1)

def calcAngle(linePoints, linePoints2):
	l1 = (linePoints[0].x-linePoints[len(linePoints)-1].x, linePoints[0].y-linePoints[len(linePoints)-1].y)
	l2 = (linePoints2[0].x-linePoints2[len(linePoints2)-1].x, linePoints2[0].y-linePoints2[len(linePoints2)-1].y)

	x1, y1 = l1
	x2, y2 = l2
	inner_product = x1*x2 + y1*y2
	len1 = math.hypot(x1, y1)
	len2 = math.hypot(x2, y2)
	if len1 == 0.0 or len2==0.0 or inner_product/(len1*len2) >= 1.0: return 0
	if inner_product/(len1*len2) <= -1.0: return math.pi
	return math.acos(inner_product/(len1*len2))

def calcDist(point1, point2):
    return math.sqrt((point1.x-point2.x)**2 + (point1.y-point2.y)**2) # + (point1.z-point2.z)**2) no z

def pairable(point, points, thres):
	res = []	
	i=0
	for oP in points:
		if calcDist(point, oP) < thres:
			res.append(i)
		i += 1
	return res


with open('corners.csv', 'wb') as csvfile:
	csvwriter = csv.writer(csvfile, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	hypos = []
	wallHypos = []	
	minWallLength = 5.0
    with open('walls.csv', 'wb') as csvfileWalls:
		csvwriterWalls = csv.writer(csvfileWalls, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		with open('buildings.geojson') as jsonfile:
			curLine = []
			lastLine = []
			lastWall = []
			curWall = []
			tolerance = math.pi/10.0
			data = gj.load(jsonfile)

			for polygon in data.features:
				for poly in polygon.geometry.coordinates:
					point = Point()     
					point.x = poly[len(poly)-2][0]
					point.y = poly[len(poly)-2][1]
					point.z = 0.0       
					curLine = [point]
					lastLine = []
					curWall = [point]
					lastWall = []
					for p in poly:
						#print p
						point = Point()           
						point.x = p[0]
						point.y = p[1]
						point.z = 0.0       

						curLine.append(point)
						curWall.append(point)
			
						if len(curLine) < 2:
							continue

						if len(lastLine) < 2:
							lastLine = list(curLine)
							lastWall = list(curWall)
							curLine.pop(0)
							curWall.pop(0)
							continue
						
						angle = calcAngle(curLine, lastLine)
						if angle > math.pi/2.0 - tolerance and angle < math.pi/2.0 + tolerance or angle > -math.pi/2.0 - tolerance and angle < -math.pi/2.0 + tolerance:
							hypos.append(curLine[0])
							lastLine = list(curLine)
							curLine.pop(0)
						else:							
							lastLine.pop(0)
							curLine.pop(0)
							lastLine.append(curLine[0])

						if angle > tolerance:
							if calcDist(lastWall[0], lastWall[1]) >= minWallLength:
								wallHypos.append((lastWall[0], lastWall[1]))
							lastWall = list(curWall)
							curWall.pop(0)
						else:
							lastWall.pop()
							curWall.pop(0)
							lastWall.append(curWall[0])


		for wallHypo in wallHypos:
			startPoint = None
			endPoint = None
			if wallHypo[0].x <= wallHypo[1].x:
				startPoint = wallHypo[0]
				endPoint = wallHypo[1]
			else:
				startPoint = wallHypo[1]
				endPoint = wallHypo[0]

			northPoint = Point(startPoint.x, startPoint.y+100.0, 0.0)
			angle = calcAngle([startPoint, endPoint], [startPoint, northPoint])
			lineStringString = "LINESTRING(%s %s, %s %s)" % (startPoint.x, startPoint.y, endPoint.x, endPoint.y)
			csvwriterWalls.writerow([lineStringString, angle])	
	
	toDel = set()
	print len(hypos)
	for hypo in hypos:
		res = pairable(hypo, hypos, 1.5)
		if len(res) > 0: res.pop(0)
		toDel = toDel.union(set(res))		
	
	toDelList = list(toDel)
	toDelList.sort()

	for i in toDelList[::-1]:
		hypos.pop(i)
	

	print len(hypos)
	ide = 0
	for hypo in hypos:		
		ide += 1
		csvwriter.writerow([ide, hypo.x, hypo.y, 1])

		
				
			
		            

