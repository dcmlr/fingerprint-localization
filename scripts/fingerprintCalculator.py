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
import numpy as np
from numpy.linalg import norm

def calcPrints():
    exclude = []
    seq = 0
    with open('features.csv', 'rb') as csvIn:
        poles = csv.reader(csvIn, delimiter=';', quotechar='"')
        with open('fingerprints.csv', 'wb') as csvOut:
            csvwriter = csv.writer(csvOut, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(['poleId','x','y','z','otherId','dist','angle','type'])
            with open('fingerprintsWalls.csv', 'wb') as wallCsvOut:
                csvwallwriter = csv.writer(wallCsvOut, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csvwallwriter.writerow(['id','x1','y1','x2','y2','otherId','dist','angle','oX','oY','type'])
                for pole in poles:
                    if pole[0] in exclude: continue
                    with open('features.csv', 'rb') as csvIn2:
                        other = csv.reader(csvIn2, delimiter=';', quotechar='"')
                        polesInRange = []
                        for otherPole in other:
                            if pole[0] == otherPole[0]: continue # don't fingerprint self
                            distThres = 100.0
                            if dist(pole, otherPole) < distThres: polesInRange.append(otherPole)
                        print pole[0]
                        if len(polesInRange) == 0: continue
                        closest = findClosest(pole, polesInRange)
                        writePrints(pole, closest, polesInRange, csvwriter, csvwallwriter)


def writePrints(pole, closest, poles, csv, wallCsv):
    if int(pole[-1]) >= 2: # walls and lines are in wkt format
        wkt = pole[1]
        startx = float(wkt[wkt.find('(')+1:wkt.find(' ')])
        starty = float(wkt[wkt.find(' ')+1:wkt.find(',')])
        endx = float(wkt[wkt.find(',')+1:wkt.rfind(' ')])
        endy = float(wkt[wkt.rfind(' ')+1:wkt.find(')')])
        if startx > endx: # switch
            tmpx = startx
            tmpy = starty
            startx = endx
            starty = endy
            endx = tmpx
            endy = tmpy
    else:
        x = float(pole[1])
        y = float(pole[2])

    for other in poles:        
        if int(other[-1]) >= 2: # walls and lines are in wkt format
            wkt = other[1]
            oStartx = float(wkt[wkt.find('(')+1:wkt.find(' ')])
            oStarty = float(wkt[wkt.find(' ')+1:wkt.find(',')])
            oEndx = float(wkt[wkt.find(',')+1:wkt.rfind(' ')])
            oEndy = float(wkt[wkt.rfind(' ')+1:wkt.find(')')])
            oX = 0
            oY = 0
            if oStartx > oEndx: # switch
                tmpx = oStartx
                tmpy = oStarty
                oStartx = endx
                oStarty = endy
                endx = tmpx
                endy = tmpy                
        else:
            oX= float(other[1])
            oY= float(other[2])
        if int(pole[-1]) < 2:
            if int(other[-1]) >= 2:
                (angle, distance) = calcLineDistAngle([(oStartx, oStarty), (oEndx, oEndy)], (x, y))
            else:
                direction = (oX - x, oY - y)
                if direction[0] == 0 and direction[1] == 0: continue
                angle = math.acos(direction[1]/math.sqrt(direction[0]**2 + direction[1]**2))
                if direction[0] < 0: angle *= -1
                distance = dist(pole, other)
            csv.writerow([pole[0], x, y, 0.0, other[0], distance, angle, pole[-1]])
        else:
            if int(other[-1]) >= 2:
                distance = calcLineAngle([(startx, starty), (endx, endy)], [(oStartx, oStarty), (oEndx, oEndy)]) #if where fingerprinting two walls against each other, we use their respective angle as dist            
                angle = calcLineAngle([(startx, starty), (endx, endy)], [(startx, starty), (startx, starty+100.0)])
            else:
                (angle, distance) = calcLineDistAngle([(startx, starty), (endx, endy)], (oX, oY))              
                angle -= math.pi
                if angle < -math.pi: angle += 2*math.pi  
            wallCsv.writerow([pole[0], startx, starty, endx, endy, other[0], distance, angle, oX, oY, pole[-1]])
        

def findClosest(pole, poles):
    minDist = 100000000.0
    curMin = None    
    for other in poles:
        if dist(pole, other) < minDist:
            minDist = dist
            curMin = other
 
    return curMin

def calcLineAngle(linePoints, linePoints2):
    l1 = (linePoints[len(linePoints)-1][0]-linePoints[0][0], linePoints[len(linePoints)-1][1]-linePoints[0][1])
    l2 = (linePoints2[len(linePoints2)-1][0]-linePoints2[0][0], linePoints2[len(linePoints2)-1][1]-linePoints2[0][1])
    x1, y1 = l1
    x2, y2 = l2

    # always position x direction
    if x1 < 0.0:
        x1 *= -1
        y1 *= -1

    if x2 < 0.0:
        x2 *= -1
        y2 *= -1
    
    inner_product = x1*x2 + y1*y2
    len1 = math.hypot(x1, y1)
    len2 = math.hypot(x2, y2)
    if len1 == 0.0 or len2==0.0: return 0
    cosAngle = inner_product/(len1*len2)
    if cosAngle > 1.0:
        print "cosAngle too big: %s" % cosAngle
        cosAngle = 1.0
    elif cosAngle < -1.0:
        print "cosAngle too small: %s" % cosAngle
        cosAngle = -1.0
    result = math.acos(cosAngle)

    return result

def calcLineDist(linePoints, point):
    if len(linePoints) == 0: return 0
    if len(linePoints) == 1: return dist(linePoints[0], point)
    l1 = (linePoints[0][0]-linePoints[len(linePoints)-1][0], linePoints[0][1]-linePoints[len(linePoints)-1][1])
    p1 = (linePoints[len(linePoints)-1][0]-point[0], linePoints[len(linePoints)-1][1]-point[1])
    return norm(np.cross(l1, p1))/norm(l1)

def calcLineDistAngle(linePoints, point):
    if len(linePoints) == 0: return 0
    if len(linePoints) == 1: return dist(linePoints[0], point)
    l1 = (linePoints[len(linePoints)-1][0]-linePoints[0][0], linePoints[len(linePoints)-1][1]-linePoints[0][1])
    p1 = (point[0]-linePoints[0][0], point[1]-linePoints[0][1])
    l1 = l1/norm(l1)
    proj = np.dot(p1,l1)
    v_proj = proj*l1[0]+linePoints[0][0], proj*l1[1]+linePoints[0][1]
    rej = (v_proj[0]-point[0], v_proj[1]-point[1])
    angle = math.acos(rej[1]/math.sqrt(rej[0]**2 + rej[1]**2))
    if rej[0] < 0: angle *= -1
    return (angle, norm(rej))

def dist(pole, other):
    if int(pole[-1]) >= 2: # walls and lines are in wkt format
        wkt = pole[1]
        startx = float(wkt[wkt.find('(')+1:wkt.find(' ')])
        starty = float(wkt[wkt.find(' ')+1:wkt.find(',')])
        endx = float(wkt[wkt.find(',')+1:wkt.rfind(' ')])
        endy = float(wkt[wkt.rfind(' ')+1:wkt.find(')')])
        x = (startx + endx)/2.0
        y = (starty + endy)/2.0
    else:
        x = float(pole[1])
        y = float(pole[2])

    if int(other[-1]) >= 2: # walls and lines are in wkt format
        wkt = other[1]
        oStartx = float(wkt[wkt.find('(')+1:wkt.find(' ')])
        oStarty = float(wkt[wkt.find(' ')+1:wkt.find(',')])
        oEndx = float(wkt[wkt.find(',')+1:wkt.rfind(' ')])
        oEndy = float(wkt[wkt.rfind(' ')+1:wkt.find(')')])
        oX = (oStartx + oEndx)/2.0
        oY = (oStarty + oEndy)/2.0
    else:
        oX= float(other[1])
        oY= float(other[2])

    return math.sqrt((x-oX)**2 + (y-oY)**2)
                            

if __name__ == '__main__':
    calcPrints()
