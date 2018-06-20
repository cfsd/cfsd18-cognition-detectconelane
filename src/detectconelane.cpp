/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>
#include <cstdlib>
#include <mutex>
#include <condition_variable>
#include "detectconelane.hpp"

DetectConeLane::DetectConeLane(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4)
, m_senderStamp{(commandlineArguments["id"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["id"]))) : (211)}
, m_fakeSlamActivated{(commandlineArguments["fakeSlamActivated"].size() != 0) ? (std::stoi(commandlineArguments["fakeSlamActivated"])==1) : (false)}
, m_guessDistance{(commandlineArguments["guessDistance"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["guessDistance"]))) : (3.0f)}
, m_maxConeAngle{(commandlineArguments["maxConeAngle"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxConeAngle"]))) : (1.570796325f)}
, m_coneWidthSeparationThreshold{(commandlineArguments["coneWidthSeparationThreshold"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["coneWidthSeparationThreshold"]))) : (3.5f)}
, m_coneLengthSeparationThreshold{(commandlineArguments["coneLengthSeparationThreshold"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["coneLengthSeparationThreshold"]))) : (5.5f)}
, m_tick{}
, m_tock{}
, m_newClock{true}
, m_sendMutex()
{
  std::cout<<"DetectConeLane set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    std::cout<<it->first<<" "<<it->second<<std::endl;
  }
}

DetectConeLane::~DetectConeLane()
{
}

void DetectConeLane::setUp()
{
}

void DetectConeLane::tearDown()
{
}


void DetectConeLane::receiveCombinedMessage(std::map<int,ConePackage> currentFrame){
  m_tick = std::chrono::system_clock::now();
  Eigen::MatrixXd extractedCones(3,currentFrame.size());
  std::reverse_iterator<std::map<int,ConePackage>::iterator> it;
  int coneIndex = 0;
  it = currentFrame.rbegin();
  while(it != currentFrame.rend()){
    auto direction = std::get<0>(it->second);
    auto distance = std::get<1>(it->second);
    auto type = std::get<2>(it->second);
    extractedCones(0,coneIndex) = direction.azimuthAngle();
    extractedCones(1,coneIndex) = distance.distance();
    extractedCones(2,coneIndex) = type.type();
    coneIndex++;
    it++;
  }

  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

  for (int i = 0; i < extractedCones.cols(); i++) {
    int type = static_cast<int>(extractedCones(2,i));
    if(type == 1){ nLeft++; }
    else if(type == 2){ nRight++; }
    else if(type == 3){ nSmall++; }
    else if(type == 4){ nBig++; }
    else
    {
      std::cout << "WARNING! Object " << i << " has invalid cone type: " << type << std::endl;
    } // End of else
  } // End of for

  if(extractedCones.cols() > 0){
    DetectConeLane::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);
  }
}

void DetectConeLane::sortIntoSideArrays(Eigen::MatrixXd extractedCones, int nLeft, int nRight, int nSmall, int nBig)
{
  int coneNum = extractedCones.cols();
  //Convert to cartesian
  Eigen::MatrixXd cone;
  Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);

  for(int p = 0; p < coneNum; p++)
  {
    cone = DetectConeLane::Spherical2Cartesian(extractedCones(0,p), 0.0, extractedCones(1,p));
    coneLocal.col(p) = cone;
  } // End of for
//std::cout << "ConeLocal: " << coneLocal.transpose() << std::endl;

  Eigen::MatrixXd coneLeft = Eigen::MatrixXd::Zero(2,nLeft);
  Eigen::MatrixXd coneRight = Eigen::MatrixXd::Zero(2,nRight);
  Eigen::MatrixXd coneSmall = Eigen::MatrixXd::Zero(2,nSmall);
  Eigen::MatrixXd coneBig = Eigen::MatrixXd::Zero(2,nBig);
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  int type;

  for(int k = 0; k < coneNum; k++){
    type = static_cast<int>(extractedCones(2,k));
    if(type == 1)
    {
      coneLeft.col(a) = coneLocal.col(k);
      a++;
    }
    else if(type == 2)
    {
      coneRight.col(b) = coneLocal.col(k);
      b++;
    }
    else if(type == 3)
    {
      coneSmall.col(c) = coneLocal.col(k);
      c++;
    }
    else if(type == 4)
    {
      coneBig.col(d) = coneLocal.col(k);
      d++;
    } // End of else
  } // End of for


  Eigen::ArrayXXf location(1,2);
  location << -3,0;

  Eigen::MatrixXf coneLeft_f = coneLeft.cast <float> ();
  Eigen::MatrixXf coneRight_f = coneRight.cast <float> ();
  Eigen::ArrayXXf sideLeft = coneLeft_f.transpose().array();
  Eigen::ArrayXXf sideRight = coneRight_f.transpose().array();
  //std::cout<<"sideLeft: "<<sideLeft<<std::endl;
  //std::cout<<"sideRight: "<<sideRight<<std::endl;

  DetectConeLane::generateSurfaces(sideLeft, sideRight, location);
} // End of sortIntoSideArrays


void DetectConeLane::generateSurfaces(Eigen::ArrayXXf sideLeft, Eigen::ArrayXXf sideRight, Eigen::ArrayXXf location){
  Eigen::ArrayXXf orderedConesLeft;
  Eigen::ArrayXXf orderedConesRight;
  if (!m_fakeSlamActivated) {
    orderedConesLeft = DetectConeLane::orderAndFilterCones(sideLeft,location);
    orderedConesRight = DetectConeLane::orderAndFilterCones(sideRight,location);
  }else{
    orderedConesLeft = sideLeft;//DetectConeLane::orderCones(sideLeft,location);
    orderedConesRight = sideRight;//DetectConeLane::orderCones(sideRight,location);
  }

  float pathLengthLeft = DetectConeLane::findTotalPathLength(orderedConesLeft);
  float pathLengthRight = DetectConeLane::findTotalPathLength(orderedConesRight);

  Eigen::ArrayXXf longSide;
  Eigen::ArrayXXf shortSide;
  bool leftIsLong;

  if (std::abs(pathLengthLeft-pathLengthRight) > 0.01f) {
    leftIsLong = pathLengthLeft > pathLengthRight;
  } else{
    leftIsLong = orderedConesLeft.rows() > orderedConesRight.rows();
  }

  if(leftIsLong)
  {
    Eigen::ArrayXXf tmpLongSide = orderedConesLeft;
    Eigen::ArrayXXf tmpShortSide;
    if (!m_fakeSlamActivated || orderedConesRight.rows()==0) {
      tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesLeft, orderedConesRight, location, m_coneWidthSeparationThreshold,  m_guessDistance, false);
    }
    else{
      tmpShortSide=orderedConesRight;
    }
    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  }
  else
  {
    Eigen::ArrayXXf tmpLongSide = orderedConesRight;
    Eigen::ArrayXXf tmpShortSide;
    if (!m_fakeSlamActivated|| orderedConesLeft.rows()==0) {
      tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesRight, orderedConesLeft, location, m_coneWidthSeparationThreshold,  m_guessDistance, true);
    }
    else{
      tmpShortSide = orderedConesLeft;
    }
    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  } // End of else

  //std::cout<<"longSide accepted cones: "<<longSide<<"\n";
  //std::cout<<"shortSide accepted cones: "<<shortSide<<"\n";
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

  {
    std::unique_lock<std::mutex> lockSend(m_sendMutex);
    if(longSide.rows() > 1)
    {
      // findSafeLocalPath ends with sending surfaces
      DetectConeLane::findSafeLocalPath(longSide, shortSide);
    }
    else
    {
      if(longSide.rows() == 0)
      { std::cout<<"No Cones"<<"\n";
        //No cones
        opendlv::logic::perception::GroundSurfaceArea surfaceArea;
        surfaceArea.surfaceId(0);
        surfaceArea.x1(1.0f);
        surfaceArea.y1(0.0f);
        surfaceArea.x2(1.0f);
        surfaceArea.y2(0.0f);
        surfaceArea.x3(0.0f);
        surfaceArea.y3(0.0f);
        surfaceArea.x4(0.0f);
        surfaceArea.y4(0.0f);
        m_od4.send(surfaceArea, sampleTime , m_senderStamp);
        //std::cout<<"DetectConeLane send surface: "<<" x1: "<<1<<" y1: "<<0<<" x2: "<<1<<" y2: "<<0<<" x3: "<<0<<" y3: "<<0<<" x4: "<<0<<" y4 "<<0<<" frame ID: "<<0<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime)<<" senderStamp "<<m_senderStamp<<std::endl;
      }
      else if(longSide.rows() == 1 && shortSide.rows() == 0)
      { std::cout<<"1 Cone"<<"\n";
        // 1 cone
        int direction;
        if(leftIsLong)
        {
          direction = -1;
        }
        else
        {
          direction = 1;
        }

        opendlv::logic::perception::GroundSurfaceArea surfaceArea;
        surfaceArea.surfaceId(0);
        surfaceArea.x1(0.0f);
        surfaceArea.y1(0.0f);
        surfaceArea.x2(0.0f);
        surfaceArea.y2(0.0f);
        surfaceArea.x3(longSide(0,0));
        surfaceArea.y3(longSide(0,1)+1.5f*direction);
        surfaceArea.x4(longSide(0,0));
        surfaceArea.y4(longSide(0,1)+1.5f*direction);
        m_od4.send(surfaceArea, sampleTime , m_senderStamp);
        /*std::cout<<"DetectConeLane send surface: "<<" x1: "<<0<<" y1: "<<0<<" x2: "<<0<<" y2: "<<0<<" x3: "<<longSide(0,0)<<" y3: "<<longSide(0,1)+1.5f*direction<<" x4: "<<longSide(0,0)<<" y4 "<<longSide(0,1)+1.5f*direction<<" frame ID: "<<0<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime);
        */
      }
      else
      { std::cout<<"1 on each side"<<"\n";
        //1 on each side
        opendlv::logic::perception::GroundSurfaceArea surfaceArea;
        surfaceArea.surfaceId(0);
        surfaceArea.x1(0.0f);
        surfaceArea.y1(0.0f);
        surfaceArea.x2(0.0f);
        surfaceArea.y2(0.0f);
        surfaceArea.x3(longSide(0,0));
        surfaceArea.y3(longSide(0,1));
        surfaceArea.x4(shortSide(0,0));
        surfaceArea.y4(shortSide(0,1));
        m_od4.send(surfaceArea, sampleTime , m_senderStamp);
        /*std::cout<<"DetectConeLane send surface: "<<" x1: "<<0<<" y1: "<<0<<" x2: "<<0<<" y2: "<<0<<" x3: "<<longSide(0,0)<<" y3: "<<longSide(0,1)<<" x4: "<<shortSide(0,0)<<" y4 "<<shortSide(0,1)<<" frame ID: "<<0<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime);
        */

      }
    } //end send mutex
  } // End of else
} // End of generateSurfaces


// copy from perception-detectcone
Eigen::MatrixXd DetectConeLane::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::MatrixXd::Zero(2,1);
  recievedPoint << xData,
                   yData;
  return recievedPoint;
} // End of Spherical2Cartesian


void DetectConeLane::findSafeLocalPath(Eigen::ArrayXXf sidePointsLeft, Eigen::ArrayXXf sidePointsRight)
{
  Eigen::ArrayXXf longSide, shortSide;

  // Identify the longest side
  float pathLengthLeft = DetectConeLane::findTotalPathLength(sidePointsLeft);
  float pathLengthRight = DetectConeLane::findTotalPathLength(sidePointsRight);
  if(pathLengthLeft > pathLengthRight)
  {
    longSide = sidePointsLeft;
    shortSide = sidePointsRight;
  }
  else
  {
    longSide = sidePointsRight;
    shortSide = sidePointsLeft;
  } // End of else

  int nMidPoints = longSide.rows()*3;
  int nConesShort = shortSide.rows();

  // Divide the longest side into segments of equal length
  Eigen::ArrayXXf virtualPointsLong = DetectConeLane::placeEquidistantPoints(longSide,true,nMidPoints,-1);
  Eigen::ArrayXXf virtualPointsShort(nMidPoints,2);
  float shortestDist, tmpDist, factor;
  int closestConeIndex = -100;

  // Every virtual point on the long side should get one corresponding point on the short side
  for(int i = 0; i < nMidPoints; i = i+1)
  {
    // Find short side cone that is closest to the current long side point
    shortestDist = std::numeric_limits<float>::infinity();
    for(int j = 0; j < nConesShort; j = j+1)
    {
      tmpDist = ((shortSide.row(j)-virtualPointsLong.row(i)).matrix()).norm();
      if(tmpDist < shortestDist)
      {
        shortestDist = tmpDist;
        closestConeIndex = j;
      } // End of if
    } // End of for

    // Check if one of the two segments next to the cone has a perpendicular line to the long side point. If so, place the short side point
    // on that place of the segment. If not, place the point on the cone. If it's the first or last cone there is only one segment to check
    if(closestConeIndex == 0)
    {
      if(shortSide.rows() > 1)
      {
        factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(0),shortSide.row(1),virtualPointsLong.row(i));
      }

      if(shortSide.rows() > 1 && factor > 0 && factor <= 1)
      {
        virtualPointsShort.row(i) = shortSide.row(0)+factor*(shortSide.row(1)-shortSide.row(0));
      }
      else
      {
        virtualPointsShort.row(i) = shortSide.row(0);
      } // End of else
    }
    else if(closestConeIndex == nConesShort-1)
    {
      factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(nConesShort-2),shortSide.row(nConesShort-1),virtualPointsLong.row(i));
      if(factor > 0 && factor <= 1)
      {
        virtualPointsShort.row(i) = shortSide.row(nConesShort-2)+factor*(shortSide.row(nConesShort-1)-shortSide.row(nConesShort-2));
      }
      else
      {
        virtualPointsShort.row(i) = shortSide.row(nConesShort-1);
      } // End of else
    }
    else
    {
      factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(closestConeIndex-1),shortSide.row(closestConeIndex),virtualPointsLong.row(i));
      if(factor > 0 && factor <= 1)
      {
        virtualPointsShort.row(i) = shortSide.row(closestConeIndex-1)+factor*(shortSide.row(closestConeIndex)-shortSide.row(closestConeIndex-1));
      }
      else
      {
        factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(closestConeIndex),shortSide.row(closestConeIndex+1),virtualPointsLong.row(i));
        if(factor > 0 && factor <= 1)
        {
          virtualPointsShort.row(i) = shortSide.row(closestConeIndex)+factor*(shortSide.row(closestConeIndex+1)-shortSide.row(closestConeIndex));
        }
        else
        {
          virtualPointsShort.row(i) = shortSide.row(closestConeIndex);
        } // End of else
      } // End of else
    } // End of else
  } // End of for
  Eigen::ArrayXXf virtualPointsLongFinal, virtualPointsShortFinal;
  if(virtualPointsLong.rows() % 2 == 0)
  {
    // Number of points is even. Accepted.
    virtualPointsLongFinal = virtualPointsLong;
    virtualPointsShortFinal = virtualPointsShort;
  }
  else
  {
    // Number of points is odd. Add another point with tiny extrapolation in the end.
    int nLong = virtualPointsLong.rows();
    int nShort = virtualPointsShort.rows();

    virtualPointsLongFinal.resize(nLong+1,2);
    virtualPointsShortFinal.resize(nShort+1,2);

    virtualPointsLongFinal.topRows(nLong) = virtualPointsLong;
    virtualPointsShortFinal.topRows(nShort) = virtualPointsShort;

    Eigen::ArrayXXf lastVecLong = virtualPointsLong.row(nLong-1)-virtualPointsLong.row(nLong-2);
    lastVecLong = lastVecLong / ((lastVecLong.matrix()).norm());

    virtualPointsLongFinal.bottomRows(1) = virtualPointsLong.row(nLong-1) + 0.01*lastVecLong;
    virtualPointsShortFinal.bottomRows(1) = virtualPointsShort.row(nShort-1);
  } // End of else

  DetectConeLane::sendMatchedContainer(virtualPointsLongFinal, virtualPointsShortFinal);
} // End of findSafeLocalPath


Eigen::ArrayXXf DetectConeLane::placeEquidistantPoints(Eigen::ArrayXXf sidePoints, bool nEqPointsIsKnown, int nEqPoints, float eqDistance)
{
// Places linearly equidistant points along a sequence of points.
// If nEqPoints is known it will not use the input value for eqDistance, and instead calculate a suitable value.
// If nEqPoints is not known it will not use the input value for nEqPoints, and instead calculate a suitable value.

  int nCones = sidePoints.rows();

  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  Eigen::ArrayXXf segLength(nCones-1,1);
  for(int i = 0; i < nCones-1; i = i+1)
  {
    segLength(i) = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
    pathLength = pathLength + segLength(i);
  }

  if(nEqPointsIsKnown)
  {
    // Calculate equal subdistances
    eqDistance = pathLength/(nEqPoints-1);
  }
  else
  {
    //Calculate how many points will fit
    nEqPoints = static_cast<int>(std::ceil(pathLength/eqDistance))+1;
  }

  // The latest point that you placed
  Eigen::ArrayXXf latestPointCoords = sidePoints.row(0);
  // The latest cone that you passed
  int latestConeIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of linearly equidistant points
  Eigen::ArrayXXf newSidePoints(nEqPoints,2);
  // The first point should be at the same place as the first cone
  newSidePoints.row(0) = latestPointCoords;

  // A temporary vector
  Eigen::ArrayXXf vec(1,2);
  // Temporary distances
  float distanceToGoFromLatestPassedPoint, lengthOfNextSeg;
  // Start stepping through the given path
  for(int i = 1; i < nEqPoints-1; i = i+1)
  {
    // If the next point should be in the segment you are currently in, simply place it.
    if(remainderOfSeg > eqDistance)
    {
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (eqDistance/remainderOfSeg)*vec;
    }
    else // If you need to go to the next segment, keep in mind which cones you pass and how long distance you have left to go.
    {
      latestConeIndex = latestConeIndex+1;
      distanceToGoFromLatestPassedPoint = eqDistance-remainderOfSeg;
      lengthOfNextSeg = segLength(latestConeIndex);

      while(distanceToGoFromLatestPassedPoint > lengthOfNextSeg)
      {
        latestConeIndex = latestConeIndex+1;
        distanceToGoFromLatestPassedPoint = distanceToGoFromLatestPassedPoint - lengthOfNextSeg;
        lengthOfNextSeg = segLength(latestConeIndex);
      } // End of while

      latestPointCoords = sidePoints.row(latestConeIndex);
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (distanceToGoFromLatestPassedPoint/segLength(latestConeIndex))*vec;
    } // End of else

    // In every case, save the point you just placed and check how much of that segment is left.
    newSidePoints.row(i) = latestPointCoords;
    remainderOfSeg = ((sidePoints.row(latestConeIndex+1)-latestPointCoords).matrix()).norm();

  } // End of for
  // The last point should be at the same place as the last cone.
  newSidePoints.row(nEqPoints-1) = sidePoints.row(nCones-1);

  return newSidePoints;
} // End of placeEquidistantPoints


Eigen::ArrayXXf DetectConeLane::traceBackToClosestPoint(Eigen::ArrayXXf p1, Eigen::ArrayXXf p2, Eigen::ArrayXXf q)
{
   // Input: The coordinates of the first two points. (row vectors)
   //        A reference point q (vehicle location)
   // Output: the point along the line that is closest to the reference point.

   Eigen::ArrayXXf v = p1-p2;	// The line to trace
   Eigen::ArrayXXf n(1,2);	// The normal vector
   n(0,0) = -v(0,1); n(0,1) = v(0,0);
   //float d = (p1(0,0)*v(0,1)-v(0,0)*p1(0,1))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between [0,0] and the vector
   float d = (v(0,1)*(p1(0,0)-q(0,0))+v(0,0)*(q(0,1)-p1(0,1)))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between q and the vector
   return q+n*d;       // Follow the normal vector for that distance
}


Eigen::ArrayXXf DetectConeLane::orderCones(Eigen::ArrayXXf cones, Eigen::ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones in order
  int nCones = cones.rows();
  Eigen::ArrayXXf current = vehicleLocation;
  Eigen::ArrayXXi found(nCones,1);
  found.fill(-1);
  Eigen::ArrayXXf orderedCones(nCones,2);
  float shortestDist;
  float tmpDist;
  int closestConeIndex;

  // The first chosen cone is the one closest to the vehicle. After that it continues with the closest neighbour
  for(int i = 0; i < nCones; i = i+1)
  {

    shortestDist = std::numeric_limits<float>::infinity();
    // Find closest cone to the last chosen cone
    for(int j = 0; j < nCones; j = j+1)
    {
      if(!((found==j).any()))
      {
        tmpDist = ((current-cones.row(j)).matrix()).norm();
        if(tmpDist < shortestDist)
        {
          shortestDist = tmpDist;
          closestConeIndex = j;
        } // End of if
      } // End of if
    } // End of for

    found(i) = closestConeIndex;
    current = cones.row(closestConeIndex);
  } // End of for
  // Rearrange cones to have the order of found
  for(int i = 0; i < nCones; i = i+1)
  {
    orderedCones.row(i) = cones.row(found(i));
  } // End of for
  return orderedCones;
} // End of orderCones


Eigen::ArrayXXf DetectConeLane::orderAndFilterCones(Eigen::ArrayXXf cones, Eigen::ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones that satisfy some requirements, in order

  int nCones = cones.rows();
  Eigen::ArrayXXf current = vehicleLocation;
  Eigen::ArrayXXi found(nCones,1);
  found.fill(-1);

  float shortestDist, tmpDist, line1, line2, line3, angle;
  int closestConeIndex;
  int nAcceptedCones = 0;

  for(int i = 0; i < nCones; i = i+1)
  {
    shortestDist = std::numeric_limits<float>::infinity();
    closestConeIndex = -1;
    // Find closest cone to the last chosen cone
    for(int j = 0; j < nCones; j = j+1)
    {
      if(!((found==j).any()))
      {
        tmpDist = ((current-cones.row(j)).matrix()).norm();
        if(tmpDist < shortestDist && ((tmpDist < m_coneLengthSeparationThreshold && i>0) || (tmpDist < m_coneLengthSeparationThreshold+3.5f && i<1)) )
        {
          // If it's one of the first two cones, the nearest neighbour is accepted
          if(i < 2)
          {
            shortestDist = tmpDist;
            closestConeIndex = j;
          }
          // Otherwise the nearest neighbour needs to be considered to be placed in roughly the same direction as the two previous cones
          // i.e the angle between the previous line and the next must be larger than pi/2 (it has a forward going component)
          else
          {
            // The angle is found with the cosine rule
            line1 = ((cones.row(found(i-2))-cones.row(found(i-1))).matrix()).norm();
            line2 = ((cones.row(found(i-1))-cones.row(j)).matrix()).norm();
            line3 = ((cones.row(j)-cones.row(found(i-2))).matrix()).norm();
            angle = std::acos((float)(-std::pow(line3,2)+std::pow(line2,2)+std::pow(line1,2))/(2*line2*line1));

            if(std::abs(angle) > m_maxConeAngle)
            {
              shortestDist = tmpDist;
              closestConeIndex = j;
            } // End of if
          } // End of else
        } // End of if
      } // End of if
    } // End of for

    // If no remaining cone was accepted, the algorithm finishes early
    if(closestConeIndex == -1)
    {
std::cout << "Remove invalid cones" << std::endl;
      break;
    } // End of if

    nAcceptedCones = nAcceptedCones+1;
    found(i) = closestConeIndex;
    current = cones.row(closestConeIndex);
  } // End of for

  // Rearrange cones to have the order of found
  Eigen::ArrayXXf orderedCones(nAcceptedCones,2);
  for(int i = 0; i < nAcceptedCones; i = i+1)
  {
    orderedCones.row(i) = cones.row(found(i));
  }

  return orderedCones;
} // End of orderAndFilterCones


Eigen::ArrayXXf DetectConeLane::insertNeededGuessedCones(Eigen::ArrayXXf longSide, Eigen::ArrayXXf shortSide, Eigen::ArrayXXf vehicleLocation, float distanceThreshold, float guessDistance, bool guessToTheLeft)
{
  // Input: Both cone sides, vehicle position, two distance values and if the guesses should be on the left side
  // Output: The new ordered short side with mixed real and guessed cones
  int nConesLong = longSide.rows();
  int nConesShort = shortSide.rows();

  Eigen::ArrayXXf guessedCones(std::max(2*nConesLong-2,0),2); // 2n-2 is the number of guesses if all known cones need guessed matches
  float shortestDist, tmpDist;
  Eigen::ArrayXXf guess(1,2);
  int nGuessedCones = 0;

  // Every long side cone should search for a possible match on the other side
  for(int i = 0; i < nConesLong; i = i+1)
  {
    shortestDist = std::numeric_limits<float>::infinity();
    // Find closest cone on the other side
    for(int j = 0; j < nConesShort; j = j+1)
    {
      tmpDist = ((longSide.row(i)-shortSide.row(j)).matrix()).norm();
      if(tmpDist < shortestDist)
      {
        shortestDist = tmpDist;
     } // End of if
    } // End of for

    // If the closest cone is not valid, create cone guesses perpendicular to both segments connected to the current cone.
    // If it's the first or last cone, there is only on segment available.
    if(shortestDist > distanceThreshold && longSide.rows()>1)
    {
      if(i == 0)
      {
        guess = DetectConeLane::guessCones(longSide.row(0),longSide.row(1),guessDistance,guessToTheLeft,true,false);
        nGuessedCones = nGuessedCones+1;
      }
      else if(i == nConesLong-1)
      {
        guess = DetectConeLane::guessCones(longSide.row(nConesLong-2),longSide.row(nConesLong-1),guessDistance,guessToTheLeft,false,true);
        nGuessedCones = nGuessedCones+1;
      }
      else
      {
        guess = DetectConeLane::guessCones(longSide.row(i-1),longSide.row(i),guessDistance,guessToTheLeft,false,true);
        nGuessedCones = nGuessedCones+1;
        guessedCones.row(nGuessedCones-1) = guess;
        guess = DetectConeLane::guessCones(longSide.row(i),longSide.row(i+1),guessDistance,guessToTheLeft,true,false);
        nGuessedCones = nGuessedCones+1;
      } // End of else

      guessedCones.row(nGuessedCones-1) = guess;
    } // End of if
  } // End of for

  // Collect real and guessed cones in the same array, and order them
  Eigen::ArrayXXf guessedConesFinal = guessedCones.topRows(nGuessedCones);
  Eigen::ArrayXXf realAndGuessedCones(nConesShort+nGuessedCones,2);
  realAndGuessedCones.topRows(nConesShort) = shortSide;
  realAndGuessedCones.bottomRows(nGuessedCones) = guessedConesFinal;

  Eigen::ArrayXXf newShortSide = DetectConeLane::orderCones(realAndGuessedCones, vehicleLocation);
  return newShortSide;
} // End of insertNeededGuessedCones


Eigen::ArrayXXf DetectConeLane::guessCones(Eigen::ArrayXXf firstCone, Eigen::ArrayXXf secondCone, float guessDistance, bool guessToTheLeft, bool guessForFirstCone, bool guessForSecondCone)
{
  // Input: Two neighbouring cones, the guessing distance, if guesses should go to the left or not, and which known cones should
  // get matching guesses
  // Output: Guessed cone positions

  Eigen::ArrayXXf vector = secondCone-firstCone;
  float direction;
  if(guessToTheLeft)
  {
    direction = 1.0;
  }
  else
  {
    direction = -1.0;
  } // End of else

  Eigen::ArrayXXf normal(1,2);
  normal << -vector(1),vector(0);
  normal = normal/((normal.matrix()).norm());
  Eigen::ArrayXXf guessVector = direction*guessDistance*normal;

  // The guess is placed a guessVector away from the cone that should get a mathing guess
  Eigen::ArrayXXf guessedCones(1,2);
  if(guessForFirstCone && !guessForSecondCone)
  {
    guessedCones << firstCone(0)+guessVector(0),firstCone(1)+guessVector(1);
  }
  else if(!guessForFirstCone && guessForSecondCone)
  {
    guessedCones << secondCone(0)+guessVector(0),secondCone(1)+guessVector(1);
  }
  else
  {
    // Should not be in use. Works in matlab but not here were array sizes have to be defined in advance? Throw exception if this is reached?
    std::cout << "WARNING, ENTERED DANGEROUS AREA IN GUESSCONES " << std::endl;
    guessedCones << -1000,-1000;
  } // End of else

  return guessedCones;
} // End of guessCones


float DetectConeLane::findTotalPathLength(Eigen::ArrayXXf sidePoints)
{
  // Input: Cone positions
  // Output: Total length of cone sequence

  int nCones = sidePoints.rows();
  float pathLength = 0;

  float segLength;
  for(int i = 0; i < nCones-1; i = i+1)
  {
    segLength = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
    pathLength = pathLength + segLength;
  }

  return pathLength;
} // End of findTotalPathLength


float DetectConeLane::findFactorToClosestPoint(Eigen::ArrayXXf p1, Eigen::ArrayXXf p2, Eigen::ArrayXXf q)
{
  // Input: The two cones of a cone segment and a reference point
  // Output: The factor to multiply with the vector between the cones in order to reach the point on the segment that has a
  // perpendicular line to the reference point

  Eigen::ArrayXXf v = p2-p1; // The line to follow
  Eigen::ArrayXXf n(1,2);    // The normal
  n << -v(1),v(0);

  float factor = (q(0)-p1(0)+(p1(1)-q(1))*n(0)/n(1))/(v(0)-v(1)*n(0)/n(1));

  return factor;
} // End of findFactorToClosestPoint


void DetectConeLane::sendMatchedContainer(Eigen::ArrayXXf virtualPointsLong, Eigen::ArrayXXf virtualPointsShort)
{
  int nSurfaces = virtualPointsLong.rows()/2;
  //std::cout << "Sending " << nSurfaces << " surfaces" << std::endl;
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

  for(int n = 0; n < nSurfaces; n++)
  {
    opendlv::logic::perception::GroundSurfaceArea surfaceArea;
    surfaceArea.surfaceId(nSurfaces-n-1);
    surfaceArea.x1(virtualPointsLong(2*n,0));
    surfaceArea.y1(virtualPointsLong(2*n,1));
    surfaceArea.x2(virtualPointsShort(2*n,0));
    surfaceArea.y2(virtualPointsShort(2*n,1));
    surfaceArea.x3(virtualPointsLong(2*n+1,0));
    surfaceArea.y3(virtualPointsLong(2*n+1,1));
    surfaceArea.x4(virtualPointsShort(2*n+1,0));
    surfaceArea.y4(virtualPointsShort(2*n+1,1));
    m_od4.send(surfaceArea, sampleTime , m_senderStamp);
    /*std::cout<<"DetectConeLane send surface: "<<" x1: "<<virtualPointsLong(2*n,0)<<" y1: "<<virtualPointsLong(2*n,1)<<" x2: "<<virtualPointsShort(2*n,0)<<" y2: "<<virtualPointsShort(2*n,1)<<" x3: "<<virtualPointsLong(2*n+1,0)<<" y3: "<<virtualPointsLong(2*n+1,1);
    std::cout<<" x4: "<<virtualPointsShort(2*n+1,0)<<" y4 "<<virtualPointsShort(2*n+1,1)<<" frame ID: "<<nSurfaces-n-1<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime)<<" senderStamp "<<m_senderStamp<<std::endl;
    */
  } // End of for

  m_tock = std::chrono::system_clock::now();
  std::chrono::duration<double> dur = m_tock-m_tick;
  m_newClock = true;
  //std::cout<<"DetectConelane Module Time: "<<dur.count()<<std::endl;
} // End of sendMatchedContainer
