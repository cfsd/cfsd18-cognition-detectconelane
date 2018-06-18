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

#ifndef CFSD18_COGNITION_DETECTCONELANE_HPP
#define CFSD18_COGNITION_DETECTCONELANE_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include <map>
#include <chrono>
#include <mutex>
#include <condition_variable>
typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

class DetectConeLane {
 public:
  DetectConeLane(std::map<std::string, std::string>, cluon::OD4Session &od4);
  DetectConeLane(DetectConeLane const &) = delete;
  DetectConeLane &operator=(DetectConeLane const &) = delete;
  virtual ~DetectConeLane();
  void receiveCombinedMessage(std::map<int,ConePackage>);
  virtual void nextContainer(cluon::data::Envelope &);

 private:
  void setUp();
  void tearDown();

  void initializeCollection();
  void sortIntoSideArrays(Eigen::MatrixXd, int, int, int, int);
  void generateSurfaces(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::MatrixXd Spherical2Cartesian(double, double, double);
  void findSafeLocalPath(Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::ArrayXXf placeEquidistantPoints(Eigen::ArrayXXf, bool, int, float);
  Eigen::ArrayXXf traceBackToClosestPoint(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::ArrayXXf orderCones(Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::ArrayXXf orderAndFilterCones(Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::ArrayXXf insertNeededGuessedCones(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf, float, float, bool);
  Eigen::ArrayXXf guessCones(Eigen::ArrayXXf, Eigen::ArrayXXf, float, bool, bool, bool);
  float findTotalPathLength(Eigen::ArrayXXf);
  float findFactorToClosestPoint(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf);
  void sendMatchedContainer(Eigen::ArrayXXf, Eigen::ArrayXXf);

  std::mutex m_stateMutex;
  cluon::OD4Session &m_od4;
  int m_senderStamp;
  bool m_fakeSlamActivated;
  float m_guessDistance;
  float m_maxConeAngle;
  float m_coneWidthSeparationThreshold;
  float m_coneLengthSeparationThreshold;
  float m_receiveTimeLimit;
  bool m_newFrame;
  bool m_directionOK;
  bool m_distanceOK;
  bool m_runOK;
  std::map< double, float > m_directionFrame;
  std::map< double, float > m_distanceFrame;
  std::map< double, int > m_typeFrame;
  std::map< double, float > m_directionFrameBuffer;
  std::map< double, float > m_distanceFrameBuffer;
  std::map< double, int > m_typeFrameBuffer;
  int m_lastDirectionId;
  int m_lastDistanceId;
  int m_lastTypeId;
  bool m_newDirectionId;
  bool m_newDistanceId;
  bool m_newTypeId;
  std::chrono::time_point<std::chrono::system_clock> m_directionTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_distanceTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_typeTimeReceived;
  uint64_t m_nConesInFrame;
  int m_objectPropertyId;
  int m_directionId;
  int m_distanceId;
  int m_typeId;
  std::mutex m_directionMutex = {};
  std::mutex m_distanceMutex = {};
  std::mutex m_typeMutex = {};
  int m_surfaceId;
  std::chrono::time_point<std::chrono::system_clock> m_tick;
  std::chrono::time_point<std::chrono::system_clock> m_tock;
  bool m_newClock;
  std::mutex m_sendMutex;
  const double DEG2RAD = 0.017453292522222; // PI/180.0

};


#endif
