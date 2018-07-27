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
#include <cmath>
#include <map>
#include <chrono>
#include <mutex>
typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

class DetectConeLane {
 public:
  DetectConeLane(std::map<std::string, std::string>, cluon::OD4Session &od4);
  DetectConeLane(DetectConeLane const &) = delete;
  DetectConeLane &operator=(DetectConeLane const &) = delete;
  virtual ~DetectConeLane();
  void nextPos(cluon::data::Envelope);
  void nextOrange(cluon::data::Envelope);
  void receiveCombinedMessage(std::map<int,ConePackage>, cluon::data::TimeStamp, uint32_t);

 private:
  void setUp();
  void tearDown();

  void sortIntoSideArrays(Eigen::ArrayXXf, int, int, int, int, int);
  void generateSurfaces(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::ArrayXXf Spherical2Cartesian(float, float, float);
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

  cluon::OD4Session &m_od4;
  int m_senderStamp;
  uint32_t m_detectconeStamp;
  uint32_t m_slamStamp;
  bool m_alwaysSlam;
  bool m_slamActivated;
  bool m_accelerationMode;
  bool m_skidpadMode;
  bool m_isRunning;
  int m_lapCounter;
  int m_nLapsToGo;
  int m_lapCounterLockTime;
  std::chrono::time_point<std::chrono::system_clock> m_latestLapIncrease;
  int m_nOrange;
  bool m_orangeVisibleInLatestFrame;
  bool m_useNewConeLapCounter;
  bool m_countOrangeFrames;
  bool m_countDisappearanceFrames;
  int m_orangeFramesInARow;
  int m_disappearanceFramesInARow;
  int m_nMinOrangeFrames;
  int m_nMinDisappearanceFrames;
  float m_guessDistance;
  float m_minGuessSeparation;
  bool m_latePerpGuessing;
  float m_maxConeAngle;
  float m_behindMemoryDistance;
  float m_maxConeWidthSeparation;
  float m_widthSeparationMargin;
  float m_maxConeLengthSeparation;
  float m_lengthSeparationMargin;
  Eigen::Vector2d m_globalPos;
  bool m_finishFound;
  Eigen::Vector2d m_finishPos;
  double m_finishRadius;
  bool m_nearFinishInLatestFrame;
  bool m_globalPosReceived;
  bool m_noConesReceived;
  std::chrono::time_point<std::chrono::system_clock> m_tick;
  std::chrono::time_point<std::chrono::system_clock> m_tock;
  cluon::data::TimeStamp m_sampleTime;
  bool m_newClock;
  std::mutex m_posMutex;
  std::mutex m_orangeMutex;
  std::mutex m_timeStampMutex;
  std::mutex m_sendMutex;
  const double DEG2RAD = 0.017453292522222; // PI/180.0

};


#endif
