/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "detectconelane.hpp"
#include "collector.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<=0) {
    std::cerr << argv[0] << " is a path planner for the CFSD18 project." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=211 --guessDistance=3 --maxConeAngle=1.57 --receiveTimeLimit=0.001 --fakeSlamActivated=1 [more...]" <<  std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    (void)VERBOSE;

    cluon::data::Envelope data;
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Lap{static_cast<uint16_t>(219)};
    DetectConeLane detectconelane(commandlineArguments,od4,od4Lap);
    int gatheringTimeMs = (commandlineArguments.count("gatheringTimeMs")>0)?(std::stoi(commandlineArguments["gatheringTimeMs"])):(60);
    int separationTimeMs = (commandlineArguments.count("separationTimeMs")>0)?(std::stoi(commandlineArguments["separationTimeMs"])):(5);
    bool skidpadMode = (commandlineArguments["useSkidpadMode"].size() != 0) ? (std::stoi(commandlineArguments["useSkidpadMode"])==1) : (false);
    bool accelerationMode = (commandlineArguments["useAccelerationMode"].size() != 0) ? (std::stoi(commandlineArguments["useAccelerationMode"])==1) : (false);
    int nMessageTypes = (accelerationMode)?(2):(3);
    Collector collector(detectconelane,gatheringTimeMs,separationTimeMs,nMessageTypes);
    uint32_t detectconeStamp = (commandlineArguments.count("detectConeId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeId"]))):(118);
    uint32_t slamStamp = (commandlineArguments.count("slamId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["slamId"]))):(120);
    uint32_t attentionStamp = (commandlineArguments.count("attentionId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["attentionId"]))):(116);
    uint32_t simDetectconeStamp = (commandlineArguments.count("simDetectConeId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["simDetectConeId"]))):(231);
    uint32_t yawRateStamp = (commandlineArguments.count("yawRateId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["yawRateId"]))):(112);
    uint32_t gpsId = (commandlineArguments.count("gpsId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["gpsId"]))):(114);
    bool useRawGPS = (commandlineArguments["useRawGPS"].size() != 0) ? (std::stoi(commandlineArguments["useRawGPS"])==1) : (false);
    uint32_t gpsStamp = (useRawGPS) ? (108) : (gpsId);
    uint32_t const speedId1=(commandlineArguments["speedId1"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["speedId1"])) : (0);
    uint32_t const speedId2=(commandlineArguments["speedId2"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["speedId2"])) : (0);
    uint32_t id = (commandlineArguments.count("id")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["id"]))):(211);

    auto poseEnvelope{[&detectconelane,senderStamp = gpsStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          detectconelane.nextPos(envelope);
        }
      }
    };

    auto orangeEnvelope{[&detectconelane, &detectconeStamp, &simDetectconeStamp](cluon::data::Envelope &&envelope)
      {
        uint32_t sender = envelope.senderStamp();
        if(sender == detectconeStamp || sender == simDetectconeStamp){
          detectconelane.nextOrange(envelope);
        }
      }
    };

    auto yawRateEnvelope{[&detectconelane, senderStamp = yawRateStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          detectconelane.nextYawRate(envelope);
        }
      }
    };

    auto groundSpeedEnvelope{[&detectconelane, speedId1, speedId2](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == speedId1 || envelope.senderStamp() == speedId2){
          detectconelane.nextGroundSpeed(envelope);
        }
      }
    };

    auto pointEnvelope{[&attentionStamp, &simDetectconeStamp, &collector](cluon::data::Envelope &&envelope)
      {
        uint32_t sender = envelope.senderStamp();
        if(sender == attentionStamp || sender == simDetectconeStamp){
          collector.CollectCones(envelope);
        }
      }
    };

    auto coneEnvelope{[&detectconeStamp, &slamStamp, &simDetectconeStamp ,&collector](cluon::data::Envelope &&envelope)
      {
        uint32_t sender = envelope.senderStamp();
        if(sender == slamStamp){
          detectconeStamp = slamStamp;
        }
        if(sender == detectconeStamp || sender == simDetectconeStamp){
          collector.CollectCones(envelope);
        }
      }
    };

    if(accelerationMode){
      // Direction and distance from attention or simulation. Ground speed and yaw rate from UKF.
      od4Lap.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),groundSpeedEnvelope);
      od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),pointEnvelope);
      od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),pointEnvelope);
      od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(),yawRateEnvelope);
      od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),groundSpeedEnvelope);
    }else{
      // Direction, distance and type from either detectcone, slam or simulation
      od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),coneEnvelope);
      od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),coneEnvelope);
      od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),coneEnvelope);
      // Number of orange cones in frame from detectcone
      od4.dataTrigger(opendlv::logic::perception::Object::ID(),orangeEnvelope);
    }
    if(!skidpadMode){
      // GPS
      od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(),poseEnvelope);
    }

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
      cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
      opendlv::system::SignalStatusMessage readySignal;
      readySignal.code(1);
      od4.send(readySignal, sampleTime, id);
    }
  }
  return retCode;
}
