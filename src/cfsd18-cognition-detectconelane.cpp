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
    DetectConeLane detectconelane(commandlineArguments,od4);
    int gatheringTimeMs = (commandlineArguments.count("gatheringTimeMs")>0)?(std::stoi(commandlineArguments["gatheringTimeMs"])):(60);
    int separationTimeMs = (commandlineArguments.count("separationTimeMs")>0)?(std::stoi(commandlineArguments["separationTimeMs"])):(5);
    Collector collector(detectconelane,gatheringTimeMs,separationTimeMs,3);
    uint32_t detectconeStamp = (commandlineArguments.count("detectConeId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeId"]))):(231); //231: Simulation is default
    uint32_t id = (commandlineArguments.count("id")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["id"]))):(211);

    auto coneEnvelope{[senderStamp = detectconeStamp,&collector](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          collector.CollectCones(envelope);
        }
      }
    };
    od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),coneEnvelope);

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
