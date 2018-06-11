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
#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>

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

    auto envelopeRecieved{[&laner = detectconelane](cluon::data::Envelope &&envelope)
      {
        laner.nextContainer(envelope);
      }
    };
    od4.dataTrigger(opendlv::logic::perception::ObjectProperty::ID(),envelopeRecieved);
    od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),envelopeRecieved);
    od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),envelopeRecieved);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),envelopeRecieved);

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    while (od4.isRunning()) {
      std::this_thread::sleep_for(1s);
      std::chrono::system_clock::time_point tp;
    }
  }
  return retCode;
}
