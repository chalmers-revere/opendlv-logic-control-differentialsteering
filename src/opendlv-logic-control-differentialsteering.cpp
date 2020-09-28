/*
 * Copyright (C) 2019 Ola Benderius
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

#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
       (0 == commandlineArguments.count("freq")) ||
       (0 == commandlineArguments.count("speed-max")) ||
       (0 == commandlineArguments.count("track-width")) ) {
    std::cerr << argv[0] << " controls a differentially steered vehicle by "
      << "controlling two independent motors, one on each side." << std::endl
      << "Usage:   " << argv[0] << " --cid=<CID> --freq=<Frequency to send> "
      << "--speed-max=<Maximum speed> --track-width=<Track width> "
      << "[--id-input=<Sender stamp, input message (default 0)>] "
      << "[--id-left=<Sender stamp, left motor (default 0)>] "
      << "[--id-right=<Sender stamp, right motor (default 1)>] [--verbose]" 
      << std::endl
      << "Example: " << argv[0] << " --cid=111" << std::endl;
  } else {
    uint32_t const senderStampInput{
      (commandlineArguments.count("id-input") != 0) ?
        static_cast<uint32_t>(std::stoi(commandlineArguments["id-input"])) : 0};
    uint32_t const senderStampLeft{
      (commandlineArguments.count("id-left") != 0) ?
        static_cast<uint32_t>(std::stoi(commandlineArguments["id-left"])) : 0};
    uint32_t const senderStampRight{
      (commandlineArguments.count("id-right") != 0) ?
        static_cast<uint32_t>(std::stoi(commandlineArguments["id-right"])) : 1};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    float const trackWidth{std::stof(commandlineArguments["track-width"])};
    float const speedMax{std::stof(commandlineArguments["speed-max"])};

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex requestMutex;
    float vxRequest{0.0f};
    float yawRateRequest{0.0f};

    std::mutex stateMutex;
    int32_t state;

    auto onGroundMotionRequest{[&vxRequest, &yawRateRequest, &senderStampInput,
      &requestMutex, &verbose](cluon::data::Envelope &&envelope)
      {
        if (envelope.senderStamp() == senderStampInput) {

          auto msg = cluon::extractMessage<opendlv::proxy::GroundMotionRequest>(
              std::move(envelope));

          {
            std::lock_guard<std::mutex> lock(requestMutex);
            vxRequest = msg.vx();
            yawRateRequest = msg.yawRate();
          }
    
          if (verbose) {
            std::cout << "Got request, vx=" << msg.vx() << " yawRate=" 
              << msg.yawRate() << std::endl;
          }
        }
      }};
    
    auto onSwitchStateRequest{[&state, &stateMutex, &verbose](cluon::data::Envelope &&envelope)
      {
        if (envelope.senderStamp() == 99) {

          auto msg = cluon::extractMessage<opendlv::proxy::SwitchStateRequest>(
              std::move(envelope));

          std::lock_guard<std::mutex> lock(stateMutex);
          state = msg.state();

          if (verbose) {
            std::cout << "Got switch state, state=" << state << std::endl;
          }
        }
      }};
    
    auto atFrequency{[&od4, &trackWidth, &speedMax, &vxRequest, &yawRateRequest,
      &senderStampLeft, &senderStampRight, &requestMutex, &state, &stateMutex,
      &verbose]() -> bool
      {
        {
          std::lock_guard<std::mutex> lock(stateMutex);
          if (state == 1) {
            if (verbose) {
              std::cout << "Not in state '1', supressing output" << std::endl;
            }
            return true;
          }
        }

        float vl;
        float vr;
        {
          std::lock_guard<std::mutex> lock(requestMutex);
          vl = vxRequest - yawRateRequest * trackWidth;
          vr = vxRequest + yawRateRequest * trackWidth;
        }
        cluon::data::TimeStamp ts = cluon::time::now();
        float pedalPositionLeft;
        float pedalPositionRight;
        {
          pedalPositionLeft = vl / speedMax;
          pedalPositionLeft = pedalPositionLeft > 1.0f ? 0.99f 
            : pedalPositionLeft;
          pedalPositionLeft = pedalPositionLeft < -1.0f ? -0.99f 
            : pedalPositionLeft;
          opendlv::proxy::PedalPositionRequest pedalPositionRequestLeft;
          pedalPositionRequestLeft.position(pedalPositionLeft);
          od4.send(pedalPositionRequestLeft, ts, senderStampLeft);
        }
        {
          pedalPositionRight = vr / speedMax;
          pedalPositionRight = pedalPositionRight > 1.0f ? 0.99f 
            : pedalPositionRight;
          pedalPositionRight = pedalPositionRight < -1.0f ? -0.99f 
            : pedalPositionRight;
          opendlv::proxy::PedalPositionRequest pedalPositionRequestRight;
          pedalPositionRequestRight.position(pedalPositionRight);
          od4.send(pedalPositionRequestRight, ts, senderStampRight);
        }
    
        if (verbose) {
          std::cout << "Sending pedal requests, left=" << pedalPositionLeft
            << " right=" << pedalPositionRight << " (wants to go vl=" << vl
            << " vr=" << vr << ", max is " << speedMax << ")." << std::endl;
        }

        return true;
      }};

    od4.dataTrigger(opendlv::proxy::GroundMotionRequest::ID(),
        onGroundMotionRequest);
    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);

    retCode = 0;
  }
  return retCode;
}

