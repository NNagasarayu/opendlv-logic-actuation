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

#include <opencv2/imgcodecs/imgcodecs.hpp> //Remove
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <math.h>
#include <mutex>
#include <vector>
#include <string>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
       (0 == commandlineArguments.count("freq")) ||
       (0 == commandlineArguments.count("dist-to-curve")) ||
       (0 == commandlineArguments.count("sender")) ) {
    std::cerr << argv[0] << " takes it inputs from a UDP conference." << std::endl;
    std::cerr << "Usage:   " << argv[0] << "--cid=<OD4 session> --freq=<frequency at which message is sent> --dist-to-curve=<distance to cone wall to classify as curve> --sender=<sender stamp to use> [--verbose]" << std::endl;
    std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
    std::cerr << "         --freq:   The frequency for seneding new lane responses" << std::endl;
    std::cerr << "         --dist-to-curve:   The distance in pixels used to find wall infront" << std::endl;
    std::cerr << "         --sender: The senderstamp of groundsteer and pedal request" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --freq=10 --dist-to-curve=100 --sender=0 --verbose" << std::endl;
  } else {
    const uint32_t freq{static_cast<uint32_t>(
      std::stoi(commandlineArguments["freq"]))};
    float const curveWallDist{std::stof(commandlineArguments["dist-to-curve"])};
    const uint32_t sender{static_cast<uint32_t>(
      std::stoi(commandlineArguments["sender"]))};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    
    cluon::OD4Session od4{static_cast<uint16_t>(
      std::stoi(commandlineArguments["cid"]))};
    
    //Initialize variables
    std::mutex coneAimPointMutex;
    float coneAimPointDistance{0.0f};
    std::vector<float> moveAvgAngle;
    uint32_t avgSize{5};
    uint32_t indexAvg{avgSize-1};
    uint32_t calibrate{0};
    uint32_t calibrateRounds{5};
    bool intersection{false};
    
    std::mutex kiwiMutex;
    int32_t kiwiX{0};
    int32_t kiwiY{-1000};
    uint32_t roundsWithoutKiwi{0};
    bool seenKiwi{false};
    double kiwiAngle;
    
    std::mutex distancesMutex;
    float front{100};
    
    uint32_t roundsToWaitAtIntersection{5};
    uint32_t waitCounter{0};
    
    auto onAimPoint = [&coneAimPointMutex, &coneAimPointDistance, &calibrate,
      &calibrateRounds, &moveAvgAngle, &indexAvg, &avgSize, &intersection,
      &VERBOSE](cluon::data::Envelope &&env)
    {
      auto senderStamp = env.senderStamp();
      opendlv::logic::action::AimPoint ap =
        cluon::extractMessage<opendlv::logic::action::AimPoint> 
        (std::move(env));
      
      {
        std::lock_guard<std::mutex> lock(coneAimPointMutex);
        
        if (senderStamp==0 || senderStamp ==1) {
          coneAimPointDistance = ap.distance();
          
          indexAvg = (indexAvg + 1)%avgSize;
          if (moveAvgAngle.size()<avgSize) {
            moveAvgAngle.push_back(ap.azimuthAngle());
          } else {
            moveAvgAngle[indexAvg] = ap.azimuthAngle();
          }
        }
        
      }
      intersection = senderStamp==1;
      
      if (calibrate < calibrateRounds) {
        calibrate++;
      }
      
      if (VERBOSE) {
        std::cout << "recieved cone aim point: angle="<< moveAvgAngle[indexAvg] <<
          ", distance=" << coneAimPointDistance << std::endl;
      }
    };
    
    auto onKiwi = [&kiwiMutex, &kiwiX, &kiwiY, &seenKiwi, &roundsWithoutKiwi,
      &VERBOSE](cluon::data::Envelope &&env)
    {
      auto senderStamp = env.senderStamp();
      opendlv::logic::perception::KiwiPosition kp =
        cluon::extractMessage<opendlv::logic::perception::KiwiPosition> 
        (std::move(env));
      
      {
        std::lock_guard<std::mutex> lock(kiwiMutex);
        if (senderStamp==0) {
          kiwiX = kp.x();
          kiwiY = kp.y();
        }
      }
      if (kiwiY>-999) {
        seenKiwi=true;
        roundsWithoutKiwi=0;
      } else {
        roundsWithoutKiwi++;
      }
      
    };
    
    auto onDistance = [&distancesMutex, &front, &VERBOSE
      ](cluon::data::Envelope &&env)
    {
      
      auto senderStamp = env.senderStamp();
      opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

      // Store distance readings.
      std::lock_guard<std::mutex> lck(distancesMutex);
      if (senderStamp==0){front = dr.distance();}
      
      if (VERBOSE) {
          std::cout << "recieved distance readings: front="<< front << std::endl;
      }
    };
    
    auto atFrequency{[&coneAimPointMutex, &moveAvgAngle, &coneAimPointDistance,
      &calibrate, &calibrateRounds, &distancesMutex, &front, &curveWallDist,
      &sender, &od4, &VERBOSE, &kiwiX, &kiwiY, &kiwiAngle, &seenKiwi,
      &roundsWithoutKiwi, &intersection, &roundsToWaitAtIntersection, 
      &waitCounter]() -> bool
    {
      const float maxSpeed{0.12f};
    
      // Part I: Lane detetion
      float pedalPositionCones{maxSpeed};
      float groundSteerCones{0.0f};
      {
        float yawError{0.0f};
        float distance;
        {
          std::lock_guard<std::mutex> lock(coneAimPointMutex);
          for (uint32_t i=0; i<moveAvgAngle.size(); i++) {
            yawError += moveAvgAngle[i];
          }
          yawError = yawError/moveAvgAngle.size();
          distance = coneAimPointDistance;
        }
        
        pedalPositionCones = maxSpeed;
        groundSteerCones = yawError/12.0f;
        
        if (distance < curveWallDist){
          // 0 if no "cone wall", 1 directly in fron of us
          float speedFactor = (curveWallDist - distance)/curveWallDist;
          pedalPositionCones = 0.02f*(speedFactor) + pedalPositionCones*(
            1.0f-speedFactor);
        }
        
        if (calibrate < calibrateRounds) {
          pedalPositionCones = 0.0f;
        }
      }
      
      // Part II: Front sensor
      float pedalPositionFront{maxSpeed};
      {
        {
          std::lock_guard<std::mutex> lock(distancesMutex);
          if (front <0.1f) {
            pedalPositionFront = -0.5f;
          } else if (front < 0.2f) {
            pedalPositionFront = 0.0f;
          }
        }
      }
      
      //Part III: Kiwi detection
      float pedalPositionKiwi{maxSpeed};
      {
        if (intersection) {
          float intersectionSpeed = 0.09f;
          if (waitCounter < roundsToWaitAtIntersection) {
            pedalPositionKiwi=0.0f;
          } else if (!seenKiwi){
            pedalPositionKiwi = intersectionSpeed;
          } else {
            if (roundsWithoutKiwi==0){
              kiwiAngle = std::atan2(-kiwiX, kiwiY);
            }
            if (roundsWithoutKiwi>1 || kiwiAngle > static_cast<float>(M_PI/4)){
              // if we don't see a kiwi or if it is comming from the left,
              // go ahead
              pedalPositionKiwi = intersectionSpeed;
            } else {
              // There is an kiwi straight ahead or to the right. Wait for it 
              // to pass.
              pedalPositionKiwi = 0.0f;
            }
          }
          waitCounter++;
        } else {
          waitCounter=0;
          if (seenKiwi){
            if (roundsWithoutKiwi == 0) {
              kiwiAngle = std::atan2(-kiwiX, kiwiY);
              if (kiwiY<50) {
                pedalPositionKiwi = 0.0f;
              } else if (kiwiY<150) {
                float speedFactor{(kiwiY-50)/(150.0f-50.0f)}; //1 if 150 0 if 50
                //Quickly decline speed toward zero due to frame rate of yolo
                pedalPositionKiwi = maxSpeed*static_cast<float>(
                  std::pow(speedFactor, 1.5));
              }
            }
            if (roundsWithoutKiwi < 2 && std::abs(kiwiAngle)<M_PI/4) {
              // Kiwi last seen infront of kiwi (within 30 degrees),
              // probaly due to miss classification, trust front distance sensor
              if (front < 0.1f) {
                // To close, break
                pedalPositionKiwi = -0.5f;
              } else if (front < 0.5f) {
                pedalPositionKiwi = maxSpeed*(front-0.1f)/(0.4f);
              } else {
                pedalPositionKiwi = maxSpeed;
              }
            }
          }
        }
      }
      
      float pedalPosition;
      if (calibrate == 0 && !seenKiwi) {
        pedalPosition=0.0f;
      } else if (calibrate == 0) {
        pedalPosition = std::min(pedalPositionKiwi, pedalPositionFront);
      } else if (!seenKiwi) {
        pedalPosition = std::min(pedalPositionCones, pedalPositionFront);
      } else {
        pedalPosition = std::min(pedalPositionCones, pedalPositionKiwi);
        pedalPosition = std::min(pedalPosition, pedalPositionFront);
      }
      float groundSteer=groundSteerCones;
      
      // Steering; range: +38deg (left) .. -38deg (right). (0.66rad)
      cluon::data::TimeStamp sampleTime;
      opendlv::proxy::GroundSteeringRequest gsr;
      gsr.groundSteering(groundSteer); //TODO: change back to 0
      od4.send(gsr, sampleTime, sender);

      // accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
      opendlv::proxy::PedalPositionRequest ppr;
      ppr.position(pedalPosition); //TODO: change back to 0
      od4.send(ppr, sampleTime, sender);
      
      
      if (VERBOSE) {
        std::cout << "sending stering commands: pedal position="<< 
          ppr.position() << ", ground steering=" << gsr.groundSteering() << 
          std::endl;
      }
      
      return true;
    }};
    
    od4.dataTrigger(opendlv::logic::action::AimPoint::ID(), onAimPoint);
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);
    od4.dataTrigger(opendlv::logic::perception::KiwiPosition::ID(), onKiwi);
    
    // Register the time trigger, spawning a thread that blocks execution 
    // until CTRL-C is pressed
    od4.timeTrigger(freq, atFrequency);
    retCode = 0;
  }
  return retCode;
}

