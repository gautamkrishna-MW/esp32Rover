#define DEBUG_BUILD

#include "roverClass.h"
#include "commandQueue.h"

std::unique_ptr<roverClass> rover;
std::unique_ptr<RoverCommandQueue> cmdQueue;
// std::string testInpStr("F 30 25\nB 50 50\nL 30 75\nR 30 100\nS 30 50\n");
std::string testInpStr("F 30 25\nF 30 50\nF 30 75\nF 30 100\n");

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // activates the serial communication
  rover = std::make_unique<roverClass>() ;
  cmdQueue = std::make_unique<RoverCommandQueue>();
  cmdQueue->readFromString(testInpStr);
}

void loop() {

  // cmdQueue.readFromSerial(Serial);
  if (cmdQueue->hasCommand()) {
      RoverCommand cmdStruct = cmdQueue->getNextCommand();
      switch(cmdStruct.cmd) {
        case Command::FORWARD:
          Serial.printf("Forward -> Count: %d Speed: %d\n", cmdStruct.pulseCount, cmdStruct.speed);
          rover->forward(cmdStruct.pulseCount, cmdStruct.speed);
          break;
        case Command::BACKWARD:
          Serial.printf("Reverse -> Count: %d Speed: %d\n", cmdStruct.pulseCount, cmdStruct.speed);
          rover->reverse(cmdStruct.pulseCount, cmdStruct.speed);
          break;
        case Command::LEFT:
          Serial.printf("Left -> Count: %d Speed: %d\n", cmdStruct.pulseCount, cmdStruct.speed);
          rover->left(cmdStruct.pulseCount, cmdStruct.speed);
          break;
        case Command::RIGHT:
          Serial.printf("Right -> Count: %d Speed: %d\n", cmdStruct.pulseCount, cmdStruct.speed);
          rover->right(cmdStruct.pulseCount, cmdStruct.speed);
          break;
        case Command::STOP:
          Serial.printf("Stop\n");
          rover->stop();
          break;
        case Command::INVALID:
          Serial.println("Invalid Motion");
          break;
      }
      delay(1000);

      // int k = rand() % 5;
      // if (k == 0)
      //   testInpStr = std::string("F 30 50\n");
      // else if (k == 1)
      //   testInpStr = std::string("B 30 50\n");
      // else if (k == 2)
      //   testInpStr = std::string("L 30 50\n");
      // else if (k == 3)
      //   testInpStr = std::string("R 30 50\n");
      // else if (k == 4)
      //   testInpStr = std::string("S 30 50\n");
      cmdQueue->readFromString(testInpStr);
  }

  // if (rover->isSensorCalibrated())
  //   Serial.println("Sensor calibrated!!");
  // else
  //   Serial.println("Sensor still calibrating...");

  // rover->getEulerOrientation();
  // Serial.printf("Orientation: X: %f, Y: %f, Z: %f\n", rover->orientation[0], rover->orientation[1], rover->orientation[2]);
}
