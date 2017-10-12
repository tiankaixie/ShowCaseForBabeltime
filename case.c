/*
  This file is one the show cases for what I do currently -- teaching arduino
  programming using Vortex. Basically, it is based on C programming, but we
  developed our own library and let students call the API like IRSensor, etc.

  The reason I would like to show this code is because I think it is partially
  related to the gaming. Basically this code lets Vortex bot (a small robot)
  run a simple maze. We use state machine concepts to let robot move according
  to specific situation. By using grid search strategy, the robot judge move or
  make turn (including how many degrees to turn) or move back when get stuck,
  etc. I believe in game programming, we need the similar solutions for game
  objects.

*/


#include <PWMMotor.h>
#include <IRSensor.h>
#include <InterruptEncoder.h>
#include <NaviUtility.h>
#include <GridSearchRun.h>

const int disSize = 6;
int nMazeStep =0;
float angleTolerance = 6.0;
int obstacleThreshold= 5;
int gridSearchSize = 8;
float axleLength = 170;
int lastDistance = -10000;
long lastTimeStamp = 0;
float turnAngle = 0.0;
int turnCount = 0;
int turnCountLimit = 6;

GridSearchRun * gridSearch = 0;
IRSensor * irsensor = 0;
long count = 0;

ISR(PCINT2_vect){           //motor encoder interrupt
  count++;
}

PWMMotor * leftMotor =0;
PWMMotor * rightMotor = 0;

int state = 0;

long interruptLeftEncoder = 0;
int leftEncoderPin = 0;
long interruptRightEncoder = 0;
int rightEncoderPin = 1;

void forwardLeftEncoderCallBack() {
  interruptLeftEncoder++;
}

void forwardRightEncoderCallBack() {
  interruptRightEncoder++;
}

void backwardLeftEncoderCallBack() {
  interruptLeftEncoder--;
}

void backwardRightEncoderCallBack() {
  interruptRightEncoder--;
}

int turn(float angle) {

  int a = getTurnAngle(leftMotor->getPosition(), rightMotor->getPosition(), 13);

  if (abs(a - angle) < angleTolerance)
  {
    leftMotor->setPower(0);
    rightMotor->setPower(0);
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
    return 0;
  } else if (a - angle < 0) {
    // turn right
    leftMotor->setPower(90);
    rightMotor->setPower(-90);
    turnCount ++;
    if (turnCount > turnCountLimit)
    {
      return 0;
    }
  } else {
    // turn left
    leftMotor->setPower(-90);
    rightMotor->setPower(90);
    turnCount ++;
    if (turnCount > turnCountLimit)
    {
      return 0;
    }

  }
  return 1;
}

bool action() {
  switch (state)
  {
    case 0: {
      // move forward
      interruptLeftEncoder = 0;
      interruptRightEncoder = 0;

      leftMotor->setPower(150);
      rightMotor->setPower(150);
      lastTimeStamp = millis();

      state = 1;
      break;
    }
    case 1: {
      // detect jamming every 2 seconds
      if ( millis() - lastTimeStamp > 2000 ) {
        int distance = max(leftMotor->getPosition(), rightMotor->getPosition());
        if (distance - lastDistance < 4) {
          // jammed
          state = 5;
        }
        lastDistance = distance;
        lastTimeStamp = millis();
      } else {
        state = 2;
      }
      break;
    }
    case 2: {
      // detect obstable
      if ( irsensor->measure() == 0 ) {
        if (irsensor->leftCount > obstacleThreshold || irsensor->rightCount > obstacleThreshold) {
          state = 3; // found obstacle
          // set up searching
          gridSearch->setTurnPower(90);
          gridSearch->setAngleTolerance(10);
          gridSearch->reset();
          gridSearch->generateGrid(-90.0, 90.0);
        } else {
          state = 0;
        }
      }
      break;
    }
    case 3: {
      // find exits
      if (gridSearch->run() == 0) {
        turnAngle = gridSearch->getBestAngle();
        state = 4;
        turnCount = 0;
      }
      break;
    }
    case 4: {
      // turn to the most likely exit
      if ( turn(turnAngle) ==0 ) {
        state = 0; //  stop turning
      }
      break;
    }
    case 5: {
      // avoid jamming
      Serial.println(millis() - lastTimeStamp);
      if ( millis() - lastTimeStamp < 5000 ) {
        leftMotor->setPower(-100);
        rightMotor->setPower(-100);
      } else {
        leftMotor->resetEncoder();
        rightMotor->resetEncoder();
        lastDistance = max(leftMotor->getPosition(), rightMotor->getPosition());
        lastTimeStamp = millis();
        state =0;
      }
      break;
    }
  }
  return false;
}

void setup(){
  //setup motor
  leftMotor = new PWMMotor(9,5,false);
  leftMotor->setEncoder(leftEncoderPin,&interruptLeftEncoder,forwardLeftEncoderCallBack,backwardLeftEncoderCallBack);
  rightMotor = new PWMMotor(10,6,false);
  rightMotor->setEncoder(rightEncoderPin,&interruptRightEncoder,forwardRightEncoderCallBack,backwardRightEncoderCallBack);

  //setup IR sensor
  irsensor = new IRSensor(8, 12, 7, &count);

  // grid search
  gridSearch = new GridSearchRun(leftMotor, rightMotor, irsensor, gridSearchSize, axleLength, angleTolerance);

  delay(5000);

  Serial.begin(9600);
}

void loop(void){
  action();
}

void cleanup() {
  if (gridSearch) delete gridSearch;
  if (irsensor) delete irsensor;
  if (leftMotor) delete leftMotor;
  if (rightMotor) delete rightMotor;
}
