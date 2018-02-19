/*
 * KCurrentState.cpp
 *
 *  Created on: 15 maj 2014
 *      Author: MattLech
 */

#include "KCurrentState.h"
#include "FarmbotSensor.h"

static KCurrentState *instance;

FarmbotSensor FarmbotSensor;

long x = 0;
long y = 0;
long z = 0;
unsigned int speed = 0;
bool endStopState[3][2];
long Q = 0;
int lastError = 0;

KCurrentState *KCurrentState::getInstance()
{
  if (!instance)
  {
    instance = new KCurrentState();
  };
  return instance;
};

KCurrentState::KCurrentState()
{
  x = 0;
  y = 0;
  z = 0;
  speed = 0;
  Q = 0;
  lastError = 0;
}

long KCurrentState::getAX()
{
  return FarmbotSensor.ax;
}

long KCurrentState::getAY()
{
  return FarmbotSensor.ay;
}

long KCurrentState::getAZ()
{
  return FarmbotSensor.az;
}
long KCurrentState::getGX()
{
  return FarmbotSensor.gx;
}

long KCurrentState::getGY()
{
  return FarmbotSensor.gy;
}

long KCurrentState::getGZ()
{
  return FarmbotSensor.gz;
}
long KCurrentState::getMX()
{
  return FarmbotSensor.mx;
}

long KCurrentState::getMY()
{
  return FarmbotSensor.my;
}

long KCurrentState::getMZ()
{
  return FarmbotSensor.mz;
}

long KCurrentState::getLat()
{
  return FarmbotSensor.latitude;
}

long KCurrentState::getLog()
{
  return FarmbotSensor.longitude;
}

long KCurrentState::getSpeed()
{
  return FarmbotSensor.speed;
}

long KCurrentState::getRoll()
{
  return FarmbotSensor.roll_deg;
}

long KCurrentState::getPitch()
{
  return FarmbotSensor.pitch_deg;
}

long KCurrentState::getHeading()
{
  return FarmbotSensor.heading_deg;
}

long *KCurrentState::getPoint()
{
  //static long currentPoint[3] = {x, y, z};
  //return currentPoint;
}




void KCurrentState::setAX(long newAX)
{
  FarmbotSensor.ax = newAX;
}

void KCurrentState::setAY(long newAY)
{
  FarmbotSensor.ay = newAY;
}

void KCurrentState::setAZ(long newAZ)
{
  FarmbotSensor.az = newAZ;
}

void KCurrentState::setGX(long newGX)
{
  FarmbotSensor.gx = newGX;
}

void KCurrentState::setGY(long newGY)
{
  FarmbotSensor.gy = newGY;
}

void KCurrentState::setGZ(long newGZ)
{
  FarmbotSensor.gz = newGZ;
}


void KCurrentState::setMX(long newMX)
{
  FarmbotSensor.mx = newMX;
}

void KCurrentState::setMY(long newMY)
{
  FarmbotSensor.my = newMY;
}

void KCurrentState::setMZ(long newMZ)
{
  FarmbotSensor.mz = newMZ;
}

void KCurrentState::setLat(long newLat)
{
  FarmbotSensor.latitude = newLat;
}

void KCurrentState::setLog(long newLog)
{
  FarmbotSensor.longitude = newLog;
}

void KCurrentState::setSpeed(long newSpeed)
{
  FarmbotSensor.speed = newSpeed;
}

void KCurrentState::setRoll(long newRoll)
{
  FarmbotSensor.roll_deg = newRoll;
}

void KCurrentState::setPitch(long newPitch)
{
  FarmbotSensor.pitch_deg = newPitch;
}

void KCurrentState::setHeading(long newHeading)
{
  FarmbotSensor.heading_deg = newHeading;
}







int KCurrentState::getLastError()
{
  return lastError;
}

void KCurrentState::setLastError(int error)
{
  lastError = error;
}

void KCurrentState::setEndStopState(unsigned int axis, unsigned int position, bool state)
{
  endStopState[axis][position] = state;
}

void KCurrentState::setStepsPerMm(long stepsX, long stepsY, long stepsZ)
{
  stepsPerMmX = stepsX;
  stepsPerMmY = stepsY;
  stepsPerMmZ = stepsZ;
}

void KCurrentState::storeEndStops()
{
/* From Frambot
  KCurrentState::getInstance()->setEndStopState(0, 0, digitalRead(X_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(0, 1, digitalRead(X_MAX_PIN));
  KCurrentState::getInstance()->setEndStopState(1, 0, digitalRead(Y_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(1, 1, digitalRead(Y_MAX_PIN));
  KCurrentState::getInstance()->setEndStopState(2, 0, digitalRead(Z_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(2, 1, digitalRead(Z_MAX_PIN));
*/
}

void KCurrentState::printPosition()
{
/* From Frambot
  if (stepsPerMmX <= 0) { stepsPerMmX = 1; }
  if (stepsPerMmY <= 0) { stepsPerMmY = 1; }
  if (stepsPerMmZ <= 0) { stepsPerMmZ = 1; }

  Serial.print("R82");
  Serial.print(" X");
  Serial.print((float)x / (float)stepsPerMmX);
  Serial.print(" Y");
  Serial.print((float)y / (float)stepsPerMmY );
  Serial.print(" Z");
  Serial.print((float)z / (float)stepsPerMmZ * 1.0);
  printQAndNewLine();
*/
}
void KCurrentState::printAcceleration(){
  Serial.print("R82");
  Serial.print(" AX");
  Serial.print(FarmbotSensor.ax);
  Serial.print(" AY");
  Serial.print(FarmbotSensor.ay);
  Serial.print(" AZ");
  Serial.print(FarmbotSensor.az);
  printQAndNewLine();
}

void KCurrentState::printOrientation(){
  Serial.print("R82");
  Serial.print(" R");
  Serial.print(FarmbotSensor.roll_deg);
  Serial.print(" P");
  Serial.print(FarmbotSensor.pitch_deg);
  Serial.print(" H");
  Serial.print(FarmbotSensor.heading_deg);
  printQAndNewLine();
}

String KCurrentState::getPosition()
{
/* From Frambot
  if (stepsPerMmX <= 0) { stepsPerMmX = 1; }
  if (stepsPerMmY <= 0) { stepsPerMmY = 1; }
  if (stepsPerMmZ <= 0) { stepsPerMmZ = 1; }

  String output = "";

  output += "R82";
  output += " X";
  output += (float)x / (float)stepsPerMmX * 1.0;
  output += " Y";
  output += (float)y / (float)stepsPerMmY * 1.0;
  output += " Z";
  output += (float)z / (float)stepsPerMmZ * 1.0;
  //output += getQAndNewLine();

  return output;
*/
}

void KCurrentState::printBool(bool value)
{
  if (value)
  {
    Serial.print("1");
  }
  else
  {
    Serial.print("0");
  }
}

void KCurrentState::printEndStops()
{
/* From Frambot  
  Serial.print("R81");
  Serial.print(" XA");
  printBool(endStopState[0][0]);
  Serial.print(" XB");
  printBool(endStopState[0][1]);
  Serial.print(" YA");
  printBool(endStopState[1][0]);
  Serial.print(" YB");
  printBool(endStopState[1][1]);
  Serial.print(" ZA");
  printBool(endStopState[2][0]);
  Serial.print(" ZB");
  printBool(endStopState[2][1]);
  //Serial.print("\r\n");
  printQAndNewLine();
*/
}

void KCurrentState::print()
{
  printPosition();
  printEndStops();
}

void KCurrentState::setQ(long q)
{
  Q = q;
}

void KCurrentState::resetQ()
{
  Q = 0;
}

void KCurrentState::printQAndNewLine()
{
  Serial.print(" Q");
  Serial.print(Q);
  Serial.print("\r\n");
}

String KCurrentState::getQAndNewLine()
{
  String output = "";

  output += " Q";
  output += Q;
  output += "\r\n";

  return output;
}

void KCurrentState::setEmergencyStop()
{
  emergencyStop = true;
}

void KCurrentState::resetEmergencyStop()
{
  emergencyStop = false;
}

bool KCurrentState::isEmergencyStop()
{
  return emergencyStop;
}
