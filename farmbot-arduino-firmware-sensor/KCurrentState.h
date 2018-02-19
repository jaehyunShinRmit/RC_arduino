/*
 * KCurrentState.h
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef KCURRENTSTATE_H_
#define KCURRENTSTATE_H_
#include "Arduino.h"
#include "pins.h"

class KCurrentState
{
public:
  static KCurrentState *getInstance();

  long getAX();
  long getAY();
  long getAZ();
  long getGX();
  long getGY();
  long getGZ();
  long getMX();
  long getMY();
  long getMZ();
  long getLat();
  long getLog();
  long getSpeed();
  long getRoll();
  long getPitch();
  long getHeading();

  
  void setAX(long);
  void setAY(long);
  void setAZ(long);
  void setGX(long);
  void setGY(long);
  void setGZ(long);
  void setMX(long);
  void setMY(long);
  void setMZ(long);
  void setLat(long);
  void setLog(long);
  void setSpeed(long);
  void setRoll(long);
  void setPitch(long);
  void setHeading(long);
  
  long *getPoint();
  int getLastError();
  void setLastError(int error);

  void setEndStopState(unsigned int, unsigned int, bool);
  void printPosition();
  void printAcceleration();
  void printOrientation();
  
  String getPosition();
  void storeEndStops();
  void printEndStops();
  void print();
  void printBool(bool);

  void setQ(long);
  void resetQ();
  void printQAndNewLine();
  String getQAndNewLine();

  void setEmergencyStop();
  void resetEmergencyStop();
  bool isEmergencyStop();

  void setStepsPerMm(long stepsX, long stepsY, long stepsZ);

private:
  KCurrentState();
  KCurrentState(KCurrentState const &);
  void operator=(KCurrentState const &);

  long stepsPerMmX;
  long stepsPerMmY;
  long stepsPerMmZ;

  bool emergencyStop = false;
};

#endif /* KCurrentState_H_ */
