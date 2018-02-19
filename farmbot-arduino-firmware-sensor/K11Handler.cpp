/*
 * K11Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K11Handler.h"

static K11Handler *instance;

K11Handler *K11Handler::getInstance()
{
  if (!instance)
  {
    instance = new K11Handler();
  };
  return instance;
};

K11Handler::K11Handler()
{
}

int K11Handler::execute(Command *command)
{
  /*
  	Serial.print("G00 was here\r\n");

  	Serial.print("R99");
  	Serial.print(" X ");
  	Serial.print(command->getX());
  	Serial.print(" Y ");
  	Serial.print(command->getY());
  	Serial.print(" Z ");
  	Serial.print(command->getZ());
  	Serial.print(" A ");
  	Serial.print(command->getA());
  	Serial.print(" B ");
  	Serial.print(command->getB());
  	Serial.print(" C ");
  	Serial.print(command->getC());
  	Serial.print("\r\n");


  StepperControl::getInstance()->moveToCoords(
      command->getX(), command->getY(), command->getZ(),
      command->getA(), command->getB(), command->getC(),
      false, false, false);
 
  if (LOGGING)
  {
    CurrentState::getInstance()->print();
  }
   */
  return 0;
}
