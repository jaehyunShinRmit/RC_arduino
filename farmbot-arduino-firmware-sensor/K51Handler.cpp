/*
 * K51Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K51Handler.h"

static K51Handler *instance;

K51Handler *K51Handler::getInstance()
{
  if (!instance)
  {
    instance = new K51Handler();
  };
  return instance;
};

K51Handler::K51Handler()
{
}

int K51Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report sesnsor data \r\n");
  }

  KCurrentState::getInstance()->printAcceleration();

  return 0;
}
