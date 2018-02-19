/*
 * KCodeProcessor.cpp
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Author: Tim Evers
 *      Modified by: Jaehyun Shin 20/02/2018
 */

#include "KCodeProcessor.h"
#include "KCurrentState.h"

KCodeProcessor::KCodeProcessor()
{
}

KCodeProcessor::~KCodeProcessor()
{
}

void KCodeProcessor::printCommandLog(Command *command)
{
  Serial.print("command == NULL: ");
  Serial.println("\r\n");
}

int KCodeProcessor::execute(Command *command)
{

  int execution = 0;
  bool isMovement = false;

  bool emergencyStop = false;
  //emergencyStop = CurrentState::getInstance()->isEmergencyStop();

  int attempt = 0;
  int maximumAttempts = ParameterList::getInstance()->getValue(PARAM_MOV_NR_RETRY);

  long Q = command->getQ();
  //CurrentState::getInstance()->setQ(Q);

  if
  (
    command->getCodeEnum() == K11 ||
    command->getCodeEnum() == K12 ||
    command->getCodeEnum() == K13 ||
    command->getCodeEnum() == K14 ||
    command->getCodeEnum() == K15 
  )
  {
    isMovement = true;
  }
  //Only allow reset of emergency stop when emergency stop is engaged
  if (emergencyStop)
  {
    if (!(
      command->getCodeEnum() == F09 ||
      command->getCodeEnum() == F20 ||
      command->getCodeEnum() == F21 ||
      command->getCodeEnum() == F22 ||
      command->getCodeEnum() == F81 ||
      command->getCodeEnum() == F82 ||
      command->getCodeEnum() == F83 ))
    {

    Serial.print(COMM_REPORT_EMERGENCY_STOP);
    KCurrentState::getInstance()->printQAndNewLine();
    return -1;
    }
  }

  // Tim 2017-04-15 Disable until the raspberry code is ready
  /*
  // Do not execute the command when the config complete parameter is not
  // set by the raspberry pi and it's asked to do a move command

  if (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) != 1) 
  {
    if (  command->getCodeEnum() == G00 ||
      command->getCodeEnum() == G01 ||
      command->getCodeEnum() == F11 ||
      command->getCodeEnum() == F12 ||
      command->getCodeEnum() == F13 ||
      command->getCodeEnum() == F14 ||
      command->getCodeEnum() == F15 ||
      command->getCodeEnum() == F16 ) 
    {

            Serial.print(COMM_REPORT_NO_CONFIG);
      CurrentState::getInstance()->printQAndNewLine();
      return -1;
    }
  }
  */

  // Return error when no command or invalid command is found
  if (command == NULL)
  {
    Serial.print(COMM_REPORT_BAD_CMD);
    KCurrentState::getInstance()->printQAndNewLine();

    if (LOGGING)
    {
      printCommandLog(command);
    }
    return -1;
  }

  if (command->getCodeEnum() == CODE_UNDEFINED)
  {
    Serial.print(COMM_REPORT_BAD_CMD);
    KCurrentState::getInstance()->printQAndNewLine();

    if (LOGGING)
    {
      printCommandLog(command);
    }
    return -1;
  }

  // Get the right handler for this command
  KCodeHandler *handler = getKCodeHandler(command->getCodeEnum());

  if (handler == NULL)
  {
    Serial.print(COMM_REPORT_BAD_CMD);
    KCurrentState::getInstance()->printQAndNewLine();

    Serial.println("R99 handler == NULL\r\n");
    return -1;
  }

  // Report start of command
  Serial.print(COMM_REPORT_CMD_START);
  KCurrentState::getInstance()->printQAndNewLine();

  // Execute command with retry
  KCurrentState::getInstance()->setLastError(0);
  while (attempt < 1 || (!emergencyStop && attempt < maximumAttempts && execution != 0 && execution != 2))
  // error 2 is timeout error: stop movement retries
  {

    if (LOGGING || debugMessages)
    {
      Serial.print("R99 attempt ");
      Serial.print(attempt);
      Serial.print(" from ");
      Serial.print(maximumAttempts);
      Serial.print("\r\n");
    }

    attempt++;
    if (attempt > 1)
    {
      Serial.print(COMM_REPORT_CMD_RETRY);
      KCurrentState::getInstance()->printQAndNewLine();
    }
    
    handler->execute(command);
    execution = KCurrentState::getInstance()->getLastError();
    emergencyStop = KCurrentState::getInstance()->isEmergencyStop();

    if (LOGGING || debugMessages)
    {
      Serial.print("R99 execution ");
      Serial.print(execution);
      Serial.print("\r\n");
    }
  }

  // Clean serial buffer
  while (Serial.available() > 0)
  {
    Serial.read();
  }

  // if movemement failed after retry
  // and parameter for emergency stop is set
  // set the emergency stop

  if (execution != 0)
  {
    if (isMovement)
    {
      if (ParameterList::getInstance()->getValue(PARAM_E_STOP_ON_MOV_ERR) == 1)
      {
        KCurrentState::getInstance()->setEmergencyStop();
      }
    }
  }

  // Report back result of execution
  if (execution == 0)
  {
    Serial.print(COMM_REPORT_CMD_DONE);
    KCurrentState::getInstance()->printQAndNewLine();
  }
  else
  {
    Serial.print(COMM_REPORT_CMD_ERROR);
    KCurrentState::getInstance()->printQAndNewLine();
  }

  KCurrentState::getInstance()->resetQ();
  return execution;
};

KCodeHandler *KCodeProcessor::getKCodeHandler(CommandCodeEnum codeEnum)
{
  KCodeHandler *handler = NULL;
  // These are if statements so they can be disabled as test
  // Usefull when running into memory issues again
  if (codeEnum == K11)
  {
    handler = K11Handler::getInstance();
  }
  if (codeEnum == K51)
  {
    handler = K51Handler::getInstance();
  }
   return handler;
}
