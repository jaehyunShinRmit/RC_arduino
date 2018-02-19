/*
 * KCodeProcessor.h
 *
 */

#ifndef KCODEPROCESSOR_H_
#define KCODEPROCESSOR_H_

#include "Command.h"
#include "Config.h"
#include "Debug.h"

#include "KCodeHandler.h"
#include "K11Handler.h"
#include "K51Handler.h"
#include "ParameterList.h"

class KCodeProcessor
{
public:
  KCodeProcessor();
  virtual ~KCodeProcessor();
  int execute(Command *command);

protected:
  KCodeHandler *getKCodeHandler(CommandCodeEnum);

private:
  void printCommandLog(Command *);
};

#endif /* KCODEPROCESSOR_H_ */
