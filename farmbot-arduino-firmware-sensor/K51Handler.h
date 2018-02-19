/*
 * K51Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 */

#ifndef K51HANDLER_H_
#define K51HANDLER_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K51Handler : public KCodeHandler
{
public:
  static K51Handler *getInstance();
  int execute(Command *);

private:
  K51Handler();
  K51Handler(K51Handler const &);
  void operator=(K51Handler const &);
  long adjustStepAmount(long);
  long getNumberOfSteps(double, double);
};

#endif /* K51HANDLER_H_ */
