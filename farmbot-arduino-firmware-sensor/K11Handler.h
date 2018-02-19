/*
 * K11Handler.h
 */

#ifndef K11HANDLER_H_
#define K11HANDLER_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K11Handler : public KCodeHandler
{
public:
  static K11Handler *getInstance();
  int execute(Command *);

private:
  K11Handler();
  K11Handler(K11Handler const &);
  void operator=(K11Handler const &);
  long adjustStepAmount(long);
  long getNumberOfSteps(double, double);
};

#endif /* K11HANDLER_H_ */
