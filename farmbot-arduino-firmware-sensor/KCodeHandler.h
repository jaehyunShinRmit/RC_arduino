/*
 * KCodeHandler.h
 *
 *  Created on: 15 maj 2014
 *      Author: MattLech
 */

#ifndef KCodeHandler_H_
#define KCodeHandler_H_
#include "Command.h"

class KCodeHandler
{
public:
  KCodeHandler();
  virtual ~KCodeHandler();
  virtual int execute(Command *);
};

#endif /* KCodeHandler_H_ */
