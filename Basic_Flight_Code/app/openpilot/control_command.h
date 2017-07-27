#ifndef CONTROL_COMMAND_H
#define CONTROL_COMMAND_H
#include "common.h"
#include "data_common.h"
#include "param_common.h"
#include "include.h"
//void AttitudeInitialize();
void command_handler();

extern bool fly;
extern bool exfly;
extern bool sweep_mode;
#endif // CONTROL_COMMAND_H
