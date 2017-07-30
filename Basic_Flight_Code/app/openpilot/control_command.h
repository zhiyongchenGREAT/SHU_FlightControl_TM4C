#ifndef CONTROL_COMMAND_H
#define CONTROL_COMMAND_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>
//void AttitudeInitialize();
void command_handler();

extern bool fly;
extern bool exfly;
extern bool sweep_mode;
#endif // CONTROL_COMMAND_H
