#ifndef YRK_SONAR_H
#define YRK_SONAR_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern float sonar_distance[3],sonar_speed,sonar_acc;

void sonar_init();
void sonar_triger();
#endif // YRK_SONAR_H
