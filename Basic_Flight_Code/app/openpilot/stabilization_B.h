/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{ 
 * @addtogroup StabilizationModule Stabilization Module
 * @{ 
 *
 * @file       stabilization.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Attitude stabilization module.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @brief      Attitude stabilization.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef STABILIZATION_B_H
#define STABILIZATION_B_H
   
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>
   
enum {ROLL,PITCH,YAW,MAX_AXES};
enum {
  PID_RATE_ROLL,   // Rate controller settings
  PID_RATE_PITCH,
  PID_RATE_YAW,
  PID_ATT_ROLL,    // Attitude controller settings
  PID_ATT_PITCH,
  PID_ATT_YAW,
  PID_VBAR_ROLL,   // Virtual flybar settings
  PID_VBAR_PITCH,
  PID_VBAR_YAW,
  PID_COORDINATED_FLIGHT_YAW,
  PID_MAX
};

extern struct pid pids[PID_MAX];

extern void StabilizationInitialize();
extern void stabilize();
#endif /* STABILIZATION_B_H */

/**
  * @}
  * @}
  */
