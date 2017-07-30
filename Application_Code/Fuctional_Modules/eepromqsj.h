#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

void eeprom_read(int select);
void eeprom_write(int select);
extern int16 eeprom_readdate[9];