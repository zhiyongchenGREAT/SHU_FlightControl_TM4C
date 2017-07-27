#include "common.h"
#include "driverlib/sysctl.h"
#if defined ( __ICCARM__ )      // IAR 
#pragma optimize=none 
#endif 
void SysCtlDelay_MS(vuint32 MS)
{
  while(MS>0)
  {
  SysCtlDelay(16000);
  MS--;
  }
}