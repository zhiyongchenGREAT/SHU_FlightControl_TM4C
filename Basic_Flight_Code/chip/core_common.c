#include <core_common.h>

#pragma optimize=none 

void SysCtlDelay_MS(vuint32 MS)
{
  while(MS>0)
  {
    SysCtlDelay(16000);
    MS--;
  }
}