# SHU_FlightControl_TM4C
Flight control firmware development based on TI TM4C MCU


!!!Important note:

1.The flight mode should be set to "00" to enable manual control, "01"-"03" is prohibited to use in normal case & need to be rewrote when performing spcific use. Be very carful since mode "01"-"03" may contain automatic flight task.

2.The parameters in control_command.c & FLOW_B.c must be reconfig costomizedly when performing on different drone.

rev1.0.4 2017/8/13:
NUEDC final version. Warning: this version code has problem of starting rotor which require further revision

1.add key detect to enable flight mode change

2.rewrite auto control code

3.add pic hold to hold drone horizontally when no optical flow sensor is applicable

4.split "flight init task" to 4 differrent functions to persue improved expensibility

5.add "one-key startup" in "flight routine task"

6.several trivial adjustment to meet the need of the competition


rev0.6 2017/8/3:

optical flow para ameliorate

1. Optical flow para ameliorate, compatible with old verision flow senosr

2. Code structure improvment: attitude solving, flow

rev0.5 2017/7/31:

Add back core_uartstdio.h to retrive the normal state of flow

rev0.4 2017/7/30:

1. Add several inc.h to make the code "include" more perspicuous.

2. Attach these inc.h to all the execuable files and bulid successfully.

rev0.3 2017/7/28:

1. Add test tim0 so that code execution time measurement can be feasible

2. The tir0/tmr1 init has also been optimized for JLINK DBG

rev0.2 2017/7/27:

preliminary improvement on code structure

1. Add comment template to IAR8

2. Optimize the Main entrance init code style

