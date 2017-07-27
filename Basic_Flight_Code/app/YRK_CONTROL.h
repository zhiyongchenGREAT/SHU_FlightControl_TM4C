#ifndef __YRK_CONTROL_H__
#define __YRK_CONTROL_H__
//#define CAR_CTL_RATE 1250
struct Sensors     
{
    
  int32 gx; 
  int32 gy;
  int32 gz;
  uint16 ax;
  uint16 ay;
  uint16 az;
} ;
struct Sub 
{
 int32 gx;
 int32 gy;
 int32 gz;
 int32 ax;
 uint16 ay;
 uint16 az;
 uint16 pit;
// int acc_angle;
};
struct Anglestate     
{
    float pit;//积分角度输出
    float pit_a;
    int32 yaw_s;///角速度
    int32 rol_s;
    int32 pit_s; 
    float pit_d;
    
};
struct carstate     
{
    int16 acc;
    int16 ecr; 
    int16 ecl;
    int16 ecr2; 
    int16 ecl2;
    float angleval;
    int16 anglevalsubtrim;
    float aacangleval;
    int16 speed0;
    int16 speed;
    int16 speedold;
    float speedval;
    float speedvali ;
    uint8 speedcount;
    float anglespeed; 
    float dValue;  
    int16 ruddval;
      
    int16 ruddspeed;
    int16 ruddacc;  
    int16 speedgiven;
    int16 ruddspeedgiven;  
    float dece;   
    
    int16 maxspeed;
    int16 minspeed;
} ;
extern uint8 CAR_STATE;
extern struct Sensors sen;     
extern struct Sub sub;
extern struct Anglestate ang;
extern struct carstate car;
//函数声明
extern void car_init();                        //初始化CAR_CONTROL
extern void get_sensor_value();
extern void angle_pro();
extern void getspeed();
extern void speedcontrol(); 
extern void mix();
extern float ap,ad,rp,rd,sp,sd;
#endif  //__FIRE_CAR_CONTROL_H__