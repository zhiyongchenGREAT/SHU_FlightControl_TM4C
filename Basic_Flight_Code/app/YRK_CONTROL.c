#include "common.h"
#include "YRK_Digital_Scope.h"
#include "YRK_HCTL2020.h"
#include "YRK_L3G4200_SOFT.h"
#include "YRK_MMA7361.h"
#include "YRK_MOTOR_SERVO.h"
#include "YRK_KEY_SCROLL.h"
#include "YRK_CONTROL.h"
#include <math.h>
#include "main.h"

#define CAR_CTL_RATE 1250

float M_PI=0;
float L3G4200_INTCONST=0;
uint8 CAR_STATE=0;

struct Sensors sen;  //ad缓存

struct Sub sub;

struct Anglestate ang;

float ap=1.8,ad=0.4,rp=160.5,rd=0.325,sp=0.91,sd=0;

struct carstate car;
void car_init()
{
  M_PI=acos(-1.0);
  get_sensor_value();
  get_sensor_value();
  sub.ax=32323;
  sub.ay=34500;
  sub.az=26500;
  sub.gx=0;
  sub.gy=0;
  sub.gz=0;
  sub.pit=7000;
  ang.pit=atan2((sen.az-sub.az),-(sen.ax-sub.ax))/M_PI*32768+sub.pit;
  L3G4200_INTCONST=(25/9.0)*(CAR_CTL_RATE/1000000.0)*1.345;
  car.speedgiven=0;
  car.maxspeed=5000;
  car.minspeed=3700;
}
void get_sensor_value()
{
  sen.gx=l3g4200_getdata(L3G4200_X);
  sen.gy=l3g4200_getdata(L3G4200_Y);
  sen.gz=l3g4200_getdata(L3G4200_Z);
  sen.ax=mma7361_getdata_x();
  sen.ay=mma7361_getdata_y();
  sen.az=mma7361_getdata_z();
}
void angle_pro() 
{
float adj2=0.002;//纠正常数
float adj3=5000;//纠正失效界限//规避奇异点
float adj4=1000;//角度纠正截止值
	ang.pit_a =atan2((sen.az-sub.az),-(sen.ax-sub.ax))/M_PI*32768+sub.pit;

	
	ang.pit_s =sen.gy-sub.gy;

  
        

	
	ang.pit_d = ang.pit_a-ang.pit;
	if(abs(sen.ax-sub.ax)+abs(sen.az-sub.az)<=adj3)
	ang.pit_d=0;
	
	if(ang.pit_d>adj4)
	ang.pit_d=adj4;
		if(ang.pit_d<-adj4)
	ang.pit_d=-adj4;
        
	ang.pit+=ang.pit_s*L3G4200_INTCONST+ang.pit_d*adj2;
       
	//if(abs(sen.ax)+abs(sen.az)<=adj3)
	//ang.pit_d=0;
        //else
       // ang.pit_d = ang.pit_a-ang.pit;
	
	//if(ang.pit_d>adj4)
	//ang.pit_d=adj4;
	//	if(ang.pit_d<-adj4)
	//ang.pit_d=-adj4;

	 
	//*0.0012;//+car.acc*0.08;//最后一个系数为1：20.84ms/2 其他类推	
	
	
} 

void getspeed() 
{
car.ecl=hctl2020_getdata_l();
car.ecr=hctl2020_getdata_r();
car.ecl2+=car.ecl;
car.ecr2+=car.ecr;
 if(car.speedgiven>20000)
 car.speedgiven=20000;
 if(car.speedgiven<-20000)
 car.speedgiven=-20000;
 //car.speedgiven=0; 
 car.speedold=car.speed;//前一次车速
 car.acc=car.ecl+car.ecr-car.speed0;//车速改变量
 car.speed0=car.ecl+car.ecr; //车速
 car.speed=car.speed0-car.speedgiven;//车速偏差量
 car.dece+=car.speed0/16124.0;
 car.speedcount=1;  

}
void speedcontrol() 
{ 
  //int fValue;
  
  //car.speed=car.speed0;
  //car.speed0=0;

 
  //if(car.speedgiven!=0&&car.speedgiven-car.ecl-car.ecr<2)
 
//	fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
	car.speedval =(( car.speed-car.speedold)* car.speedcount *0.05 +car.speedold);
	  car.speedcount++;
  //car.speed=0;
 // else
}


void rudd_control() 
{


float ruddvalsub=0,ruddcutval=32000,adj=1.5,ruddeadzone=0;
  //GET_Value(1);
 // if(sen[5].value+sen[6].value>1000) 
  //{  
//car.ruddval=(line_l[30]-line_r[30])*adj;      //      *******
      ang.rol_s =sen.gx-sub.gx;
      ang.yaw_s =sen.gz-sub.gz;
      //car.ruddval=0;//(line_r-ccd.l)*6;
  car.ruddval+=ruddvalsub;
  if(car.ruddval>=ruddcutval)
  car.ruddval=ruddcutval;
  if(car.ruddval<=-ruddcutval)
  car.ruddval=-ruddcutval;
  car.ruddspeed=-ang.yaw_s+ang.rol_s;
  
  
      //car.ruddacc=(sen[3].value-sub.acc_y);
 //   if(ruddval>0)
  //  rudd_d=1;
  //  else
  //  rudd_d=-1;
 // } 
 // else
 // {
//  ruddval=rudd_d*2200;
 // }                                                          


  //rudd=ruddval;
}


void mix()
{
  float balance,rudd;
 
  //float ap=2.0,ad=0.4,rp=0,rd=0,sp=1.6,sd=0;
  //float ap=0,ad=0,rp=0,rd=0,sp=1,sd=0;
  balance=ang.pit*ap+ang.pit_s*ad+car.speedval*sp;
  rudd=car.ruddval*rp+car.ruddspeed*rd;
  //rudd=ang.pit_s*ad
  OutData[0]=car.speedval;
  //OutData[1]=(int16)balance;
  motor_set(CAR_STATE*(balance-rudd),CAR_STATE*(balance+rudd));
  servo1_set(ang.pit*-0.14+ang.pit_s*-0.03);
}
