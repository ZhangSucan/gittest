#include "motion.h"
#include "motor_control.h"
#include "tim.h"
#include "stdio.h"
#include "math.h"

extern double v_left;
extern double v_right;

struct My_speed
{
	double v_start;
	double k_angle;
	double k_displacement;
};

struct My_speed v_60 = {0.6,0.0032,0.00046};
struct My_speed v_75 = {0.75,0.0032,0.00046};

struct My_speed v = {0.6,0.0032,0.00046};

double v_start=0.5;

double Kp_angle=1,Ki_angle=0,Kd_angle=0.02;
double err_angle=0,last_err_angle=0;
double I_angle=0,D_angle=0;

double d_v_angle=0,d_v_displacement=0;
//double k_angle=0.0032;                //0.00213
//double k_displacement=0.00046;         //0.0003

double k_angle=0.002;               
double k_displacement=0.0003;  

double Kp_displacement=0.79,Ki_displacement=0,Kd_displacement=0.25;
double err_displacement=0,last_err_displacement=0;
double I_displacement=0,D_displacement=0;


extern double err_left,I,D,last_err_left,last_last_err_left,PID_actual_left;
extern double err_right,last_err_right,last_last_err_right,PID_actual_right;

void car_start(void)
{
	err_left=0;
	last_err_left=0;
  last_last_err_left=0;
	err_right=0;
	last_err_right=0;
  last_last_err_right=0;
	v_left=0.1;
	v_right=0.1;
	HAL_Delay(20);
	v_left=0.2;
	v_right=0.2;
	HAL_Delay(20);
	v_left=0.3;
	v_right=0.3;
	HAL_Delay(20);
	v_left=0.4;
	v_right=0.4;
	HAL_Delay(20);
}

void car_stop(void)
{
	v_left=0.3;
	v_right=0.3;
//	v_left=-1;
// 	v_right=-1;
	HAL_Delay(20);
	v_left=0;
	v_right=0;
	HAL_Delay(20);
//	v_left=0;
//	v_right=0;
//	HAL_Delay(20);
}
void move_adjust(double angle,double displacement)
{	
// if(angle<10&&angle>-10)
// {
//	 angle=0;
//	//displacement=displacement+20;
// }
//	 if(displacement<100&&displacement>-100)
// {
//	 angle=0;
//	 displacement=displacement+20;
// }
	if(angle<70&&angle>-70)
	{
		err_angle=angle;
		I_angle+=err_angle;
		D_angle=err_angle-last_err_angle;
		d_v_angle=Kp_angle*err_angle+Ki_angle*I_angle+Kd_angle*D_angle;
		last_err_angle=err_angle;
	}

	if(displacement<320&&displacement>-320)
	{
	displacement=displacement+20;
	err_displacement=displacement;
	I_displacement+=err_displacement;
	D_displacement=err_displacement-last_err_displacement;
  d_v_displacement=Kp_displacement*err_displacement+Ki_displacement*I_displacement+Kd_displacement*D_displacement;
  v_left=v_start+k_displacement*d_v_displacement+k_angle*d_v_angle;
	if(v_left>1.2) v_left=1.2;
	if(v_left<0) v_left=0;
	v_right=v_start-k_displacement*d_v_displacement-k_angle*d_v_angle;
	if(v_right>1.2) v_right=1.2;
	if(v_right<0) v_right=0;
	last_err_displacement=err_displacement;
	}
}
