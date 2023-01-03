#include "motor_control.h"
#include "tim.h"
#include "gpio.h"



const double PWM_MAX_PERIOD_COUNT=60000;


//     PID Speed
const double Kp=0.5,Ki=0.2,Kd=0.01;
//const double Kp=1,Ki=0,Kd=0;
double err_left=0,I=0,D=0,last_err_left=0,last_last_err_left=0,PID_actual_left=0;
double err_right=0,last_err_right=0,last_last_err_right=0,PID_actual_right=0;

/*    PID Location */
double Kp_location=100,Ki_location_left=1,Ki_location_right=1,Kd_location=150;
double err_location_left=0,err_location_right=0;
double last_err_location_left=0,last_err_location_right=0;
double I_location_left=0,I_location_right=0;
double D_location_left=0,D_location_right=0;


double  PID_cauculate_left(double target,double real_value)  //PID Speed left
{
	err_left=target-real_value;
	PID_actual_left+=Kp*(err_left-last_err_left)+Ki*err_left+Kd*(err_left-2*last_err_left+last_last_err_left);
	last_last_err_left=last_err_left;
	last_err_left=err_left;
	return PID_actual_left;
}

double  PID_cauculate_right(double target,double real_value)  //PID Speed right
{
	err_right=target-real_value;
	PID_actual_right+=Kp*(err_right-last_err_right)+Ki*err_right+Kd*(err_right-2*last_err_right+last_last_err_right);
	last_last_err_right=last_err_right;
	last_err_right=err_right;
	return PID_actual_right;
}

double PID_cauculate_location_left(double target,double real_value)   //PID Location left
{
	if(err_location_left<300&&err_location_left>-300)
	{
		Ki_location_left=1000;
	}
  err_location_left=target-real_value;
	I_location_left+=err_location_left;
	D_location_left=err_location_left-last_err_location_left;
	double PID_actual=Kp_location*err_location_left+Ki_location_left*I+Kd_location*D;
	last_err_location_left=err_location_left;
	return PID_actual;
}

double PID_cauculate_location_right(double target,double real_value)   //PID Location right
{
		if(err_location_right<300&&err_location_right>-300)
	{
		Ki_location_right=1000;
	}
  err_location_right=target-real_value;
	I_location_right+=err_location_right;
	D_location_right=err_location_right-last_err_location_right;
	double PID_actual=Kp_location*err_location_right+Ki_location_right*I+Kd_location*D;
	last_err_location_right=err_location_right;
	return PID_actual;
}

void Motor_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim1);	
	HAL_TIM_Base_Start_IT(&htim2);	
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);  
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4); 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	
}



void set_speed_left(double v,double target)
{
  double v_r_min=target*60/3.1415926/0.064;
	double real_v=PID_cauculate_left(v_r_min,v);
	if(target>0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
		if(real_v<0)  real_v=0;
		int pwm_left=PWM_MAX_PERIOD_COUNT-real_v*PWM_MAX_PERIOD_COUNT/350.0;
		if(pwm_left<0) pwm_left=1;
		pwm_left=(pwm_left >PWM_MAX_PERIOD_COUNT)? PWM_MAX_PERIOD_COUNT :pwm_left;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_left);
	//	printf("pwm_left:");
	//	printf("%d\r\n",pwm_left);
	}
//	else if(target==0)
//	{
//		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
//		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
//	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		if(real_v>0)  real_v=0;
		int pwm_left=-real_v*PWM_MAX_PERIOD_COUNT/350.0;
		pwm_left=(pwm_left > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :pwm_left;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_left);
	}
	}

	
void set_speed_right(double v,double target)
{
	double v_r_min=target*60/3.1415926/0.064;
	double real_v=PID_cauculate_right(v_r_min,v);
	if(target>0)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		if(real_v<0)  real_v=0;
		int pwm_right=real_v*PWM_MAX_PERIOD_COUNT/350.0;
		pwm_right=(pwm_right > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :pwm_right;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_right);
	//	printf("pwm_right:");
	//	printf("%d\r\n",pwm_right);
	}
//		else if(target==0)
//	{
//		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
//		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//	}
	else
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		if(real_v>0)  real_v=0;
		int pwm_right=PWM_MAX_PERIOD_COUNT+real_v*PWM_MAX_PERIOD_COUNT/350.0;
		if(pwm_right<0) pwm_right=0;
		pwm_right=(pwm_right > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :pwm_right;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_right);

	}

}

void set_location_left(double target,double location)
{
	double real_pwm=PID_cauculate_location_left(target,location);
		if(real_pwm>0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
		int pwm_left=PWM_MAX_PERIOD_COUNT-real_pwm;
		if(pwm_left<0) pwm_left=0;
		pwm_left=(pwm_left > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :pwm_left;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_left);
//		printf("pwm_left:");
//		printf("%d\r\n",pwm_left);

	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
    real_pwm=-real_pwm;
		int pwm_left=(real_pwm > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :real_pwm;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_left);
	}
}

void set_location_right(double target,double location)
{
	double real_pwm=PID_cauculate_location_right(target,location);
		if(real_pwm<0)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		int pwm_right=PWM_MAX_PERIOD_COUNT-real_pwm;
		if(pwm_right<0) pwm_right=1;
		pwm_right=(pwm_right > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :pwm_right;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_right);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		int pwm_right=(real_pwm > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT :real_pwm;
		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_right);
//		printf("pwm_right:");
//  	printf("%d\r\n",pwm_right);
	}
}


		
	 

	
	
		  



