#ifndef _MOTOR_CONTROL_
#define _MOTOR_CONTROL_

void Motor_Init(void);
void set_speed_left(double v,double target);
void set_speed_right(double v,double target);
void set_location_left(double target,double location);
void set_location_right(double target,double location);

double PID_cauculate_left(double target,double real_value);
double PID_cauculate_right(double target,double real_value);
double PID_cauculate_location_left(double target,double real_value);
double PID_cauculate_location_right(double target,double real_value);

#endif
