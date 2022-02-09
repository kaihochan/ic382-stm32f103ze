#include "pid.h"

// Variable used for PID control
double_t speed_err_L = 0, speed_err_R = 0;
double_t p_error_L = 0, p_error_R = 0;
double_t error_i_L = 0, error_i_R = 0;

// PID base calculate
// p_error is a pointer variable that can be return latest value
double_t pidbase(double_t target, double_t mers, char flag)
{
	// Varible used for PID control
	double_t error = 0, p_error = 0, error_i = 0, error_d = 0, speed = 0;
	
	// Proportional
	error = target - mers;
	
	// Integral
	flag == 'L' ? error_i = error_i_L : error_i = error_i_R;
	error_i = error_i + error;
	if ((error == 0) || (abs(error) >= 40))
		error_i = 0;
	flag == 'L' ? error_i_L = error_i : error_i_R = error_i;
	
	// Derivative
	flag == 'L' ? p_error = p_error_L : p_error = p_error_R;
	error_d = error - p_error;
	
	p_error = error;
	flag == 'L' ? p_error_L = p_error : p_error_R = p_error;
	
	speed = k_p * error + k_i * error_i + k_d * error_d;
	
	return speed;
}
