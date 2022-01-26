#include <ros.h>

// Variable used for PID control
double_t speed_err_L = 0, speed_err_R = 0;
double_t p_error_L = 0, p_error_R = 0;
double_t error_i_L = 0, error_i_R = 0;

// Define constant, PID controller gain value
#define k_p 0.5
#define k_i 0.2
#define k_d 0.3
