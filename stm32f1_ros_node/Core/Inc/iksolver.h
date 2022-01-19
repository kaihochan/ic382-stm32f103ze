/***
@Author: Vincent Chan
@About: Inverse Kinematic Solver
***/
#include "ros_main.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <utility>
/*** User defined variables ***/

// Resultant duty cycles of each motor
std::pair<int,int> twin_motors_duty_cycle;
std::pair<int,int> twin_motors_rotations;


// Wheel physical parameters in meters
double wheel_seperation = 0.47;
double max_velocity = 15;

// Wheel speed in m/s and rad/s
double motor1_v = 0, motor2_v = 0;
int motor1_rev = 0, motor2_rev = 0;

/*** Function Prototypes ***/
void twin_drive_ik(double linear_x,double angular_Z);
int clipping(int motor_pwm,int min_value,int max_value);

/*** Function Definition ***/
// 1. Transform velocity to duty cycles
void twin_drive_ik(double linear_x,double angular_z)
{
	// Kinematics
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
	
	// PWM
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
	
	// Output 
	int motor_1_pwm = 0, motor_2_pwm = 0;
	twin_motors_duty_cycle.first = motor_1_pwm;
	twin_motors_duty_cycle.second = motor_2_pwm;
	
	twin_motors_rotations.first = motor1_rev;
	twin_motors_rotations.second = motor2_rev;
}

// 2. Ensure the PWM is within reasonable range
int clipping(int motor_pwm,int min_value,int max_value)
{
	int output_pwm = 0;
	motor_pwm > max_value ? output_pwm = max_value : output_pwm=motor_pwm;
	motor_pwm < min_value ? output_pwm = min_value : output_pwm=motor_pwm;
	return output_pwm;
}