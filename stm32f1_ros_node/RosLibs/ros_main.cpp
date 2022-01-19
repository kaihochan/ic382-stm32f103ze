#include "ros_main.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "motor_manage.h"
#include "iksolver.h"

// Varible used for Noetic Pi input to STM32
double_t speed_linear = 0, speed_angular = 0, v_L = 0, v_R = 0, dutycy1 = 0, dutycy2 = 0;

// Varible used for STM32 output to Noetic Pi
long encoder_value_m1 = 0, encoder_value_m1_p = 0, encoder_value_m2 = 0, encoder_value_m2_p = 0;

// Variable used for PID control
double_t speed_err_L = 0, speed_err_R = 0;
double_t p_error_L = 0, p_error_R = 0;
double_t error_i_L = 0, error_i_R;

// Define constant, PID controller gain value
#define k_p 0.5
#define k_i 0.2
#define k_d 0.3

// Define constant, robot specification related
#define r_diameter 0.46
#define speed_max 100
#define max_ctr_period 3600

// Extern peripherals
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim8;

// ROS Node Handler
ros::NodeHandle nh;

// ROS stm32_status Publisher
char stm32_cmd_vel_ready_msg[]= "STM32_cmd_vel_OK";
std_msgs::String str_msg;
ros::Publisher stm32_status_pub("/stm32/system_up", &str_msg);

// ROS motor encoders Publisher
geometry_msgs::Vector3 encoders_data;
ros::Publisher encoder_pub("/stm32/encoders",&encoders_data);


// Callback of cmd_vel
void cmd_vel_msg(const geometry_msgs::Twist& msg)
{
	// Get linear and angular speed (m/s and rad/s)
	speed_angular = msg.angular.z;
	speed_linear = msg.linear.x;
	// Transform cmd_vel to duty cycles
	twin_drive_ik(speed_linear,speed_angular);
	
	// Execute motor control
	double motor1_pwm = twin_motors_duty_cycle.first;
	double motor2_pwm = twin_motors_duty_cycle.second;
	twin_motors_rotations.first == 0? motor_controller(MOTOR1,CLOCKWISE,motor1_pwm,max_register_value):motor_controller(MOTOR1,COUNTERCLOCKWISE,motor1_pwm,max_register_value);
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
	
	// Find v_L and v_R
	v_L = (speed_linear + 0.5 * r_diameter * speed_angular);
	v_R = (speed_linear - 0.5 * r_diameter * speed_angular);
	
	// Find duty cycle
	// if speed + error adjust > max speed, duty cycle = max counter
	abs(v_L + speed_err_L) <= speed_max ? dutycy1 = abs(max_ctr_period * (v_L + speed_err_L) / speed_max) : dutycy1 = max_ctr_period;
	abs(v_R + speed_err_R) <= speed_max ? dutycy2 = abs(max_ctr_period * (v_R + speed_err_R) / speed_max) : dutycy2 = max_ctr_period;
	
	// GPIO enable motor
	if (speed_linear > 0)
	{
		HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_SET);	
	}
	else if (speed_linear < 0)
		{
			HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);
		}
		else if ((speed_linear == 0) && (speed_angular == 0))
			{
				HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);
			}
			else if ((speed_linear == 0) && (speed_angular < 0))
				{
					HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_SET);
				}
				else if ((speed_linear == 0) && (speed_angular > 0))
					{
						HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);
					}
	
	// Set motor PWM signal output
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,dutycy1);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,dutycy2);

	// LED Indicators
	HAL_GPIO_TogglePin(EXTENSION_LED4_GPIO_Port,EXTENSION_LED4_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED3_GPIO_Port,EXTENSION_LED3_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED2_GPIO_Port,EXTENSION_LED2_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED1_GPIO_Port,EXTENSION_LED1_Pin);
}

// ROS cmd_vel Subscriber
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel", &cmd_vel_msg);

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

void setup(void)
{
	
	// Turn Off ALL Core Board LEDs (PULLED-HIGH)
	HAL_GPIO_WritePin(CORE_LED0_GPIO_Port,CORE_LED0_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CORE_LED1_GPIO_Port,CORE_LED1_Pin,GPIO_PIN_SET);
	
	// Turn On ALL Extension Board LEs (PULLED-HIGH)
	HAL_GPIO_WritePin(EXTENSION_LED4_GPIO_Port,EXTENSION_LED4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED3_GPIO_Port,EXTENSION_LED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED2_GPIO_Port,EXTENSION_LED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED1_GPIO_Port,EXTENSION_LED1_Pin,GPIO_PIN_RESET);
	
	// Initialize ROS node
	nh.initNode();
	
	// Call ROS Master to keep a registry of this publisher
	nh.advertise(stm32_status_pub);
	nh.advertise(encoder_pub);
	
	// Subscribe cmd_vel
	nh.subscribe(velocity_sub);
	
	// Enable all motor related peripherals (ALL CORE LEDs should be ON)
	motor_peripherals_init(MOTOR1);
	
	// ENABLE PWM FOR MOTOR 1 AND 2 (WHEEL)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	
	// ENABLE PWM FOR MOTOR 3 AND 4 (TOP MODULE)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	
	// ENABLE ENCODER FOR MOTOR 1 AND 2 (WHEEL)
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	// ENABLE ENCODER FOR MOTOR 3 AND 4 (TOP MODULE)
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
}


void loop(void)
{
  // Life LED	
	HAL_GPIO_TogglePin(CORE_LED0_GPIO_Port,CORE_LED0_Pin);
	HAL_GPIO_TogglePin(CORE_LED1_GPIO_Port,CORE_LED1_Pin);	
	
	// Get encoders values and publish
	encoder_value_m1 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4));
	encoder_value_m2 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim5));
	
	// Push encoder value into p_value
	encoder_value_m1_p = encoder_value_m1;
	encoder_value_m2_p = encoder_value_m2;
	
	// Publish encoder vectors
	encoders_data.x = encoder_value_m1;
	encoders_data.y = encoder_value_m2;
	encoder_pub.publish(&encoders_data);
	
	// Error calculation with PID controller
	// PID currently disabled
	//speed_err_L = pidbase(v_L, (encoders_data.x - encoder_value_m1_p), 'L');
	//speed_err_R = pidbase(v_R, (encoders_data.y - encoder_value_m2_p), 'R');

	// STM32 Virtual COM Port (VCP)
	unsigned char buffer[]= {"STM32 USB COM Port OK!\r\n"};
	CDC_Transmit_FS(buffer,sizeof(buffer));

	nh.spinOnce();
	HAL_Delay(200);
	
	// Reset IWDG within 13.104s.
	HAL_IWDG_Refresh(&hiwdg);

}
