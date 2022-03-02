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
#include "pid.h"

// Varible used for Noetic Pi input to STM32
double_t speed_linear = 0, speed_angular = 0, v_L = 0, v_R = 0, dutycy1 = 0, dutycy2 = 0;

// Varible used for STM32 output to Noetic Pi
long encoder_value_m1 = 0, encoder_value_m1_p = 0, encoder_value_m2 = 0, encoder_value_m2_p = 0;

// Define constant, robot specification related
#define r_diameter 0.46
#define speed_max 5
#define max_ctr_period 3600

// Extern peripherals
extern IWDG_HandleTypeDef hiwdg;

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
	
	// Find v_L and v_R
	v_L = (speed_linear + 0.5 * r_diameter * speed_angular);
	v_R = (speed_linear - 0.5 * r_diameter * speed_angular);
	
	// Find duty cycle
	// if speed + error adjust > max speed, duty cycle = max counter
	abs(v_L ) <= speed_max ? dutycy1 = abs(max_ctr_period * (v_L ) / speed_max) : dutycy1 = max_ctr_period;
	abs(v_R ) <= speed_max ? dutycy2 = abs(max_ctr_period * (v_R ) / speed_max) : dutycy2 = max_ctr_period;
	
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


// Linear stepping motor control (2 MAR 2022)
void linear_step(char en=0, char dir=1, double_t ctr=500)
{
	if (en == 0)
	{
		HAL_GPIO_WritePin(LIN_ENA_GPIO_Port,LIN_ENA_Pin,GPIO_PIN_RESET);
		return;
	}
		else if (en == 1)
			HAL_GPIO_WritePin(LIN_ENA_GPIO_Port,LIN_ENA_Pin,GPIO_PIN_SET);
	
	if (dir == 1)
		HAL_GPIO_WritePin(LIN_DIR_GPIO_Port,LIN_DIR_Pin,GPIO_PIN_SET);
		else if (dir == 0)
			HAL_GPIO_WritePin(LIN_DIR_GPIO_Port,LIN_DIR_Pin,GPIO_PIN_RESET);
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ctr);
}


// RC servo motor control (2 MAR 2022)
void rc_servo(short ang, char ch=1)
{
	double_t ctr;
	if (ang == 0)
		ctr = 645;
		else if (ang == 90)
			ctr = 1520;
			else if (ang == 180)
				ctr = 2150;
	
	if (ch == 1)
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,ctr);
		else if (ch == 2)
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,ctr);
}


// ROS cmd_vel Subscriber
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel", &cmd_vel_msg);

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
	
	// ENABLE ENCODER FOR MOTOR 1 AND 2 (WHEEL)
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	// ENABLE PWM FOR LINEAR STEPPING MOTOR
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	
	// ENABLE PWM1 AND PWM2 FOR RC SERVO MOTOR
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);
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
