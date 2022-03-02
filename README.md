# ic382-stm32f103ze
IC382, 2021-2022 Sem 2, Group N3

Link to ros-main.cpp: [click here](https://github.com/kaihochan/ic382-stm32f103ze/blob/main/stm32f1_ros_node/RosLibs/ros_main.cpp)

Pin allocation:
| Pin | Function | Name | Component |
| :---- | :----| :----| :----|
| PA0 | TIM5/PWM_CH1 | MOTOR1_PWM | MOTOR1 |
| PA1 | TIM5/PWM_CH2 | MOTOR2_PWM | MOTOR2 |
| PF7 | GPIO/OUTPUT | MOTOR1_IN1 | MOTOR1 |
| PF6 | GPIO/OUTPUT | MOTOR1_IN2 | MOTOR1 |
| PF9 | GPIO/OUTPUT | MOTOR2_IN1 | MOTOR2 |
| PF8 | GPIO/OUTPUT | MOTOR2_IN2 | MOTOR2 |
| PE9 | TIM1/ENC_CH1 | MOTOR1_ENCODER_A | MOTOR1 |
| PE11 | TIM1/ENC_CH2 | MOTOR1_ENCODER_B | MOTOR1 |
| PA6 | TIM3/ENC_CH1 | MOTOR2_ENCODER_A | MOTOR2 |
| PA7 | TIM3/ENC_CH2 | MOTOR2_ENCODER_B | MOTOR2 |
| PD12 | TIM4/PWM_CH1 | LIN_PUL | LINEAR STEPPING |
| PF4 | GPIO_OUTPUT | LIN_DIR | LINEAR STEPPING |
| PF5 | GPIO_OUTPUT | LIN_ENA | LINEAR STEPPING |
| PC6 | TIM8/PWM_CH1 | SER1_PWM | RC SERVO MOTOR 1 |
| PC7 | TIM8/PWM_CH2 | SER2_PWM | RC SERVO MOTOR 2 |

Pending work: 
  1. PID implementation
