#include "easy_main.h"

#include "common_includes.h"
#include "../dRobot/general_msgs/geometry_msgs.h"
#include "../dRobot/general_odrs/numeric_odrs.h"

#include "../dRobot/Button.h"
#include "../dRobot/Keyboard.h"
#include "../dRobot/Kinematics.h"
#include "../dRobot/LED.h"
#include "../dRobot/Odometer.h"
#include "../dRobot/DeadReckoning.h"
#include "../dRobot/PathGuider.h"
#include "../dRobot/PIDController.h"
#include "../dRobot/PosePoint.h"
#include "../dRobot/MotorDriver.h"
#include "../dRobot/WheelObserver.h"

/* External global variables */

// Timer for Ticker interrupt
extern TIM_HandleTypeDef htim6;

// TImer for PWM generation
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

// Timer counters for encoder
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// UART connection for PC serial
extern UART_HandleTypeDef huart3;


/* All devices */

// Input device
dRobot::Keyboard keyboard(&huart3);
dRobot::Button button(GPIOF, GPIO_PIN_14);

// Point device
dRobot::PosePoint wheel_point1;
dRobot::PosePoint wheel_point2;
dRobot::PosePoint wheel_point3;
dRobot::PosePoint wheel_point4;

// Sensor device
dRobot::Odometer odometer_r(TIM2, 4000, 0.030);
dRobot::Odometer odometer_f(TIM3, 4000, 0.030);
dRobot::Odometer odometer_l(TIM4, 4000, 0.030);

// System device
dRobot::Kinematics kinematics;
dRobot::DeadReckoning dead_reckoning(0.74385/2.0, 10.0 - 0.74385/2.0, 0.0);
dRobot::PIDController pid_controller1(0.30, 0.001, 0.00001);
dRobot::PIDController pid_controller2(0.30, 0.001, 0.00001);
dRobot::PIDController pid_controller3(0.30, 0.001, 0.00001);
dRobot::PIDController pid_controller4(0.30, 0.001, 0.00001);
dRobot::WheelObserver wheel_observer;
dRobot::PathGuider path_guider(3.0, 2.0);

// Output device
dRobot::LED led(GPIOB,GPIO_PIN_14);
dRobot::MotorDriver motor_driver1(&(TIM1->CCR1), &(TIM1->CCR2), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver2(&(TIM8->CCR1), &(TIM8->CCR2), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver3(&(TIM8->CCR3), &(TIM8->CCR4), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver4(&(TIM1->CCR3), &(TIM1->CCR4), 0, 1000, 0.75);


/* Setup */

void easy_setup(){
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	ticker_init(&htim6, 16, 'm');

	// setup wheel_point
	wheel_point1.setPosition(-0.530/sqrt(2.0), 0.530/sqrt(2.0), 0.0);
	wheel_point1.setAngle_z(-3.0*M_PI/4.0);

	wheel_point2.setPosition(-0.530/sqrt(2.0), -0.530/sqrt(2.0), 0.0);
	wheel_point2.setAngle_z(-M_PI/4.0);

	wheel_point3.setPosition(0.530/sqrt(2.0), -0.530/sqrt(2.0), 0.0);
	wheel_point3.setAngle_z(M_PI/4.0);

	wheel_point4.setPosition(0.530/sqrt(2.0), 0.530/sqrt(2.0), 0.0);
	wheel_point4.setAngle_z(3.0*M_PI/4.0);


	// setup odometer_r
	odometer_r.setPosition(0.21965/sqrt(2.0), 0.21965/sqrt(2.0), 0.0);
	odometer_r.setAngle_z(3.0*M_PI/4.0/* - M_PI*/);
	odometer_r.setup();

	// setup odometer_l
	odometer_f.setPosition(-0.21965/sqrt(2.0), 0.21965/sqrt(2.0), 0.0);
	odometer_f.setAngle_z(-3.0*M_PI/4.0/* + M_PI*/);
	odometer_f.setup();

	// setup odometer_r
	odometer_l.setPosition(-0.21965/sqrt(2.0), -0.21965/sqrt(2.0), 0.0);
	odometer_l.setAngle_z(-M_PI/4.0/* + M_PI*/);
	odometer_l.setup();

	// setup kinematics
	odometer_r >> &kinematics.wheel[0];
	odometer_f >> &kinematics.wheel[1];
	odometer_l >> &kinematics.wheel[2];
	kinematics.setup(0.020, 1);

	// setup dead_reckoning
	kinematics >> &dead_reckoning.kinematics;
	dead_reckoning.setup(0.020, 2);

	dead_reckoning >> &wheel_observer.positioning;

	// setup pid_controller
	motor_driver1 >> &pid_controller1.output;
	pid_controller1.setup();

	motor_driver2 >> &pid_controller2.output;
	pid_controller2.setup();

	motor_driver3 >> &pid_controller3.output;
	pid_controller3.setup();

	motor_driver4 >> &pid_controller4.output;
	pid_controller4.setup();

	// setup wheel_observer
	kinematics >> &wheel_observer.obs;
	pid_controller1 >> &wheel_observer.err[0];
	pid_controller2 >> &wheel_observer.err[1];
	pid_controller3 >> &wheel_observer.err[2];
	pid_controller4 >> &wheel_observer.err[3];
	wheel_point1 >> &wheel_observer.wheel[0];
	wheel_point2 >> &wheel_observer.wheel[1];
	wheel_point3 >> &wheel_observer.wheel[2];
	wheel_point4 >> &wheel_observer.wheel[3];
	wheel_observer.setup(0.020, 2);

	// setup keyboard
	led >> &keyboard.toggle_dev[1];
	wheel_observer >> &keyboard.twist_dev;
	//motor_driver1 >> &keyboard.volume_dev[1];
	//motor_driver2 >> &keyboard.volume_dev[2];
	//motor_driver3 >> &keyboard.volume_dev[3];
	//motor_driver4 >> &keyboard.volume_dev[4];
	keyboard.setup("KeyI");

	// setup path_guider
	dead_reckoning >> &path_guider.positioner;
	wheel_observer >> &path_guider.controller;
	path_guider.setup(0.020, 3);

	// setup button
	led >> &button.out;
	button.setup("BtnW");

	ticker_schedule();
	ticker_start();
}


void easy_loop(){
	dRobot::pose2d_msg crr_pose;
	dRobot::pose2d_odr new_pose;
	dRobot::float_array_odr path_odr;
	float arr[16] = {};

	path_odr.size = sizeof(arr) / sizeof(float);
	path_odr.arr = arr;

	// wait for button
	HAL_Delay(1000);
	while(button.shareMsg().val != 1) HAL_Delay(10);

	/*
	// path 1: to catch 1st boll
	arr[0] = 0.1256;
	arr[1] = -1.8767;
	arr[2] = 1.8767;
	arr[3] = 0.3744;

	arr[4] = 7.2511;
	arr[5] = -13.8767;
	arr[6] = 0.0;
	arr[7] = 9.6256;

	arr[8] = 1.5708;
	arr[9] = -4.7124;
	arr[10] = 4.7124;
	arr[11] = 0.0;

	arr[12] = 0.0;
	arr[13] = -15.0;
	arr[14] = 15.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	while(button.shareMsg().val != 1) HAL_Delay(10);

	// pose correction
	crr_pose = dead_reckoning.shareMsg();
	new_pose.x = crr_pose.x + 0.25;
	new_pose.y = crr_pose.y + 0.350;
	new_pose.theta = crr_pose.theta;
	dead_reckoning.shareOdr(new_pose);

	// path 2: to try spot1
	arr[0] = 3.83;
	arr[1] = -7.86;
	arr[2] = 9.18;
	arr[3] = 0.5;

	arr[4] = 2.58;
	arr[5] = -6.75;
	arr[6] = 4.26;
	arr[7] = 3.0;

	arr[8] = 1.5708;
	arr[9] = -4.7124;
	arr[10] = 4.7124;
	arr[11] = 1.5708;

	arr[12] = 0.0;
	arr[13] = -15.0;
	arr[14] = 15.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	*/
	while(button.shareMsg().val != 1) HAL_Delay(10);

	new_pose.x = 5.65;
	new_pose.y = 3.09;
	new_pose.theta = 3.1416;
	dead_reckoning.shareOdr(new_pose);

	// path 3: to catch 2nd ball
	arr[0] = -3.83;
	arr[1] = 3.63;
	arr[2] = -4.95;
	arr[3] = 5.65;

	arr[4] = -2.58;
	arr[5] = 0.99;
	arr[6] = 1.50;
	arr[7] = 3.09;

	arr[8] = -1.5708;
	arr[9] = 4.7124;
	arr[10] = -4.7124;
	arr[11] = 3.1416;

	arr[12] = 0.0;
	arr[13] = -15.0;
	arr[14] = 15.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	while(button.shareMsg().val != 1) HAL_Delay(10);
	while(1);

	// path 4: to try spot2
	arr[0] = 0.6050;
	arr[1] = -1.635;
	arr[2] = 6.18;
	arr[3] = 0.50;

	arr[4] = 3.43;
	arr[5] = -11.76;
	arr[6] = 11.76;
	arr[7] = 0.50;

	arr[8] = 0.0;
	arr[9] = 0.0;
	arr[10] = -1.5708;
	arr[11] = 3.1416;

	arr[12] = 0.0;
	arr[13] = -5.0;
	arr[14] = 5.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	while(button.shareMsg().val != 1) HAL_Delay(10);

	// path 5: to catch 3rd ball
	arr[0] = -0.605;
	arr[1] = 0.18;
	arr[2] = -4.725;
	arr[3] = 5.65;

	arr[4] = -3.92;
	arr[5] = 0.0;
	arr[6] = 0.0;
	arr[7] = 4.42;

	arr[8] = 0.0;
	arr[9] = 0.0;
	arr[10] = -1.5708;
	arr[11] = 3.1416;

	arr[12] = 0.0;
	arr[13] = -15.0;
	arr[14] = 15.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	while(button.shareMsg().val != 1) HAL_Delay(10);

	// path 6: to try spot3
	arr[0] = -4.12;
	arr[1] = 3.09;
	arr[2] = 6.18;
	arr[3] = 0.5;

	arr[4] = 5.25;
	arr[5] = -11.76;
	arr[6] = 11.76;
	arr[7] = 0.50;

	arr[8] = 0.0;
	arr[9] = 0.0;
	arr[10] = 1.5708;
	arr[11] = 1.5708;

	arr[12] = 0.0;
	arr[13] = -15.0;
	arr[14] = 15.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	//while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);
	while(button.shareMsg().val != 1) HAL_Delay(10);

	// path 7: to kicking standby position
	/*
	arr[0] = -5.4037;
	arr[1] = 8.0673;
	arr[2] = 0.0767;
	arr[3] = 0.3744;

	arr[4] = -7.0471;
	arr[5] = 22.8767;
	arr[6] = -21.3767;
	arr[7] = 9.6256;

	arr[8] = 1.0505;
	arr[9] = -3.1515;
	arr[10] = 3.1515;
	arr[11] = 0.0;

	arr[12] = 0.0;
	arr[13] = -17.0;
	arr[14] = 17.0;
	arr[15] = 1.0;

	path_guider.shareOdr(path_odr);

	while(path_guider.shareMsg().val < 1.0) HAL_Delay(10);

	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
	*/
	while(1);
}
#endif
