#include <pid_regulator.h>
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>
#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <process_image.h>

uint16_t speedCorrection = 0;
uint16_t speedDist = 0;
uint8_t changestate = 0;

//robots starts searching after bootup
enum STATE state = SEARCH;

//internal functions
void toggle_color_leds(void); //lights leds up according to targeted color
int16_t pid_regulator(float distance, float goal, float kp, float ki, float kd);

static THD_WORKING_AREA(waPidRegulator, 128);
static THD_FUNCTION(PidRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	int16_t speed = 0;
	int16_t speed_correction = 0;

	while (1) {
		time = chVTGetSystemTime();

		switch (state) {
		case SEARCH:
			toggle_color_leds();
			speed_correction = pid_regulator(get_line_position(), (IMAGE_BUFFER_SIZE / 2),
											 KP_SEARCH, 0, 0);

			//if the line is nearly in front of the camera, don't rotate
			if (abs(speed_correction) < ROTATION_THRESHOLD) {
				speed_correction = 0;
			}

			right_motor_set_speed(-ROTATION_COEFF * speed_correction);
			left_motor_set_speed(ROTATION_COEFF * speed_correction);

			if (speed_correction == 0) {
				set_rgb_led(LED2, 0, 0, 0);
				set_rgb_led(LED4, 0, 0, 0);
				set_rgb_led(LED6, 0, 0, 0);
				set_rgb_led(LED8, 0, 0, 0);
				state = TARGET;
				changestate = 1;
			}

			break;

		case TARGET:
			speed_correction = pid_regulator(get_line_position(), (IMAGE_BUFFER_SIZE / 2),
											 KP_TARGET, KI_TARGET, KD_TARGET);

			//if the line is nearly in front of the camera, don't rotate
			if (abs(speed_correction) < ROTATION_THRESHOLD) {
				speed_correction = 0;
			}

			right_motor_set_speed(-ROTATION_COEFF * speed_correction);
			left_motor_set_speed(ROTATION_COEFF * speed_correction);

			if (speed_correction == 0) {
				state = CHARGE;
				chThdSleepMilliseconds(WAIT_AFTER_TARGET);
				changestate = 1;
			}
			//it's possible to lose the line due to noise, robot goes back to searching in that case
			if(get_line_position()==STARTING_POS) {
				state = SEARCH;
				changestate = 1;
			}

			break;

		case CHARGE:
			//speed is regulated with distance from tof sensor
			speed = pid_regulator(VL53L0X_get_dist_mm() / 10, GOAL_DISTANCE, //divide by 10 for mm to cm
								  KP_CHARGE, KI_CHARGE, KD_CHARGE);

			right_motor_set_speed(speed);
			left_motor_set_speed(speed);

			if (speed < ERROR_THRESHOLD) {
				state = TURNAROUND;
				changestate = 1;
			}

			break;

		case TURNAROUND:
			set_body_led(1);
			left_motor_set_speed(DANCE_SPEED);
			right_motor_set_speed(-DANCE_SPEED);
			chThdSleepMilliseconds(DANCE_TIME);	//robot turns for the given amount of time

			state = GOBACK;
			set_body_led(0);
			changestate = 1;

			break;

		case GOBACK:
			//robot goes back to roughly the center
			speed = pid_regulator(VL53L0X_get_dist_mm() / 10, ARENA_RADIUS,
								  KP_GOBACK, KI_GOBACK, KD_GOBACK);

			right_motor_set_speed(speed); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(speed); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

			if (speed < ERROR_THRESHOLD) {
				state = SEARCH;
				changestate = 1;
			}

			break;

		default:
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(WAIT_TIME_PID));

	}
}

//simple PID regulator implementation
int16_t pid_regulator(float distance, float goal, float kp, float ki, float kd) {

	float error = 0;
	float speed = 0;
	float errordiff = 0;

	static float sum_error = 0;
	static float lastError = 0;

	if (changestate) {
		sum_error = 0;
		lastError = 0;
		changestate = 0;
	}

	error = distance - goal;

	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is noisy
	if (fabs(error) < ERROR_THRESHOLD * goal) {
		return 0;
	}

	sum_error += error;
	errordiff = (error - lastError) / WAIT_TIME_PID;
	lastError = error;

	//we set a maximum and a minimum for the sum and difference to avoid an uncontrolled growth

	if (sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if (sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

	if (errordiff > MAX_DIFF_ERROR)
		errordiff = MAX_DIFF_ERROR;
	else if (errordiff < -MAX_DIFF_ERROR)
		errordiff = -MAX_DIFF_ERROR;

	speed = kp * error + ki * sum_error + kd * errordiff;

	return (int16_t) speed;
}

void pid_regulator_start(void) {
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO+1, PidRegulator, NULL);
}

void toggle_color_leds(void) {
	switch(target_color()) {
	case RED:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, 0);
		break;
	case GREEN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY, 0);
		break;
	case BLUE:
		set_rgb_led(LED2, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, 0, 0, RGB_MAX_INTENSITY);
		break;
	case YELLOW:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		break;
	case MAGENTA:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		break;
	case CYAN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		break;
	default:
		set_rgb_led(LED2, 0, 0, 0);
		set_rgb_led(LED4, 0, 0, 0);
		set_rgb_led(LED6, 0, 0, 0);
		set_rgb_led(LED8, 0, 0, 0);
	}
}
