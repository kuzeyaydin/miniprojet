#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <pi_regulator.h>
#include <process_image.h>

uint16_t speedCorrection = 0;
uint16_t speedDist = 0;
uint8_t changestate = 0;

enum STATE state = SEARCH;

//simple PI regulator implementation
int16_t pid_regulator(float distance, float goal, float kp, float ki, float kd) {

	float error = 0;
	float speed = 0;
	float errordiff = 0;

	static float sum_error_rot = 0;
	static float lastError = 0;

	if (changestate) {
		sum_error_rot = 0;
		lastError = 0;
		changestate = 0;
	}

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if (fabs(error) < ERROR_THRESHOLD * goal) {
		return 0;
	}

	sum_error_rot += error;
	errordiff = (error - lastError) / WAITTIMEPID;
	lastError = error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if (sum_error_rot > MAX_SUM_ERROR) {
		sum_error_rot = MAX_SUM_ERROR;
	} else if (sum_error_rot < -MAX_SUM_ERROR) {
		sum_error_rot = -MAX_SUM_ERROR;
	}

	speed = kp * error + ki * sum_error_rot + kd * errordiff;

	return (int16_t) speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	int16_t speed = 0;
	int16_t speed_correction = 0;

	while (1) {
		time = chVTGetSystemTime();

		switch (state) {
		case SEARCH:
			speed_correction = pid_regulator(get_line_position(),
					(IMAGE_BUFFER_SIZE / 2), KP_SEARCH, 0, 0);
			//if the line is nearly in front of the camera, don't rotate
			if (abs(speed_correction) < ROTATION_THRESHOLD) {
				speed_correction = 0;
			}
			right_motor_set_speed(-ROTATION_COEFF * speed_correction); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(ROTATION_COEFF * speed_correction); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
			if (speed_correction == 0) {
				state = TARGETACQUISITION;
				changestate = 1;
			}
			break;

		case TARGETACQUISITION:
			speed_correction = pid_regulator(get_line_position(),
					(IMAGE_BUFFER_SIZE / 2), KP_TARGET, KI_TARGET, KD_TARGET);
			//if the line is nearly in front of the camera, don't rotate
			if (abs(speed_correction) < ROTATION_THRESHOLD) {
				speed_correction = 0;
			}
			right_motor_set_speed(- ROTATION_COEFF * speed_correction); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(+ ROTATION_COEFF * speed_correction); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
			if (speed_correction == 0) {
				state = CHARGE;
				changestate = 1;
			}

			break;

		case CHARGE:
			speed = pid_regulator(VL53L0X_get_dist_mm() / 10, GOAL_DISTANCE,
			KP_CHARGE, KI_CHARGE, KD_CHARGE);

			right_motor_set_speed(speed); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(speed); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
			if (speed < ERROR_THRESHOLD) {
				state = TURNAROUND;
				changestate = 1;
			}
			break;

		case TURNAROUND:
			left_motor_set_speed(DANCE_SPEED); //IL FAUT ENTRER LA BONNE VALEUR
			right_motor_set_speed(-DANCE_SPEED); //IL FAUT ENTRER LA BONNE VALEUR
			chThdSleep(MS2ST(DANCE_TIME));

			state = GOBACK;
			changestate = 1;
			break;

		case GOBACK:
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
		/*
		 //computes the speed to give to the motors
		 //distance_cm is modified by the image processing thread
		 speed = pi_regulator_distance(get_distance_cm(), GOAL_DISTANCE);
		 speedDist = speed;

		 //computes a correction factor to let the robot rotate to be in front of the line
		 //speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2)); //on utilise pas de pi pour la rotation ??
		 speed_correction = pi_regulator_rotation(get_line_position(),(IMAGE_BUFFER_SIZE/2));
		 speedCorrection = speed_correction;

		 //if the line is nearly in front of the camera, don't rotate
		 if(abs(speed_correction) < ROTATION_THRESHOLD){
		 speed_correction = 0;
		 }
		 //applies the speed from the PI regulator and the correction for the rotation
		 right_motor_set_speed(speed - ROTATION_COEFF*speed_correction); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		 left_motor_set_speed(speed + ROTATION_COEFF*speed_correction); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		 */
		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(WAITTIMEPID));

	}
}

void pi_regulator_start(void) {
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO,
			PiRegulator, NULL);
}
