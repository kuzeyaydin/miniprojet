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

enum STATE state = SEARCH;

//simple PI regulator implementation
int16_t pi_regulator_rotation(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error_rot = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD_ROT){
		return 0;
	}

	sum_error_rot += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error_rot > MAX_SUM_ERROR){
		sum_error_rot = MAX_SUM_ERROR;
	}else if(sum_error_rot < -MAX_SUM_ERROR){
		sum_error_rot = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error_rot;

    return (int16_t)speed;
}

int16_t pi_regulator_distance(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error_dist = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD_DIST){
		return 0;
	}

	sum_error_dist += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error_dist > MAX_SUM_ERROR){
		sum_error_dist = MAX_SUM_ERROR;
	}else if(sum_error_dist < -MAX_SUM_ERROR){
		sum_error_dist = -MAX_SUM_ERROR;
	}

	speed = KP_D * error + KI_D * sum_error_dist;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        switch (state){
			case SEARCH :
				speed_correction = pi_regulator_rotation(get_line_position(),(IMAGE_BUFFER_SIZE/2));
				//if the line is nearly in front of the camera, don't rotate
				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}
				right_motor_set_speed(0 - ROTATION_COEFF*speed_correction); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(0 + ROTATION_COEFF*speed_correction); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
				if(speed_correction == 0)
					state = CHARGE;
				break;

			case CHARGE :
				speed = pi_regulator_distance(VL53L0X_get_dist_mm()/10, GOAL_DISTANCE);
				right_motor_set_speed(speed); //right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed); //left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
				if(speed == 0)
					state = TURNAROUND;
				break;

			case TURNAROUND :
	/*			left_motor_set_pos(); //IL FAUT ENTRER LA BONNE VALEUR
				right_motor_set_pos(); //IL FAUT ENTRER LA BONNE VALEUR
				if(condition)
					state = GOBACK;
	*/			break;

			case GOBACK :
	/*			speed = pi_regulator_distance(get_distance_cm(), nouveauBUT);
				right_motor_set_speed(speed);
				left_motor_set_speed(speed);
				if(speed == 0)
					state = SEARCH;
	*/			break;

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
		chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
