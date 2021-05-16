#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define CAPTURED_LINE			100 //this line from camera will be used for image treatment
#define IMAGE_BUFFER_SIZE		640	//image length, image will be 1xIMAGE_BUFFER_SIZE
#define WIDTH_SLOPE				5	//accounting for noise with a...
#define MIN_LINE_WIDTH			40	//...color line on a white background
#define AUTO_WHITE_BALANCE		0 	//0 for off, 1 for on

enum color {
	RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, BLACK, WHITE
};

//adding tolerances to account for different ambient lights
//8*actual tolerance for r&b and 4*actual tolerance for green
//since all values are bitshifted to left for easier testing
#define RED_TOLERANCE			32
#define GREEN_TOLERANCE			8
#define BLUE_TOLERANCE			32
//due to camera not being very good, different colors might have different positions for the same line,
//this adds a tolerance when checking if they are at the same line
#define LINE_POS_TOLERANCE		0.05
#define SAME_LINE_POS 			IMAGE_BUFFER_SIZE*LINE_POS_TOLERANCE

//camera looks at left(rotates left) while searching for 0,
//looks at right for IMAGE_BUFFER_SIZE
#define STARTING_POS			0

#define ARENA_RADIUS			35.0f //centimeter

#define ROTATION_THRESHOLD		10 //noise threshold for feedback control while rotating
#define ROTATION_COEFF			0.5 //coefficient compared to forward advancement feedback control
#define PXTOCM					1570.0f //converts pixels to centimeters
#define GOAL_DISTANCE 			10.0f //distance to stop when robot encounters an obstacle, centimeters
#define ERROR_THRESHOLD			0.01f	//error threshold for feedback control, percent

//PID regulation parameters for different stages
//Kps and Kis are found experimentally, Kds is determined by Ziegler-Nichols tuning methods having kd=ki/4
#define KP_SEARCH				1.2f
#define KP_TARGET				20*KP_SEARCH
#define KI_TARGET				1.0f		//No 0
#define KD_TARGET				KI_TARGET/4	//No 0
#define KP_CHARGE				500.0f
#define KI_CHARGE				1.0f		//No 0
#define KD_CHARGE				KI_CHARGE/4	//No 0
#define KP_GOBACK				KP_CHARGE
#define KI_GOBACK				KI_CHARGE
#define KD_GOBACK				KD_CHARGE
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI_TARGET) //set denominator to biggest Ki
#define MAX_DIFF_ERROR			(MOTOR_SPEED_LIMIT/KD_TARGET) //set denominator to biggest Kd
#define WAIT_TIME_PID			5 //time between each pid regulation, millisecond

#define DANCE_SPEED				MOTOR_SPEED_LIMIT/3
#define DANCE_TIME				1800 //experimental value, ms

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
