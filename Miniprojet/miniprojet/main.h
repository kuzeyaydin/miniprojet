#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40

#define RED_THRESHOLD			32 //le rouge est stocké a gauche on multiplie donc le threshold de base (qui est 2) par 8=16
#define GREEN_THRESHOLD			10
#define BLUE_THRESHOLD			2
#define SAME_LINE_POS 			IMAGE_BUFFER_SIZE*0.05

#define STARTING_POS			0 //pour ne pas bouger

#define ARENA_RADIUS			35.0f //centimeter

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			0.5  //2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f //centimeter
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD	0.01f	//percent
#define KP_SEARCH				1.6f
#define KP_TARGET				5*KP_SEARCH
#define KI_TARGET				2.0f
#define KD_TARGET				KI_TARGET/4 //Ziegler-Nichols tuning methods usually have kd=ki/4
#define KP_CHARGE				500.0f
#define KI_CHARGE				1.0f
#define KD_CHARGE				KI_CHARGE/4
#define KP_GOBACK				KP_CHARGE
#define KI_GOBACK				KI_CHARGE
#define KD_GOBACK				KD_CHARGE
#define DANCE_SPEED				MOTOR_SPEED_LIMIT/3
#define DANCE_TIME				1800 //experimental value, ms
#define WAITTIMEPID				10 // millisecond
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI_CHARGE)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
