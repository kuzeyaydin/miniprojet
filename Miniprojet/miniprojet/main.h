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

#define RED_THRESHOLD			32 //le rouge est stock� a gauche on multiplie donc le threshold de base (qui est 2) par 8=16
#define GREEN_THRESHOLD			10
#define BLUE_THRESHOLD			3
#define SAME_LINE_POS 			IMAGE_BUFFER_SIZE*0.05

#define STARTING_POS			0 //pour ne pas bouger

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			0.5  //2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD_ROT		5 		//[px] a cause du bruit de la camera
#define ERROR_THRESHOLD_DIST	0.5f	//[cm] because of the noise of the camera
#define KP						0.8f
#define KI 						0.1f	//must not be zero //3.5f Initially
#define KP_D					100.0f
#define KI_D					1.0f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
