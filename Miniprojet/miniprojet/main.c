#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>
#include <spi_comm.h>
#include <process_image.h>
#include <pid_regulator.h>
#include <leds.h>

void toggle_color_leds(void);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();


    //starts the serial communication
    serial_start();
    //starts spi communication
    spi_comm_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//starts tof sensor
	VL53L0X_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pid_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
		toggle_color_leds();
    	//waits 10 msecond
        chThdSleepMilliseconds(10);
    }
}

void toggle_color_leds(void) {
	switch(target_color()) {
	case RED:
		set_rgb_led(LED2, RGB_MAX_INTENSITY/2, 0, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY/2/2, 0, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY/2, 0, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY/2, 0, 0);
		break;
	case GREEN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY/2, 0);
		break;
	case BLUE:
		set_rgb_led(LED2, 0, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED4, 0, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED6, 0, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED8, 0, 0, RGB_MAX_INTENSITY/2);
		break;
	case YELLOW:
		set_rgb_led(LED2, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2, 0);
		break;
	case MAGENTA:
		set_rgb_led(LED2, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED4, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED6, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED8, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/2);
		break;
	case CYAN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY/2, RGB_MAX_INTENSITY/2);
		break;
	default:
		set_rgb_led(LED2, 0, 0, 0);
		set_rgb_led(LED4, 0, 0, 0);
		set_rgb_led(LED6, 0, 0, 0);
		set_rgb_led(LED8, 0, 0, 0);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
