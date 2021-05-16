#include <hal.h>
#include <memory_protection.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <spi_comm.h>
#include <process_image.h>
#include <pid_regulator.h>

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts spi communication
    spi_comm_start();
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
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
