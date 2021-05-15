#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static float distance_cm = 0;
static uint16_t line_position = STARTING_POS, red_line_position = STARTING_POS,
		green_line_position = STARTING_POS, blue_line_position = STARTING_POS,
		temp_line_position = STARTING_POS; //0; //IMAGE_BUFFER_SIZE / 2;//middle //0 comme a il tourne tant qu'il n'a pas trouver de ligne
_Bool isLineFound = FALSE;
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer, uint8_t tolerance) {

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM / GOAL_DISTANCE;

	//performs an average
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;
	mean -= tolerance;

	do {
		wrong_line = 0;
		while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
			if (buffer[i] > mean && buffer[i + WIDTH_SLOPE] < mean) {
				begin = i;
				stop = 1;
			}
			i++;
		}
		if (begin && i <= (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
			stop = 0;
			while (!stop && i <= IMAGE_BUFFER_SIZE) {
				if (buffer[i] > mean && buffer[i - WIDTH_SLOPE] < mean) {
					end = i;
					stop = 1;
				}
				i++;
			}

			if (i > IMAGE_BUFFER_SIZE || !end) {
				end = IMAGE_BUFFER_SIZE;
			}

		} else {
			line_not_found = 1;
		}

		if (!line_not_found && (end - begin) < MIN_LINE_WIDTH) {
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}

	} while (wrong_line);

	if (line_not_found) {
		begin = 0;
		end = 0;
		//width = last_width;
		width = 0;
		temp_line_position = STARTING_POS; //0;//IMAGE_BUFFER_SIZE / 2;
	} else {
		last_width = width = (end - begin);
		temp_line_position = (begin + end) / 2; //gives the line position.
	}

	return width;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	/*systime_t time;
	 time = chVTGetSystemTime();
	 //-> Functions to measure <-//
	 chprintf((BaseSequentialStream *)&SDU1, "captureime�=�%d\n", chVTGetSystemTime()-time);
	 */

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2,
			SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(0);
	while (1) {
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

	}
}

static THD_WORKING_AREA(waProcessImage, 4048);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	uint8_t *img_buff_ptr;
	uint8_t imageRed[IMAGE_BUFFER_SIZE] = { 0 }, imageBlue[IMAGE_BUFFER_SIZE] =
			{ 0 }, imageGreen[IMAGE_BUFFER_SIZE] = { 0 };
	uint16_t redWidth = 0, greenWidth = 0, blueWidth = 0;

	bool send_to_computer = true;

	while (1) {
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			imageRed[i / 2] = (uint8_t) ((img_buff_ptr[i] & 0xF8));
		}

		//extract only the blue pixels
		for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
			imageBlue[i / 2] = (uint8_t) ((img_buff_ptr[i + 1] & 0x1F)<<3);
		}

		//extract only the Green pixels
		for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
			imageGreen[i / 2] = (uint8_t) ((img_buff_ptr[i] & 0x07)<<5);	// *8 car bit shift ne fonctionne pas
			imageGreen[i / 2] += (uint8_t) ((img_buff_ptr[i + 1] & 0xE0)>>3);
		}

		//search for a line in the image and gets its width in pixels
		redWidth = extract_line_width(imageRed, RED_THRESHOLD);
		red_line_position = temp_line_position;
		greenWidth = extract_line_width(imageGreen, GREEN_THRESHOLD);
		green_line_position = temp_line_position;
		blueWidth = extract_line_width(imageBlue, BLUE_THRESHOLD);
		blue_line_position = temp_line_position;
		//converts the width into a distance between the robot and the camera
		if (getLineColor(redWidth, greenWidth, blueWidth) == TARGET_COLOR) {
			line_position = temp_line_position;
			distance_cm = 0;
			isLineFound = TRUE;

			//PXTOCM / redWidth; //il faut changer a car on utilise pas cette distance !!!
			//distance_cm = GOAL_DISTANCE; //j'ai mis une valeur pour les tests
		} else {
			line_position = STARTING_POS;
			distance_cm = GOAL_DISTANCE;
			isLineFound = FALSE;
		}

		if (send_to_computer) {

			//sends to the computer the image
			SendUint8ToComputer(imageBlue, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
	}
}

float get_distance_cm(void) {
	return distance_cm;
}

uint16_t get_line_position(void) {
	return line_position;
}
_Bool getLineFound(void)
{
	return isLineFound;
}

enum color getLineColor(uint16_t redWidth, uint16_t greenWidth,
		uint16_t blueWidth) {

	//if detects black in green&blue but not red, it's red line
	if ((greenWidth && blueWidth)
			&& abs(green_line_position - blue_line_position) < SAME_LINE_POS
			&& (abs(red_line_position - green_line_position) > SAME_LINE_POS
					|| !redWidth)) {
		temp_line_position = green_line_position;
		return RED;
	}

	//same as before, for green
	if ((redWidth && blueWidth)
			&& abs(red_line_position - blue_line_position) < SAME_LINE_POS
			&& (abs(red_line_position - green_line_position) > SAME_LINE_POS
					|| !greenWidth)) {
		temp_line_position = red_line_position;
		return GREEN;
	}

	//same as before, for blue
	if ((redWidth && greenWidth)
			&& abs(green_line_position - red_line_position) < SAME_LINE_POS
			&& (abs(red_line_position - blue_line_position) > SAME_LINE_POS
					|| !blueWidth)) {
		temp_line_position = red_line_position;
		return BLUE;
	}

	//
	if (blueWidth
			&& (blue_line_position < green_line_position - SAME_LINE_POS
					|| !greenWidth)
			&& (blue_line_position < red_line_position - SAME_LINE_POS
					|| !redWidth)) {
		temp_line_position = blue_line_position;
		return YELLOW;
	}

	if (greenWidth
			&& (green_line_position < blue_line_position - SAME_LINE_POS
					|| !blueWidth)
			&& (green_line_position < red_line_position - SAME_LINE_POS
					|| !redWidth)) {
		temp_line_position = green_line_position;
		return MAGENTA;
	}

	if (redWidth
			&& (red_line_position < blue_line_position - SAME_LINE_POS
					|| !blueWidth)
			&& (red_line_position < green_line_position - SAME_LINE_POS
					|| !greenWidth)) {
		temp_line_position = red_line_position;
		return CYAN;
	}

	if ((greenWidth && blueWidth && redWidth)
			&& abs(green_line_position - blue_line_position) < SAME_LINE_POS
			&& abs(red_line_position - green_line_position) < SAME_LINE_POS) {
		temp_line_position = red_line_position;
		return BLACK;
	}

	temp_line_position = STARTING_POS;

	return WHITE;

}

void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO,
			ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO,
			CaptureImage, NULL);
}
