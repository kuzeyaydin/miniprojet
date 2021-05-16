#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <selector.h>

static uint16_t line_position = STARTING_POS, red_line_position = STARTING_POS,
				green_line_position = STARTING_POS, blue_line_position = STARTING_POS,
				temp_line_position = STARTING_POS;

//internal functions
enum color getLineColor(uint16_t redWidth, uint16_t greenWidth, uint16_t blueWidth);
uint16_t extract_line_width(uint8_t *buffer, uint8_t tolerance);

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, CAPTURED_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1,
			SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//auto white-balance
	po8030_set_awb(AUTO_WHITE_BALANCE);

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
	uint8_t imageRed[IMAGE_BUFFER_SIZE] = { 0 }, imageBlue[IMAGE_BUFFER_SIZE] = { 0 },
			imageGreen[IMAGE_BUFFER_SIZE] = { 0 };
	uint16_t redWidth = 0, greenWidth = 0, blueWidth = 0;

	bool send_to_computer = true;

	while (1) {
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Bitshift to left for easier testing
		//extracting each color
		for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
			imageRed[i / 2] = (uint8_t) ((img_buff_ptr[i] & RGB565_RED_BITMASK));

			imageGreen[i / 2] = (uint8_t) ((img_buff_ptr[i] & RGB565_GREEN_BITMASK_1) << 5);
			imageGreen[i / 2] += (uint8_t) ((img_buff_ptr[i + 1] & RGB565_GREEN_BITMASK_2) >> 3);

			imageBlue[i / 2] = (uint8_t) ((img_buff_ptr[i + 1] & RGB565_BLUE_BITMASK) << 3);
		}

		//search for a line in the image and gets its width in pixels, for each color
		redWidth = extract_line_width(imageRed, RED_TOLERANCE);
		red_line_position = temp_line_position;
		greenWidth = extract_line_width(imageGreen, GREEN_TOLERANCE);
		green_line_position = temp_line_position;
		blueWidth = extract_line_width(imageBlue, BLUE_TOLERANCE);
		blue_line_position = temp_line_position;

		if (getLineColor(redWidth, greenWidth, blueWidth) == target_color()) {
			line_position = temp_line_position;
		} else {
			line_position = STARTING_POS;
		}

		if (send_to_computer) {

			//sends to the computer the image
			//SendUint8ToComputer(imageBlue, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
	}
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 *  Tolerance is for regulation depending on the ambient light
 */
uint16_t extract_line_width(uint8_t *buffer, uint8_t tolerance) {

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;
	mean -= tolerance;

	do {
		//wrong line is for the cases with noise
		wrong_line = 0;
		//looking for a beginning of a line
		while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
			if (buffer[i] > mean && buffer[i + WIDTH_SLOPE] < mean && buffer[i]) {
				begin = i;
				stop = 1;
			}
			i++;
		}
		//looking for an end
		if (begin && i <= (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
			stop = 0;
			while (!stop && i <= IMAGE_BUFFER_SIZE) {
				if (buffer[i] > mean && buffer[i - WIDTH_SLOPE] < mean) {
					end = i;
					stop = 1;
				}
				i++;
			}

			//in case an end is not found
			if (i > IMAGE_BUFFER_SIZE || !end) {
				end = IMAGE_BUFFER_SIZE;
			}

		} else {
			line_not_found = 1;
		}

		//if a line is found but smaller then the given minimum width, assumed to be noise, code looks for another line
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
		width = 0;
		temp_line_position = STARTING_POS; //keeps looking
	} else {
		width = (end - begin);
		temp_line_position = (begin + end) / 2; //gives the line position.
	}

	return width;
}

uint16_t get_line_position(void) {
	return line_position;
}

/*
 * 1) for main colors, lines 2 other main colors has to match, and either there's no line in that main color
 * or the line in that color is different from the 2 others
 * 2) for mix of 2 main colors, there's a line in the other color, and for the mixed colors there's either
 * no line for them or their lines are in a different place compared to the color that's not part of the mix
 * 3) for black, lines for all 3 colors has to be the same
 * 4) if nothing is found, white is returned and robot keeps searching
*/
enum color getLineColor(uint16_t redWidth, uint16_t greenWidth, uint16_t blueWidth) {

	if ((greenWidth && blueWidth) && abs(green_line_position - blue_line_position) < SAME_LINE_POS
		&& (abs(red_line_position - green_line_position) > SAME_LINE_POS || !redWidth)) {
		temp_line_position = green_line_position;
		return RED;
	}

	if ((redWidth && blueWidth) && abs(red_line_position - blue_line_position) < SAME_LINE_POS
		&& (abs(red_line_position - green_line_position) > SAME_LINE_POS || !greenWidth)) {
		temp_line_position = red_line_position;
		return GREEN;
	}

	if ((redWidth && greenWidth) && abs(green_line_position - red_line_position) < SAME_LINE_POS
		&& (abs(red_line_position - blue_line_position) > SAME_LINE_POS || !blueWidth)) {
		temp_line_position = red_line_position;
		return BLUE;
	}

	if (blueWidth && (blue_line_position < green_line_position - SAME_LINE_POS || !greenWidth)
		&& (blue_line_position < red_line_position - SAME_LINE_POS || !redWidth)) {
		temp_line_position = blue_line_position;
		return YELLOW;
	}

	if (greenWidth && (green_line_position < blue_line_position - SAME_LINE_POS || !blueWidth)
		&& (green_line_position < red_line_position - SAME_LINE_POS || !redWidth)) {
		temp_line_position = green_line_position;
		return MAGENTA;
	}

	if (redWidth && (red_line_position < blue_line_position - SAME_LINE_POS || !blueWidth)
		&& (red_line_position < green_line_position - SAME_LINE_POS || !greenWidth)) {
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
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint16_t target_color(void) {
	if(get_selector()<BLACK) return get_selector();
	else return BLACK;
}
