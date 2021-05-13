#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static uint16_t line_position = 0; //IMAGE_BUFFER_SIZE/2;	//middle //comme ça il tourne tant qu'il n'a pas trouver de ligne

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *bufferRed, uint8_t *bufferGreen, uint8_t *bufferBlue){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
/*
	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += bufferRed[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
*/
	do{

		wrong_line = 0;
		while(stop == 0 && i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			if(bufferRed[i]>RED_THRESHOLD && bufferGreen[i] < GREEN_THRESHOLD && bufferBlue[i] < BLUE_THRESHOLD)
			{
				begin = i;
				stop = 1;
			}
			i++;
		}
		if(begin && i <= (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			stop = 0;
			while(!stop && i<=IMAGE_BUFFER_SIZE)
			{
				if(bufferRed[i]<RED_THRESHOLD_LOW ||bufferGreen[i]>GREEN_THRESHOLD || bufferBlue[i]>BLUE_THRESHOLD)
				{
					end = i;
					stop = 1;
				}
				i++;
			}
			if(!end)
			{
				end = IMAGE_BUFFER_SIZE;
			}
		}
		else
		{
			line_not_found = 1;
		}

		if(!line_not_found && (end -begin)< MIN_LINE_WIDTH)
		{
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
/*		wrong_line = 0;
		//search for a begin------------------------------------------------------------------
		//je regarde si le rouge est plus grand que RED_THRESHOLD que j'ai mis a 127 (moitier de 255)
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) //peut-être changer pour regarder moins loin dans le buffer
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(bufferRed[i] > RED_THRESHOLD)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end--------------------------------------------- Reflechir a faire une sorte de trigger de schmitt
		//je regarde quand ça s'arrête (ça peut s'arreter au dernier pixel)
		if (i < (IMAGE_BUFFER_SIZE-WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i <= IMAGE_BUFFER_SIZE)
		    {
		        if(bufferRed[i] > mean && bufferRed[i+WIDTH_SLOPE]<mean && i<IMAGE_BUFFER_SIZE-WIDTH_SLOPE)
		        {
		            end = i;
		            stop = 1;
		        }
		        else if(bufferRed[i]<mean && i > IMAGE_BUFFER_SIZE-WIDTH_SLOPE)
		        {
		        	end = i;
		        	stop = i;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found----------------------------------------------------------
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
*/
	}while(wrong_line);

	//search if blue and green pixels are activated--------------------------------------------
/*	for(uint16_t i = begin; i < end; i++)
	{
		if(bufferGreen[i]>GREEN_THRESHOLD || bufferBlue[i]>BLUE_THRESHOLD)
		{
			line_not_found = 1;
		}
	}
*/
	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
		line_position = 0;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	/*//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}*/

	return width;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(0);
    while(1){
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
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t imageRed[IMAGE_BUFFER_SIZE] = {0};
	uint8_t imageBlue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t imageGreen[IMAGE_BUFFER_SIZE] = {0};
	uint16_t width = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			imageRed[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//extract only the blue pixels
		for(uint16_t i = 0; i<(2*IMAGE_BUFFER_SIZE); i+=2){
			imageBlue[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
		}

		//extract only the Green pixels
		for(uint16_t i = 0; i<(2*IMAGE_BUFFER_SIZE); i+=2){
			imageGreen[i/2] = (uint8_t)img_buff_ptr[i]&0x07 << 5;
			imageGreen[i/2]+= ((uint8_t)img_buff_ptr[i+1]&0xE0>>3);
		}

		//search for a line in the image and gets its width in pixels
		width = extract_line_width(imageRed,imageGreen,imageBlue);

		//converts the width into a distance between the robot and the camera
		if(width){
			//distance_cm = PXTOCM/width; //il faut changer ça car on utilise pas cette distance !!!
			distance_cm = GOAL_DISTANCE; //j'ai mis une valeur pour les tests
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(imageRed, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
