#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#include "navigation.h"


static float line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t color = 0;
static uint16_t camera_line = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


void extract_line_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;


	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
			//to the mean of the image
			if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
			{
				begin = i;
				stop = 1;
			}
			i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
			stop = 0;
			while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			{
				if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
				{
					end = i;
					stop = 1;
				}
				i++;
			}
			//if an end was not found
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		}
		else//if no begin was found
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
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
	}else{
		//gives the middle position of the line
		line_position = (begin + end)/2;
		//chprintf((BaseSequentialStream *)&SDU1, "ligne: %f \n\n", line_position);
	}
	//attention, corriger (si temps), cas begin= 0 et end touver ou end =0 et begin trouver
}

static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //systime_t time;

	uint8_t *img_buff_ptr_line;
	uint8_t image_red_line[IMAGE_BUFFER_SIZE] = {0};

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
	uint8_t temp_green = 0;
	uint32_t sum_green = 0, sum_red = 0;
	bool send_to_computer = true;


    while(1){
    	//waits until an image has been captured
    	chBSemWait(&image_ready_sem);

    	//time = chVTGetSystemTime();
    	if(get_sensor_activate()==0){
			camera_line = LINE_FLOOR;

			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr_line = dcmi_get_last_image_ptr();

			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				//Extracts the red (5bits)
				//extracts the first 5bits of the first byte
				//nothing for the first byte
				image_red_line[i/2] = (uint8_t)img_buff_ptr_line[i]&MSK_RED;
			}
			extract_line_position(image_red_line);
    	}


		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image_red_line, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;

		//////////////////////////////////////////////////////////////////////////////////////////////////////

    	if(get_sensor_activate()==1){
    		camera_line = 10;
    		//bool send_to_computer = true;
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			sum_green = 0;
			sum_red = 0;

			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//Extracts the green (6bits)
			//extracts first 3bits of the first byte and 3bits of the second byte
				temp_green = (img_buff_ptr[i] & MSK_GREENa)<<5;
				temp_green = temp_green | ((img_buff_ptr[1+i] & MSK_GREENb)>>3);
				image_green[i/2] = temp_green;

			//Extracts the red (5bits)
				//extracts the first 5bits of the first byte
				//nothing for the first byte
				image_red[i/2] = (uint8_t)img_buff_ptr[i]&MSK_RED;

				//add the intensity of each color between the position 200 to 440 in the image buffer
				if(i>IMAGE_BUFFER_LOW*2 && i<IMAGE_BUFFER_HIGH*2)
				{
					sum_green = sum_green + image_green[i/2];
					sum_red = sum_red + image_red[i/2];
				}
			}
			//chprintf((BaseSequentialStream *)&SDU1, "sumred: %d \n\n", sum_red);
			//chprintf((BaseSequentialStream *)&SDU1, "sumgreen: %d \n\n", sum_green);
			//reads the color that has the greater sum of intensity
			if(sum_red > sum_green){
				color = COLOR_RED;
			}
			else{
				color = COLOR_GREEN;
			}
			//if(send_to_computer){
						//sends to the computer the image
				//		SendUint8ToComputer(image_green, IMAGE_BUFFER_SIZE);
					//}
					//invert the bool
					//send_to_computer = !send_to_computer;
    	}


    	//chprintf((BaseSequentialStream *)&SDU1,"temps0: %d \n", time);

	}

}



static THD_WORKING_AREA(waCaptureImage, 1024);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	po8030_advanced_config(FORMAT_RGB565, 0, camera_line, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    	dcmi_enable_double_buffering();
    	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    	dcmi_prepare();

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }

}


uint8_t get_color_trafic_light(void){
	return color;
}


float get_line_position(void){

	return line_position;
}

void process_image_line_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

