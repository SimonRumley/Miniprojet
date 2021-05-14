#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <capteur.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <audio_processing.h>
#include "navigation.h"

static uint8_t sensor_activate = 0;

//simple P regulator implementation
int16_t p_regulator(float distance, float goal){

	//On utilise un P regulator pour corriger l'allignement / l'orientation du e-puck2 sur la ligne à suivre.

	float error = 0;
	float speed = 0;

	//error toujours positf
	if (distance > goal){
		error = (distance - goal);
	}
	else{
		error = (goal - distance);
	}

	if(fabs(error) < ERROR_THRESHOLD){
		return SPEED_LOW;
	}
	speed = SPEED_LOW - KP*error;

	return (int16_t)speed;
}

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = 0;
    uint16_t object_distance = 0;
    uint16_t max_norm_index = 0;
    float speed_correction = 0;
    uint8_t k=0;
    uint16_t max_norm_index_t1 = 0;
    uint16_t max_norm_index_t2 = 0;
    int16_t delta_max_norm_index = INIT_DELTA;
    uint8_t o=0; //envlever

    while(1){

    	time = chVTGetSystemTime();

    	//chprintf((BaseSequentialStream *)&SDU1,"temps0: %d \n",time);

    	//distance entre le e-puck et les feux grace au capteur TOF

    	object_distance = get_object_distance();

    	//frequence a laquelle se situe le pic maximal (norme max en fréquence du signal) detecté par le micro frontal
    	max_norm_index = get_max_norm_index();

    	//permet de verifier si deux sons à 3030Hz sont entendus entre deux thread (pour éviter les erreurs)
    	if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H && k==0){
			max_norm_index_t1 = max_norm_index;
		}
    	if(k==3)
		{
			max_norm_index_t2 = max_norm_index;
			delta_max_norm_index = max_norm_index_t2-max_norm_index_t1;
		}

    	if(k<3){k++;}else{k=0;}

    	//chprintf((BaseSequentialStream *)&SDU1,"max_i: %d \n", max_norm_index);



    	//si le TOF voit un objet (le feu) entre 15 et 8cm, alors la camera va regarder la couleur du feux
    	if (object_distance < DISTANCE_READ_TRAFIC_LIGHT && object_distance > DISTANCE_SECURITY &&
    			(max_norm_index < FREQ_LEFT_L || max_norm_index > FREQ_LEFT_H)){
			sensor_activate = 1;
			//stop si le feu est rouge
			if(get_color_trafic_light()==COLOR_RED)
			{
				speed=0;
				speed_correction=0;
			}
			//ralenti si le feu est vert
			else{
				speed=SPEED_GREEN;
				speed_correction=0;
			}
		}
    	//si le TOF voit un objet (le feu) a moins de 8cm, il s'arrete pour eviter la collision
		else if(object_distance < DISTANCE_SECURITY &&
				(max_norm_index < FREQ_LEFT_L || max_norm_index > FREQ_LEFT_H)){
			sensor_activate = 1;
			speed=0;
			speed_correction=0;
			//allumer led;
		}
    	//detection de l'alarme et le epuck2 fait un tour sur lui-meme
    	else if (max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H
			 && delta_max_norm_index <= DELTA_RANGE && delta_max_norm_index >= -DELTA_RANGE)
		{
    		systime_t time_turn;
			time_turn = chVTGetSystemTime();
			//chprintf((BaseSequentialStream *)&SDU1,"delta: %d \n", delta_max_norm_index);

			while(chVTGetSystemTime()-time_turn<MS2ST(2120)){
				left_motor_set_speed(-TURNING_SPEED);
				right_motor_set_speed(TURNING_SPEED);
			}
			//chprintf((BaseSequentialStream *)&SDU1,"delta3: %d \n", delta_max_norm_index);

    		o++; //enlever
    		//chprintf((BaseSequentialStream *)&SDU1,"%d \n",o);
    		delta_max_norm_index = INIT_DELTA;
    		k=0;
    }
    //permet au epuck2 de suivre une ligne au sol avec une correction par un P regulator
    else{
    		sensor_activate = 0;

    	    //line_position is modified by the image processing thread
    		speed =  p_regulator(get_line_position(),IMAGE_BUFFER_SIZE/2);
			//computes a correction factor to let the robot rotate to be in front of the line
			speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
    	}

    	//if the line is not exactly in the middle of the buffer, don't rotate
		//	if(abs(speed_correction) < ROTATION_THRESHOLD){
		//		speed_correction = 0;
		//	}

    	//applique une P regulation a speed et une correction pour la rotation
    	left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
    	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);

		//100Hz
    	//systime_t time1 = chVTGetSystemTime();

    	//chprintf((BaseSequentialStream *)&SDU1,"temps1: %d \n",time1);
		chThdSleepUntilWindowed(time, time + MS2ST(100));
        }
}

uint8_t get_sensor_activate(void){
	return sensor_activate;
}

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+3, Navigation, NULL);
}
