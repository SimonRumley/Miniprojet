#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <capteur.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <motors.h>
#include <process_image.h>
#include <main.h>

static	uint16_t object_distance = 0;

static THD_WORKING_AREA(waCapteur, 256);
static THD_FUNCTION(Capteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    while(1)
    {
    	time = chVTGetSystemTime();
    	object_distance = VL53L0X_get_dist_mm();
    	//chprintf((BaseSequentialStream *)&SDU1,"temps1: %d \n", time);
    	chThdSleepUntilWindowed(time, time+MS2ST(50));// Refresh @  Hz. temps execution propre pris en compte
    }
}


uint16_t get_object_distance(void){
	return object_distance;
}


void capteur_start(void) {
	chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+1, Capteur, NULL);
}
