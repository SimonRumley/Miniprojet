/*
 * Ce mini-projet a ete cree par Simon Rumley et Milan Magnani sur la base des TPs 4 et 5
 * du cours Systemes embarques et robotique enseigne par le professeur Francesco Mondada
 *
 * Auteurs: Milan Magnani et Simon Rumley
 * Date: avril-mai 2021
 * Lieu: Ecublens, Lausanne
 *
 */
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
#include <chprintf.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>

#include <process_image.h>
#include <capteur.h>
#include "navigation.h"

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
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	//inits the motors
	motors_init();

	//inits the TOF
	VL53L0X_start();

	//starts the threads for the navigation and the processing of the image
	navigation_start();
	process_image_line_start();

	//starts the threads for the sensor
	capteur_start();

	//stars the threads for the mic
	mic_start(&processAudioData);
	audio_start();

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
