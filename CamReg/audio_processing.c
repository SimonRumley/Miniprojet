#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include "memory_protection.h"
#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_const_structs.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micFront_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micFront_output[FFT_SIZE];
static int16_t max_norm_index = INIT_MAX_NORM_INDEX;


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){

	float max_norm = MIN_VALUE_THRESHOLD;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}

	}
}

/*
*	Callback called when the demodulation of the front microphone is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micFront_output);
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){

	if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else{
		return NULL;
	}
}


/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

static THD_WORKING_AREA(waReadAudio, 1024);
static THD_FUNCTION(ReadAudio, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];
    while(1){

    	time = chVTGetSystemTime();
    	//we copy the buffer to avoid conflicts
    	arm_copy_f32(get_audio_buffer_ptr(FRONT_OUTPUT), send_tab, FFT_SIZE);
    	float* bufferCmplxInput = get_audio_buffer_ptr(FRONT_CMPLX_INPUT);
    	float* bufferOutput = get_audio_buffer_ptr(FRONT_OUTPUT);
    	//do a FFT to have the values in frequency (not time)
		doFFT_optimized(FFT_SIZE, bufferCmplxInput);
		arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);
		// Refresh 12,5 Hz.
		chThdSleepUntilWindowed(time, time + MS2ST(80));
        }
}

void audio_start(void){
	chThdCreateStatic(waReadAudio, sizeof(waReadAudio), NORMALPRIO+2, ReadAudio, NULL);
}

uint16_t get_max_norm_index(void){
	return max_norm_index;
}

