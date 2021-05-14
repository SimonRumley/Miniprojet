

#ifndef AUDIO_PROCESSING_H_
#define AUDIO_PROCESSING_H_
#define FFT_SIZE 	1024
#define MIN_VALUE_THRESHOLD	10000

#define MIN_FREQ		150	//we don't analyze before this index to not use resources for nothing
#define FREQ_LEFT		200 //3000Hz ici //ref 406Hz pour 26Hz ,
#define MAX_FREQ		250	//we don't analyze after this index to not use resources for nothing

#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define INIT_MAX_NORM_INDEX -1

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)

	FRONT_CMPLX_INPUT= 0,

	//Arrays containing the computed magnitude of the complex numbers

	FRONT_OUTPUT,

} BUFFER_NAME_t;


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);
uint16_t get_max_norm_index(void);

void doFFT_optimized(uint16_t size, float* complex_buffer);
void audio_start(void);

#endif /* AUDIO_PROCESSING_H_ */
