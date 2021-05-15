#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
float get_line_position(void);
uint8_t get_color_trafic_light(void);
//permet de trouver la position du milieu d'un ligne
void extract_line_position(uint8_t *buffer);
//start the ProcessImage and CaptureImage thread
void process_image_line_start(void);

#define MSK_GREENa 				0b00000111
#define MSK_GREENb 				0b11100000
#define MSK_RED 				0xF8
#define COLOR_RED 				1
#define COLOR_GREEN 			2
#define LINE_FLOOR 				478
#define LINE_FRONT 				10
#define IMAGE_BUFFER_LOW 		200
#define IMAGE_BUFFER_HIGH 		440
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40

#endif /* PROCESS_IMAGE_H */

