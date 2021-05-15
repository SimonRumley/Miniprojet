#ifndef NAVIGATION_H
#define NAVIGATION_H
#define ROTATION_COEFF			   1
#define ERROR_THRESHOLD			   5.0f
#define KP						   0.6f
#define SPEED_LOW			       600.0f
#define SPEED_GREEN  			   100
#define TURNING_SPEED 			   600
#define DISTANCE_READ_TRAFIC_LIGHT 150
#define DISTANCE_SECURITY 		   80
#define INIT_DELTA 				   10
#define DELTA_RANGE 			   2
#define MIC_COUNTER_MAX 		   3


//start le thread Navigation
void navigation_start(void);
//verifie si un objet est detecté a moins de 15cm
uint8_t get_sensor_activate(void);

#endif /* NAVIGATION_H */
