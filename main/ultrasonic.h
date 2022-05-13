#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#define JSN_MEASUREMENT_CYCLE_MS 10	// 1s
#define TRIG_GPIO_NUM GPIO_NUM_16
#define ECHO_GPIO_NUM GPIO_NUM_17
#define HIGH 	1
#define LOW 	0

void ultrasonic_setup_pins(); //Setup all the pins and such to get the GPIO ready to be used
double ultrasonic_get_distance_in(); // get distance
double ultrasonic_caliberate_sensor(); // Caliberate the Sensor
double map(double x, double in_min, double in_max, double out_min, double out_max);
double calib_bac3(double distance);
double calib_bac1(double distance);
double calib_bacb(double distance);

#endif
