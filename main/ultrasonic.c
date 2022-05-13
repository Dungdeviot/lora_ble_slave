#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ultrasonic.h"
#include "math.h"

uint64_t timer_start;
uint64_t timer_stop;

double map(double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

double calib_bac3(double distance){
	return distance + (0.00000075*pow(distance,3)-0.0002393*pow(distance,2)+0.05385*pow(distance,1)-3.847);
}

double calib_bac1(double distance){
	return distance + (0.03069*distance-3.202);
}

double calib_bacb(double distance){
	return distance + (0.06045*pow(distance,0.8756)-3.511);
}

void ultrasonic_setup_pins()
{
    //Config the Trig Pin
    gpio_pad_select_gpio(TRIG_GPIO_NUM);
    gpio_set_direction(TRIG_GPIO_NUM,GPIO_MODE_OUTPUT);

    //Config the Echo Pin
    gpio_pad_select_gpio(ECHO_GPIO_NUM);
    gpio_set_direction(ECHO_GPIO_NUM,GPIO_MODE_INPUT);
}

double ultrasonic_get_distance_in()
{
	double distance = 0.0;
	double distance_total = 0.0;
	bool no_Signal = true;

	gpio_set_level(TRIG_GPIO_NUM, HIGH);
	vTaskDelay(pdMS_TO_TICKS(0.01));
	gpio_set_level(TRIG_GPIO_NUM, LOW);
	timer_start = esp_timer_get_time();

	while(no_Signal)
	{
		if(gpio_get_level(ECHO_GPIO_NUM) == HIGH)
		{
			no_Signal = false;
			while(gpio_get_level(ECHO_GPIO_NUM) == HIGH);
			timer_stop = esp_timer_get_time();
		}
	}
	distance = (timer_stop - timer_start)*0.034/2;
	for (int i=0; i<36; i++)
	{
		vTaskDelay(pdMS_TO_TICKS(JSN_MEASUREMENT_CYCLE_MS));
		distance_total += distance;
	}
	distance = distance_total / 36.0;
	distance = calib_bac1(distance);
    return distance;
}


double ultrasonic_caliberate_sensor()
{
	double distance = 0.0;
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds for sensor stability

    for(int i = 0; i < 36; i++)
    {
        // Get 15 readings and average them
        distance += ultrasonic_get_distance_in();
        vTaskDelay(pdMS_TO_TICKS(JSN_MEASUREMENT_CYCLE_MS));
    }
    return distance / 36.0;
}
