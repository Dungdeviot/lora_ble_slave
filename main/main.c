#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "driver/gpio.h"
#include "string.h"
#include"driver/adc.h"
#include "driver/mcpwm.h"
#include"esp_adc_cal.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "ultrasonic.h"
#include "esp_timer.h"

uint32_t voltage1;
SemaphoreHandle_t xSemaphore = NULL;
bool flag = 0;
int per = 0;
int soil = 0;
bool auto_mode = false;
bool error = false;

#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 36

static esp_adc_cal_characteristics_t *adc_chars1;
static const adc_channel_t channel1 = ADC_CHANNEL_3; // ADC1 su dung GPIO 39
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Khai bao prototype
void task_rx(void *p);
void adc_init();
void pwm_init();
void task_tx_read_sensor();
char num_to_char(char num);
int char_to_num(char c);

//	Callback fuction
static void timer_callback(void* arg)
{
}

//	Timer args
const esp_timer_create_args_t timer_args = {
	.callback = &timer_callback,
	.name = "Timer"
};

void app_main() {
	esp_timer_handle_t timer;
	ESP_ERROR_CHECK(esp_timer_create(&timer_args,&timer));

	pwm_init();
	adc_init();
	ultrasonic_setup_pins();
	lora_init();
	lora_set_frequency(433e6);
	lora_enable_crc();
	// Tạo cờ binary semaphore
	//xSemaphore = xSemaphoreCreateBinary();
	gpio_set_direction(4, GPIO_MODE_OUTPUT);

	xTaskCreatePinnedToCore(task_rx,"task RX",10000,NULL,1,NULL,0);
	xTaskCreatePinnedToCore(task_tx_read_sensor,"Task TX",10000,NULL,1,NULL,1);
}//End main

void task_rx(void *p) {
	printf("task_rx is running on Core: %d\n",xPortGetCoreID());
	for(;;) {
		lora_receive();    // put into receive mode
		if(lora_received()) {
			uint8_t buff_rx[8];
			lora_receive_packet(buff_rx, sizeof(buff_rx));	//Nhan data

			int len = strlen((const char*)buff_rx);
			char mode = buff_rx[len - 1];				//Xet dia chi data gui den
			buff_rx[len - 1] = '\0';

			if(mode == 'A') {				//Dia chi 'A'
				int i = 0;
				int num = 0;
				while(buff_rx[i] != '\0'){
					num = num*10 + char_to_num(buff_rx[i]);	//Chuyen chuoi ky tu thanh so nguyen
					i++;
				}
				per = num;

			}else if (mode == 'B') {		//Dia chi 'B'
				int i = 0;
				int num = 0;
				while(buff_rx[i] != '\0'){
					num = num*10 + char_to_num(buff_rx[i]);	//Chuyen chuoi ky tu thanh so nguyen
					i++;
				}
				soil = num;
				auto_mode = true;		//Bat auto mode

			} else {
				if(!strcmp((const char*)buff_rx,"ON") && !error) {
					gpio_set_level(4, 1);
					flag = true;
					auto_mode = false;		//Tat auto mode
				}
				if(!strcmp((const char*)buff_rx,"OF")) {
					gpio_set_level(4, 0);
					flag = false;
					auto_mode = false;		//Tat auto mode
				}
			}

			printf("Received: %s\n", buff_rx);

			lora_receive();
			//xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void task_tx_read_sensor() {
	printf("task_tx_read_sensor is running on Core: %d\n",xPortGetCoreID());
	double distance = 0.0;
//	double calib_distance = ultrasonic_caliberate_sensor();		//Ghi khoang cach vi tri ban dau
	for(;;) {
		char buff[16];	//buff TX
		//==========================read status motor===========================================================
		buff[0] = '2';	//Length of status
		if(flag){
			buff[4] = 'O';
			buff[5] = 'N';
		}
		if(!flag){
			buff[4] = 'O';
			buff[5] = 'F';
		}

		buff[1] = 'x';
		buff[2] = 'x';
		buff[3] = 'x';

		buff[6] = '\0';	//End string status

		//==========================read ADC===========================================================
		uint32_t adc_reading1 = 0;
		// Multisampling
		for (int i = 0; i < NO_OF_SAMPLES; i++) {
			adc_reading1 += adc1_get_raw((adc1_channel_t)channel1);
			vTaskDelay(1);
		}
		adc_reading1 /= NO_OF_SAMPLES;
		//Convert adc_reading to voltage in mV
		voltage1 = esp_adc_cal_raw_to_voltage(adc_reading1, adc_chars1);
		voltage1 = map((double)voltage1,3350,1250,0,100);

		if((voltage1 < soil) && auto_mode && !error) {		//Xet do am dat ON/OFF motor
			gpio_set_level(4, 1);
			flag = true;
		} else if((voltage1 >= (soil+20)) && auto_mode) {
			gpio_set_level(4, 0);
			flag = false;
		}

		char buff_soil[4];
		snprintf(buff_soil, sizeof(buff_soil), "%d", voltage1);

		size_t len_soil = strlen(buff_soil);
		buff[1] = num_to_char((char)len_soil);	// length of soil
		strncat(buff,buff_soil,len_soil);


		//==========================read DIST===========================================================
		char buff_ultra[8];
//		distance = ultrasonic_get_distance_in(); //Lay gia tri cam bien
//		distance  = calib_distance - distance;
		distance  = 12.34;
		snprintf(buff_ultra, sizeof(buff_ultra), "%0.2f", distance);

		size_t len_ultra = strlen(buff_ultra);
		buff[2] = num_to_char((char)len_ultra);
		strncat(buff,buff_ultra,len_ultra);

		if(flag && (distance <=5 )) {	//Tat motor khi muc nuoc <= 5cm
			gpio_set_level(4, 0);
			flag = false;
			error = true;						//Bat co bao loi
		} else if(distance > 5) error = false;	//Tat co bao loi

		//==========================read PWM===========================================================
		char buff_pwm[4];
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, per);	//Thay doi do sang LED
		snprintf(buff_pwm, sizeof(buff_pwm), "%d", per);

		size_t len_pwm = strlen(buff_pwm);
		buff[3] = num_to_char((char)len_pwm);
		strncat(buff,buff_pwm,len_pwm);

		//==========================send data===========================================================
		lora_send_packet((uint8_t*)buff, strlen(buff)+1);
		printf("Buff RX: %s\n",buff);

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

//3 --> '3'
char num_to_char(char num) {
	return num + 48;
}

int char_to_num(char c) {
	return c - 48;
}

void adc_init(){
	adc1_config_width(width);
	adc1_config_channel_atten(channel1, atten);
	//Characterize ADC
	adc_chars1 = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars1);
}

void pwm_init(){
	mcpwm_config_t pwm_config;
	// PWM init
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 2);
	//frequency = 1000Hz
	pwm_config.frequency = 1000;
	//duty cycle of PWMxb = 0.0%
	pwm_config.cmpr_b = 0.0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	//Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

//void task_tx_ultrasonic() {
//	printf("task_tx_ultrasonic is running on Core: %d\n",xPortGetCoreID());
//	double distance = 0;
//	ultrasonic_setup_pins();
//
//	for(;;) {
//		char buff[10];
//		distance = ultrasonic_get_distance_in();
//		snprintf(buff, sizeof(buff), "%0.2f", distance);
//
//		size_t len = strlen(buff);
//		buff[len] = 'A';
//		buff[len+1] = '\0';
//
//		lora_send_packet((uint8_t*)buff, strlen(buff)+1);
//		vTaskDelay(pdMS_TO_TICKS(2000));
//	}
//}
//
//
//void task_ADC() {
//	printf("task_ADC is running on Core: %d\n",xPortGetCoreID());
//	for(;;) {
//		//printf("abc");
//		uint32_t adc_reading1 = 0;
//		// Multisampling
//		for (int i = 0; i < NO_OF_SAMPLES; i++) {
//			adc_reading1 += adc1_get_raw((adc1_channel_t)channel1);
//			vTaskDelay(10);
//			//printf("1");
//		}
//		adc_reading1 /= NO_OF_SAMPLES;
//		//Convert adc_reading to voltage in mV
//		voltage1 = esp_adc_cal_raw_to_voltage(adc_reading1, adc_chars1);
//
//		voltage1 = map((double)voltage1,3350,1250,0,99);
//
//		//printf("Voltage: %dpercent\n",voltage1);
//		char buff[10];
//		snprintf(buff, sizeof(buff), "%d", voltage1);
//
//		size_t len = strlen(buff);
//		buff[len] = 'B';
//		buff[len+1] = '\0';
//
//		lora_send_packet((uint8_t*)buff, strlen(buff)+1);
//
//		vTaskDelay(pdMS_TO_TICKS(1000));
//	}
//}
//
//void task_tx_response(void *p) {
//	printf("task_tx_response is running on Core: %d\n",xPortGetCoreID());
//	for(;;) {
//		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE){
//
//			char LED[10];
//			if(flag){
//				snprintf(LED, sizeof(LED), "ON");
//				size_t len = strlen(LED);
//				LED[len] = 'C';
//				LED[len+1] = '\0';
//				lora_send_packet((uint8_t*)LED, strlen(LED)+1);
//			}
//			if(!flag){
//				snprintf(LED, sizeof(LED), "OF");
//				size_t len = strlen(LED);
//				LED[len] = 'C';
//				LED[len+1] = '\0';
//				lora_send_packet((uint8_t*)LED, strlen(LED)+1);
//			}
//			printf("Response...\n");
//		}
//		vTaskDelay(pdMS_TO_TICKS(10));
//   }
//}
