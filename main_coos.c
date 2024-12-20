/*
 * MQTT demo application for ustack-stm32
 * 
 * start the MQTT/UDP bridge (mqtt_udp) before running this application!
 */

#include <stm32f4xx_conf.h>
#include <hal.h>
#include <usart.h>
#include <coos.h>
#include <ustack.h>
#include "hw_res.h"
#include "dht.h"
#include "adc.h"
#include "pwm.h"

uint8_t eth_frame[FRAME_SIZE];
uint8_t mymac[6] = {0x0e, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t myip[4];
uint8_t mynm[4];
uint8_t mygw[4];

int LUX_MIN = -1, LUX_MAX = -1;
int TEMP_MIN_RELAY_1 = -1, TEMP_MAX_RELAY_1 = -1;
int TEMP_MIN_RELAY_2 = -1, TEMP_MAX_RELAY_2 = -1;

const float F_VOLTAGE = 635.0;		// 590 ~ 700mV typical diode forward voltage
const float T_COEFF = -2.0;			// 1.8 ~ 2.2mV change per degree Celsius
const float V_RAIL = 3300.0;		// 3300mV rail voltage
const float ADC_MAX = 4095.0;		// max ADC value
const int ADC_SAMPLES = 1024;		// ADC read samples
const int REF_RESISTANCE = 4700;

volatile float temp = 0.0, lux = 0.0, humid = 0.0;

struct dht_s dht_11;

#define SENSOR_TEMP		"testtopic/andrelucas/sensor_temp"
#define SENSOR_LUMI		"testtopic/andrelucas/sensor_lumi"
#define SENSOR_HUMID	"testtopic/andrelucas/sensor_humid"
#define CONTROLE_DIME	"testtopic/andrelucas/controle_dime"
#define RELAY_1			"testtopic/andrelucas/relay_1"
#define RELAY_2			"testtopic/andrelucas/relay_2"
#define LIMITES_DIME	"testtopic/andrelucas/limites_dime"
#define LIMITES_RELAY_1	"testtopic/andrelucas/limites_relay_1"
#define LIMITES_RELAY_2	"testtopic/andrelucas/limites_relay_2"

/* this function is called asynchronously (on MQTT messages) */
int32_t app_udp_handler(uint8_t *packet)
{
	uint8_t dst_addr[4];
	uint16_t src_port, dst_port;
	struct ip_udp_s *udp = (struct ip_udp_s *)packet;
	char *datain, *dataval;
	char data[256];

	src_port = ntohs(udp->udp.dst_port);
	dst_port = ntohs(udp->udp.src_port);

	if (ntohs(udp->udp.dst_port) == UDP_DEFAULT_PORT) {
		memcpy(dst_addr, udp->ip.src_addr, 4);
		
		datain = (char *)packet + sizeof(struct ip_udp_s);
		datain[ntohs(udp->udp.len) - sizeof(struct udp_s)] = '\0';
		
		if (strstr(datain, RELAY_1)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val;
			
			/* print received data and its origin */
			sprintf(data, "RELAY_1: [%s]", dataval);

			val = atoi(dataval);

			/* toggle RELAY_1 */
			if (TEMP_MIN_RELAY_1 == -1 && TEMP_MAX_RELAY_1 == -1){
				if (val == 1){
					//GPIO_SetBits(GPIOA, GPIO_Pin_0); // PA0
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
				}
				else {
					//GPIO_ResetBits(GPIOA, GPIO_Pin_0);  // PA0
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				}
			}
		}
		if (strstr(datain, RELAY_2)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val;
			
			/* print received data and its origin */
			sprintf(data, "RELAY_2: [%s]", dataval);

			val = atoi(dataval);

			/* toggle RELAY_2 */
			if (TEMP_MIN_RELAY_2 == -1 && TEMP_MAX_RELAY_2 == -1){
				if (val == 1){
					//GPIO_SetBits(GPIOA, GPIO_Pin_1); // PA1
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
				}
				else {
					//GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // PA1
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				}
			}			
		}
		if (strstr(datain, CONTROLE_DIME)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val;
			
			/* print received data and its origin */
			sprintf(data, "CONTROLE_DIME: [%s]", dataval);

			val = atoi(dataval);

			/* toggle DIME */
			if (LUX_MIN == -1 && LUX_MAX == -1){
				if (val != 0){
					TIM4->CCR4 = val; 		// PB9						
				}
				else{
					TIM4->CCR4 = 0; 		// PB9	
				}	
			}
		}
		if (strstr(datain, LIMITES_DIME)){
			/* Skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val_min = -1, val_max = -1;

			/* Print received data and its origin */
			sprintf(data, "LIMITES_DIME: [%s]", dataval);

			/* Split the dataval string */
			char *char_ptr = strtok(dataval, ";");
			if (char_ptr != NULL) {
				val_min = atoi(char_ptr);
				char_ptr = strtok(NULL, ";");
				if (char_ptr != NULL) {
					val_max = atoi(char_ptr);
				}
			}

			/* Update LUX_MIN and LUX_MAX */
			LUX_MIN = val_min;
			LUX_MAX = val_max;
		}
		if (strstr(datain, LIMITES_RELAY_1)){
			/* Skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val_min = -1, val_max = -1;

			/* Print received data and its origin */
			sprintf(data, "LIMITES_RELAY_1: [%s]", dataval);

			/* Split the dataval string */
			char *char_ptr = strtok(dataval, ";");
			if (char_ptr != NULL) {
				val_min = atoi(char_ptr);
				char_ptr = strtok(NULL, ";");
				if (char_ptr != NULL) {
					val_max = atoi(char_ptr);
				}
			}

			/* Update TEMP_MIN_RELAY_1 and TEMP_MAX_RELAY_1 */
			TEMP_MIN_RELAY_1 = val_min;
			TEMP_MAX_RELAY_1 = val_max;
		}
		if (strstr(datain, LIMITES_RELAY_2)){
			/* Skip topic name */
			dataval = strstr(datain, " ") + 1;
			int val_min = -1, val_max = -1;

			/* Print received data and its origin */
			sprintf(data, "LIMITES_RELAY_2: [%s]", dataval);

			/* Split the dataval string */
			char *char_ptr = strtok(dataval, ";");
			if (char_ptr != NULL) {
				val_min = atoi(char_ptr);
				char_ptr = strtok(NULL, ";");
				if (char_ptr != NULL) {
					val_max = atoi(char_ptr);
				}
			}

			/* Update TEMP_MIN_RELAY_2 and TEMP_MAX_RELAY_2 */
			TEMP_MIN_RELAY_2 = val_min;
			TEMP_MAX_RELAY_2 = val_max;
		}
	}
	
	return 0;
}

void sensor_data_lux(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];
	
	ftoa(val, buf, 3);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

void sensor_data_temp(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];
	
	ftoa(val, buf, 3);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}


void sensor_data_humid(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];
	
	ftoa(val, buf, 3);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}



/* this function is used to register a topic */
void setup_topic(uint8_t *packet, char *topic)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	
	strcpy(data, "TOPIC ");
	strcat(data, topic);
	
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}


/* resolve ARP stuff */
void net_setup(uint8_t *packet)
{
	uint16_t i;
	
	for (i = 0; i < 100; i++) {
		netif_recv(packet);
		delay_ms(100);
	}

	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
}

float temperature()
{
	float temp_internal = 0.0;
	float voltage;
	
	for (int i = 0; i < ADC_SAMPLES; i++) {
		voltage = adc_read() * (V_RAIL / ADC_MAX);
		temp_internal += ((voltage - F_VOLTAGE) / T_COEFF);
	}
	
	return (temp_internal / ADC_SAMPLES);
}

float luminosity()
{
	float voltage, lux_internal = 0.0, rldr;
	
	for (int i = 0; i < ADC_SAMPLES; i++) {
		voltage = adc_read() * (V_RAIL / ADC_MAX);
		rldr = (REF_RESISTANCE * (V_RAIL - voltage)) / voltage;
		lux_internal += 500 / (rldr / 650);
	}
	
	return (lux_internal / ADC_SAMPLES);
}

float task_temp(void)
{	
	adc_channel(ADC_Channel_8); // PB0
	temp = temperature();
	
	return temp;
}

float task_temp_dht(void)
{	
	int val;
	
	val = dht_read(&dht_11);
	
	if (val == ERR_ERROR)
		printf("Sensor DHT_11 error: %d\n", val);
	else {
		temp = (float)(dht_11.temperature / 10) + (float)(dht_11.temperature % 10) / 10.0f;
		humid = (float)(dht_11.humidity / 10) + (float)(dht_11.humidity % 10) / 10.0f;
	}
		
	return temp;
}

float task_lux(void)
{
	adc_channel(ADC_Channel_9);
	lux = luminosity();

	return lux;
}

float task_humid_dht(void)
{
	return humid;
}

void *task_relay_1()
{
	if (TEMP_MIN_RELAY_1 != -1 && TEMP_MAX_RELAY_1 != -1){
		if (temp >= TEMP_MIN_RELAY_1 && temp <= TEMP_MAX_RELAY_1){
			GPIO_SetBits(GPIOA, GPIO_Pin_0); // PA0
		}
		else {
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);  // PA0
		}
	}

	return 0;
}

void *task_relay_2()
{
	if (TEMP_MIN_RELAY_2 != -1 && TEMP_MAX_RELAY_2 != -1){
		if (temp >= TEMP_MAX_RELAY_2 && temp <= TEMP_MAX_RELAY_2){
			GPIO_SetBits(GPIOA, GPIO_Pin_1); // PA1
		}
		else {
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // PA1
		}
	}

	return 0;
}

void *task_dime()
{
	if (LUX_MIN != -1 && LUX_MAX != -1){
		if (lux < LUX_MIN){
			TIM4->CCR4 = 999; 		// PB9						
		}
		else if (lux > LUX_MAX){
			TIM4->CCR4 = 0;			// PB9
		}
		else{
			TIM4->CCR4 = (int)(999 * (1 - (lux / LUX_MAX)));
		}
	}	

	return 0;
}


/* application tasks */
void *network_task(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	uint16_t len;
	
	len = netif_recv(packet);

	if (len > 0) {
		/* turn board LED on */
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		
		ip_in(myip, packet, len);
	
		/* turn board LED off */
		GPIO_SetBits(GPIOC, GPIO_Pin_13);	
	}
	
	return 0;
}

void *sensor_task(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	
	if (sensor_poll_data()) {
		sensor_data_temp(packet, SENSOR_TEMP, task_temp_dht());
		sensor_data_humid(packet, SENSOR_HUMID, task_humid_dht());
		//sensor_data_lux(packet, SENSOR_LUMI, task_lux());
	}
	
	return 0;
}


int main(void)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	struct task_s tasks[MAX_TASKS] = { 0 };
	struct task_s *ptasks = tasks;
	uint16_t len;

	analog_config();
	adc_config();
	pwm_config();
	dht_setup(&dht_11, DHT11, RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_7); // PA7
	
	/* setup LED */
	led_init();
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	
	/* setup timer 2 */
	tim2_init();
	
	/* setup network stack */
	if_setup();
	config(mymac + 2, USTACK_IP_ADDR);
	config(myip, USTACK_IP_ADDR);
	config(mynm, USTACK_NETMASK);
	config(mygw, USTACK_GW_ADDR);
	
	net_setup(packet);
	udp_set_callback(app_udp_handler);
	
	/* create MQTT topics */
	setup_topic(packet, SENSOR_TEMP);
	setup_topic(packet, SENSOR_LUMI);
	setup_topic(packet, SENSOR_HUMID);
	setup_topic(packet, CONTROLE_DIME);
	setup_topic(packet, RELAY_1);
	setup_topic(packet, RELAY_2);
	setup_topic(packet, LIMITES_DIME);
	setup_topic(packet, LIMITES_RELAY_1);
	setup_topic(packet, LIMITES_RELAY_2);
	
	/* setup CoOS and tasks */
	task_pinit(ptasks);
	task_add(ptasks, network_task, 50);
	task_add(ptasks, sensor_task, 100);
	task_add(ptasks, task_relay_1, 100);
	task_add(ptasks, task_relay_2, 100);
	task_add(ptasks, task_dime, 100);
	
	while (1) {
		//task_schedule(ptasks);

		len = netif_recv(packet);

		if (len > 0) {
			ip_in(myip, packet, len);
		}
		
		if (sensor_poll_data()) {
			sensor_data_temp(packet, SENSOR_TEMP, task_temp_dht());
			sensor_data_humid(packet, SENSOR_HUMID, task_humid_dht());
		}

		if (TEMP_MIN_RELAY_1 != -1 && TEMP_MAX_RELAY_1 != -1){
			if (temp >= TEMP_MIN_RELAY_1 && temp <= TEMP_MAX_RELAY_1){
				GPIO_SetBits(GPIOA, GPIO_Pin_0); // PA0
			}
			else {
				GPIO_ResetBits(GPIOA, GPIO_Pin_0);  // PA0
			}
		}

		if (TEMP_MIN_RELAY_2 != -1 && TEMP_MAX_RELAY_2 != -1){
			if (temp >= TEMP_MAX_RELAY_2 && temp <= TEMP_MAX_RELAY_2){
				GPIO_SetBits(GPIOA, GPIO_Pin_1); // PA1
			}
			else {
				GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // PA1
			}
		}

		if (LUX_MIN != -1 && LUX_MAX != -1){
			if (lux < LUX_MIN){
				TIM4->CCR4 = 999; 		// PB9						
			}
			else if (lux > LUX_MAX){
				TIM4->CCR4 = 0;			// PB9
			}
			else{
				TIM4->CCR4 = (int)(999 * (1 - (lux / LUX_MAX)));
			}
		}

	}

	return 0;
}
