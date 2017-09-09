/*
 * ciot.h
 *
 *  Created on: 2017/08/20
 *      Author: SpiralRay
 */

#ifndef CIOT_H_
#define CIOT_H_

#include "main.h"

#define GPS_POWER_ON()  { HAL_GPIO_WritePin(GPS_ENABLE_GPIO_Port, GPS_ENABLE_Pin, GPIO_PIN_SET); }
#define GPS_POWER_OFF() { HAL_GPIO_WritePin(GPS_ENABLE_GPIO_Port, GPS_ENABLE_Pin, GPIO_PIN_RESET); }

#define SAKURAIO_POWER_ON()  { HAL_GPIO_WritePin(SAKURA_WAKE_IN_GPIO_Port, SAKURA_WAKE_IN_Pin, GPIO_PIN_SET); }
#define SAKURAIO_POWER_OFF() { HAL_GPIO_WritePin(SAKURA_WAKE_IN_GPIO_Port, SAKURA_WAKE_IN_Pin, GPIO_PIN_RESET); }

#define SENSE_PERIOD 3
#define THRESHOLD_TRANSMIT_QUEUE_SIZE 20

#define BATTERY_LOW_VOLTAGE 3.35f
#define SLEEP_TIME  300 //sec

int ciot_init();
void ciot_main();

float get_voltage();
void parse_gps();

#endif /* CIOT_H_ */
