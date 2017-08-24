/*
 * ciot_queue.h
 *
 *  Created on: 2017/08/23
 *      Author: SpiralRay
 */

#ifndef CIOT_QUEUE_H_
#define CIOT_QUEUE_H_

#include "stm32l0xx_hal.h"

typedef struct{
        uint32_t speed;
        double lat;
        double lng;
        float temp;
        float press;
        uint32_t distance;
        float battery_voltage;
        uint32_t rtc_cnt;
        uint32_t status;
} CiotBuffer;

#define CIOT_BUFFER_SIZE 20

void ciot_queue_init();
int ciot_enqueue(CiotBuffer *buff);
int ciot_dequeue(CiotBuffer *buff);
int ciot_get_queue_count();

#endif /* CIOT_QUEUE_H_ */
