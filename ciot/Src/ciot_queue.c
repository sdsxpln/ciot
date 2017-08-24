/*
 * ciot_queue.c
 *
 *  Created on: 2017/08/23
 *      Author: SpiralRay
 */


#include "ciot_queue.h"
#include <string.h>

static CiotBuffer ciot_buff[CIOT_BUFFER_SIZE];
static int ciot_head, ciot_tail;

void ciot_queue_init(){
    ciot_head = 0;
    ciot_tail = 0;
}

int ciot_enqueue(CiotBuffer *buff){
    int t = (ciot_tail+1) % CIOT_BUFFER_SIZE;

    //if buffer is full
    if( t == ciot_head ){
        return -1;
    }

    memcpy( &(ciot_buff[ciot_tail]), buff, sizeof(CiotBuffer));
    ciot_tail = t;
    return 0;
}

int ciot_dequeue(CiotBuffer *buff){

    //if buffer is empty
    if( ciot_head == ciot_tail ){
        return -1;
    }

    memcpy( buff, &(ciot_buff[ciot_head]), sizeof(CiotBuffer));
    ciot_head = (ciot_head+1) % CIOT_BUFFER_SIZE;

    return 0;
}

int ciot_get_queue_count(){
    int retval = ciot_tail - ciot_head;
    if( retval < 0 ){
        retval += CIOT_BUFFER_SIZE;
    }
    return retval;
}
