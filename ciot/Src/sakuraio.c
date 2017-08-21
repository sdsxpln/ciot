/*
 * sakuraio.c
 *
 *  Created on: 2017/08/21
 *      Author: SpiralRay
 */

#include "sakuraio.h"
#include "main.h"
#include <string.h>

#define SAKURAIO_BUFFERSIZE 16

static __IO uint8_t      sakuraio_buffer[SAKURAIO_BUFFERSIZE];
static I2C_HandleTypeDef *sakuraio_i2c = NULL;

void SakuraIO_Init(I2C_HandleTypeDef *hi2c){
    sakuraio_i2c = hi2c;
}

HAL_StatusTypeDef SakuraIO_ExecuteCommand(uint8_t cmd,uint8_t requestLength, uint8_t *request, uint8_t *responseLength, uint8_t *response){
    uint8_t parity = 0x00;

    sakuraio_buffer[0] = cmd;
    sakuraio_buffer[1] = requestLength;

    //Put the parity byte into the end of the request buffer
    parity = cmd ^ requestLength;
    for(int16_t i=0; i<requestLength; i++){
        parity ^= request[i];
        sakuraio_buffer[i+2] = request[i];
    }
    sakuraio_buffer[requestLength+2] = parity;

    //cmd byte, requestLength byte and parity byte
    requestLength += 3;

    if(HAL_I2C_Master_Sequential_Transmit_IT(sakuraio_i2c, (uint16_t)SAKURAIO_SLAVE_ADDR, (uint8_t *)sakuraio_buffer, requestLength, I2C_LAST_FRAME)!= HAL_OK)
    {
        return CMD_ERROR_RUNTIME;
    }

    /*##-3- Wait for the end of the transfer #################################*/
    /*  Before starting a new communication transfer, you need to check the current
            state of the peripheral; if it is busy you need to wait for the end of current
            transfer before starting a new one.
            For simplicity reasons, this example is just waiting till the end of the
            transfer, but application may perform other tasks while transfer operation
            is ongoing. */
    while (HAL_I2C_GetState(sakuraio_i2c) != HAL_I2C_STATE_READY)
    {
    }

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address) Master restarts communication */
    if( HAL_I2C_GetError(sakuraio_i2c) == HAL_I2C_ERROR_AF ){
        return CMD_ERROR_RUNTIME;
    }

    if(HAL_I2C_Master_Sequential_Receive_IT(sakuraio_i2c, (uint16_t)SAKURAIO_SLAVE_ADDR, (uint8_t *)sakuraio_buffer, SAKURAIO_BUFFERSIZE, I2C_LAST_FRAME) != HAL_OK)
    {
        return CMD_ERROR_RUNTIME;
    }

    /*##-5- Wait for the end of the transfer #################################*/
    /*  Before starting a new communication transfer, you need to check the current
            state of the peripheral; if it’s busy you need to wait for the end of current
            transfer before starting a new one.
            For simplicity reasons, this example is just waiting till the end of the
            transfer, but application may perform other tasks while transfer operation
            is ongoing. */
    while (HAL_I2C_GetState(sakuraio_i2c) != HAL_I2C_STATE_READY)
    {
    }

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
           Master restarts communication */
    if(HAL_I2C_GetError(sakuraio_i2c) == HAL_I2C_ERROR_AF){
        return CMD_ERROR_RUNTIME;
    }

    //Check command status
    if( sakuraio_buffer[0] != CMD_ERROR_NONE ){
        return sakuraio_buffer[0];
    }

    *responseLength = sakuraio_buffer[1];

    //Check parity
    parity = 0x00;
    for(int i=0; i<(*responseLength + 2); i++){
        parity ^= sakuraio_buffer[i];
    }
    if( parity != sakuraio_buffer[*responseLength + 2] ){
        return CMD_ERROR_PARITY;
    }

    if( *responseLength > 0 ){
        memcpy(response, (uint8_t *)(sakuraio_buffer+2), *responseLength);
    }

    return CMD_ERROR_NONE;
}

uint8_t SakuraIO_GetConnectionStatus(){
  uint8_t responseLength = 1;
  uint8_t response[1] = {0x00};
  if(SakuraIO_ExecuteCommand(CMD_GET_CONNECTION_STATUS, 0, NULL, &responseLength, response) != CMD_ERROR_NONE){
    return 0x7F;
  }
  return response[0];
}

uint8_t SakuraIO_GetSignalQuality(){
  uint8_t responseLength = 1;
  uint8_t response[1] = {0x00};

  if(SakuraIO_ExecuteCommand(CMD_GET_SIGNAL_QUALITY, 0, NULL, &responseLength, response) != CMD_ERROR_NONE){
    return 0x00;
  }
  return response[0];
}

uint64_t SakuraIO_GetUnixtime(){
  uint8_t responseLength = 8;
  uint8_t response[8] = {0x00};
  if(SakuraIO_ExecuteCommand(CMD_GET_DATETIME, 0, NULL, &responseLength, response) != CMD_ERROR_NONE){
    return 0x00;
  }
  return *((uint64_t *)response);
}

uint8_t SakuraIO_Echoback(uint8_t length, uint8_t *data, uint8_t *response){
  uint8_t responseLength = length;
  if(SakuraIO_ExecuteCommand(CMD_ECHO_BACK, length, data, &responseLength, response) != CMD_ERROR_NONE){
    return 0x00;
  }
  return responseLength;
}

/* TX Commands */
uint8_t SakuraIO_EnqueueTxRaw(uint8_t ch, uint8_t type, uint8_t length, uint8_t *data, uint64_t offset){
  uint8_t request[18] = {0x00};
  uint8_t requestLength = 10;
  request[0] = ch;
  request[1] = type;
  for(uint8_t i=0;i<length;i++){
    request[2+i] = data[i];
  }
  if(offset != 0){
    requestLength = 18;
    for(uint8_t i=0;i<8;i++){
      request[10+i] = ((uint8_t *)&offset)[i];
    }
  }
  return SakuraIO_ExecuteCommand(CMD_TX_ENQUEUE, requestLength, request, NULL, NULL);
}

uint8_t SakuraIO_EnqueueInt32(uint8_t ch, int32_t value, uint64_t offset){
  return SakuraIO_EnqueueTxRaw(ch, 'i', 4, (uint8_t *)&value, offset);
}

uint8_t SakuraIO_EnqueueUint32(uint8_t ch, uint32_t value, uint64_t offset){
  return SakuraIO_EnqueueTxRaw(ch, 'I', 4, (uint8_t *)&value, offset);
}

uint8_t SakuraIO_EnqueueFloat(uint8_t ch, float value, uint64_t offset){
  return SakuraIO_EnqueueTxRaw(ch, 'f', 4, (uint8_t *)&value, offset);
}

uint8_t SakuraIO_EnqueueByte(uint8_t ch, uint8_t value[8], uint64_t offset){
  return SakuraIO_EnqueueTxRaw(ch, 'b', 8, (uint8_t *)value, offset);
}

uint8_t SakuraIO_GetTxQueueLength(uint8_t *available, uint8_t *queued){
  uint8_t response[2] = {0x00};
  uint8_t responseLength = 2;
  uint8_t ret = SakuraIO_ExecuteCommand(CMD_TX_LENGTH, 0, NULL, &responseLength, response);
  *available = response[0];
  *queued = response[1];
  return ret;
}

uint8_t SakuraIO_ClearTx(){
  return SakuraIO_ExecuteCommand(CMD_TX_CLEAR, 0, NULL, NULL, NULL);
}

uint8_t SakuraIO_Send(){
  return SakuraIO_ExecuteCommand(CMD_TX_SEND, 0, NULL, NULL, NULL);
}

uint8_t SakuraIO_GetTxStatus(uint8_t *queue, uint8_t *immediate){
  uint8_t response[2] = {0x00};
  uint8_t responseLength = 2;
  uint8_t ret = SakuraIO_ExecuteCommand(CMD_TX_STAT, 0, NULL, &responseLength, response);
  *queue = response[0];
  *immediate = response[1];
  return ret;
}

/* RX Commands */

uint8_t SakuraIO_DequeueRx(uint8_t *ch, uint8_t *type, uint8_t *value, int64_t *offset){
  uint8_t response[18] = {0x00};
  uint8_t responseLength = 18;
  uint8_t ret = SakuraIO_ExecuteCommand(CMD_RX_DEQUEUE, 0, NULL, &responseLength, response);
  if(ret != CMD_ERROR_NONE){
    return ret;
  }

  *ch = response[0];
  *type = response[1];
  for(uint8_t i=0; i<8; i++){
    value[i] = response[2+i];
  }
  for(uint8_t i=0; i<8; i++){
    ((uint8_t *)offset)[i] = response[10+i];
  }

  return ret;
}

uint8_t SakuraIO_PeekRx(uint8_t *ch, uint8_t *type, uint8_t *value, int64_t *offset){
  uint8_t response[18] = {0x00};
  uint8_t responseLength = 18;
  uint8_t ret = SakuraIO_ExecuteCommand(CMD_RX_PEEK, 0, NULL, &responseLength, response);
  if(ret != CMD_ERROR_NONE){
    return ret;
  }

  *ch = response[0];
  *type = response[1];
  for(uint8_t i=0; i<8; i++){
    value[i] = response[2+i];
  }
  for(uint8_t i=0; i<8; i++){
    ((uint8_t *)offset)[i] = response[10+i];
  }

  return ret;
}

uint8_t SakuraIO_GetRxQueueLength(uint8_t *available, uint8_t *queued){
  uint8_t response[2] = {0x00};
  uint8_t responseLength = 2;
  uint8_t ret = SakuraIO_ExecuteCommand(CMD_RX_LENGTH, 0, NULL, &responseLength, response);
  *available = response[0];
  *queued = response[1];
  return ret;
}

uint8_t SakuraIO_ClearRx(){
  return SakuraIO_ExecuteCommand(CMD_RX_CLEAR, 0, NULL, NULL, NULL);
}
