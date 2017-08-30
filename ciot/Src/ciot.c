/*
 * ciot.c
 *
 *  Created on: 2017/08/20
 *      Author: SpiralRay
 */

#include "ciot.h"
#include "stm32l0xx_hal.h"
#include <ctype.h>
#include <time.h>
#include "uart_support.h"

#include "bsp_pressure.h"
#include "bsp_temperature.h"
#include "sakuraio.h"

#include "ciot_queue.h"

extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

static PRESSURE_Drv_t *LPS25HB_P_handle = NULL;
static TEMPERATURE_Drv_t *LPS25HB_T_handle = NULL;

char gps_line[512];
int gps_index =0;
uint8_t gps_active =false;

double lat=0.0, lng=0.0;

volatile uint32_t rtc_cnt = 0;
volatile time_t unixtime = 0;

volatile uint32_t reed_cnt = 0;
volatile uint32_t speed = 0;
volatile uint32_t distance = 0;

static void RTC_AlarmConfig(void);

int ciot_init(){
    gps_index =0;
    gps_active =false;
    lat=0.0;
    lng=0.0;
    rtc_cnt = 0;

    ciot_queue_init();

    SAKURAIO_POWER_ON();

    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    SakuraIO_Init(&hi2c1);

    for(int i=0; i<100; i++){
        if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_Delay(10);
    }

    //Wait for sakura.io boot up
    for(int i=0;i<50;i++){
        if( HAL_GPIO_ReadPin(SAKURA_WAKE_OUT_GPIO_Port, SAKURA_WAKE_OUT_Pin) == GPIO_PIN_SET ){
            break;
        }
        HAL_Delay(100);
    }
    if( SakuraIO_Command_set_power_save_mode(0) != CMD_ERROR_NONE ){
        Error_Handler();
    }

    //Initialize LPS25HB
    if (BSP_PRESSURE_Init(LPS25HB_P_0, (void **)&LPS25HB_P_handle) == COMPONENT_ERROR || BSP_TEMPERATURE_Init(LPS25HB_T_0, (void **)&LPS25HB_T_handle) == COMPONENT_ERROR)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    //Enable LPS25HB
    if (BSP_PRESSURE_Sensor_Enable(LPS25HB_P_handle) == COMPONENT_ERROR || BSP_TEMPERATURE_Sensor_Enable(LPS25HB_T_handle) == COMPONENT_ERROR)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    LPS25HB_Set_AvgP(LPS25HB_P_handle, LPS25HB_AVGP_512);
    LPS25HB_Set_AvgT(LPS25HB_T_handle, LPS25HB_AVGT_64);
    LPS25HB_Set_Odr(LPS25HB_P_handle, LPS25HB_ODR_25HZ);

    GPS_POWER_ON();
    conio_init();

    while(unixtime == 0){
        parse_gps();

        float v = 0;
        for(int i=0;i<50;i++){
            v += get_voltage();
        }
        v /= 50;
        if( v < BATTERY_LOW_VOLTAGE ){
            return -1;
        }
    }

    RTC_AlarmConfig();
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);

    return 0;
}

void ciot_main(){
    uint32_t prev_cnt = rtc_cnt;
    uint8_t status_queue=CHANNEL_TRANSMIT_FREE;
    uint8_t status_immediate;


    CiotBuffer buff;
    CiotBuffer send_buff;
    uint8_t flag_transmit = 0;
    uint32_t prev_distance = 0;

    if( ciot_init() ){
        goto sleep;
    }

    __disable_irq();
    reed_cnt = 0;
    speed = 0;
    distance = 0;
    __enable_irq();

    for(;;){

        do{
            parse_gps();

            buff.battery_voltage = 0;
            for(int i=0;i<50;i++){
                buff.battery_voltage += get_voltage();
            }
            buff.battery_voltage /= 50;
            BSP_PRESSURE_Get_Press(LPS25HB_P_handle, (float *)&(buff.press) );
            BSP_TEMPERATURE_Get_Temp(LPS25HB_T_handle, (float *)&(buff.temp) );

            if( flag_transmit ){

                //To avoid the bug sakura.io module
                /* if( status_queue == CHANNEL_TRANSMIT_BUSY ){ */
                if( status_queue & CHANNEL_TRANSMIT_BUSY ){
                    //If previous state is BUSY(transmitting), update the state
                    SakuraIO_GetTxStatus(&status_queue, &status_immediate);
                }

                if( status_queue == CHANNEL_TRANSMIT_FREE ){
                    //If queue is not empty
                    if( ciot_dequeue(&send_buff) == 0 ){
                        SakuraIO_EnqueueUint32(0, send_buff.speed, 0);
                        SakuraIO_EnqueueDouble(1, send_buff.lat, 0);
                        SakuraIO_EnqueueDouble(2, send_buff.lng, 0);
                        SakuraIO_EnqueueFloat(5, send_buff.temp, 0);
                        SakuraIO_EnqueueFloat(6, send_buff.press, 0);
                        SakuraIO_EnqueueUint32(7, send_buff.distance, 0);

                        SakuraIO_EnqueueFloat(10, send_buff.battery_voltage, 0);

                        SakuraIO_EnqueueUint32(20, send_buff.status, 0);
                        SakuraIO_EnqueueUint32(21, send_buff.rtc_cnt, 0);

                        if( SakuraIO_Send() == CMD_ERROR_NONE){
                            status_queue = CHANNEL_TRANSMIT_BUSY;
                        }
                        else{
                            status_queue = CHANNEL_TRANSMIT_FAILED;
                        }
                    }
                    else{
                        flag_transmit = 0;
                    }
                }
                //If transmit failed, re-transmit
                else if( status_queue == CHANNEL_TRANSMIT_FAILED ){
                    if( SakuraIO_Send() == CMD_ERROR_NONE){
                        status_queue = CHANNEL_TRANSMIT_BUSY;
                    }
                    else{
                        status_queue = CHANNEL_TRANSMIT_FAILED;
                    }
                }
            }

            HAL_Delay(1);

        } while( rtc_cnt - prev_cnt < 5 );

        sleep:

        __disable_irq();
        buff.rtc_cnt = unixtime;
        prev_cnt = rtc_cnt;
        buff.speed = speed;
        buff.distance = distance;
        __enable_irq();

        //if( prev_distance != buff.distance ){
        if( 1 ){
            buff.lat = lat;
            buff.lng = lng;
            buff.status = 0x00;
            if( HAL_GPIO_ReadPin(V_USB_GPIO_Port, V_USB_Pin) == GPIO_PIN_SET ){
                buff.status |= 0x01;
            }
            if( gps_active ){
                buff.status |= 0x02;
            }

            ciot_enqueue(&buff);
        }
        prev_distance = buff.distance;

        if( buff.battery_voltage < BATTERY_LOW_VOLTAGE ){
            break;
        }

        if( ciot_get_queue_count() >= 12 ){
            flag_transmit = 1;
        }
    }

    UARTx->CR1 &= ~(USART_CR1_RXNEIE);
    GPS_POWER_OFF();

    if( SakuraIO_Command_set_power_save_mode(2) != CMD_ERROR_NONE ){
        Error_Handler();
    }
    SAKURAIO_POWER_OFF();

    if( LPS25HB_DeActivate(LPS25HB_P_handle) != LPS25HB_OK ){
        Error_Handler();
    }

    HAL_NVIC_DisableIRQ(RTC_IRQn);

    //TODO : Enter STOP Mode
    for(;;);
}

float get_voltage(){
    if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
    {
        Error_Handler();
    }
    /* Check if the continuous conversion of regular channel is finished */
    if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
        uint32_t v = HAL_ADC_GetValue(&hadc);
        return (float)v *(14.9/4.9*1.8/4096);
    }
    return 0;
}

int split(char *str, const char delim, char *token[], int max_item)
{
    int cnt = 0;
    int len = strlen(str);

    token[cnt++] = str;
    for (int i = 0; i < len; i++){
        if (str[i] == delim){
            str[i] = '\0';
            if (cnt == max_item)
                return cnt;
            token[cnt++] = str + i + 1;
        }
    }
    return cnt;
}

int is_nmea_valid(char *str, int len){
    uint8_t parity = 0x00;
    for(int i=1;i<len-3;i++){
        parity ^= (uint32_t)str[i];
    }

    if( strtol(str+len-2, 0, 16) == parity ){
        return 1;
    }
    return 0;
}

void parse_gps(){
    uint8_t c;
    while( (c = getch()) != false){
        if(gps_index<510){
            if(gps_index ==0){
                if( c == '$' ) gps_line[gps_index++] = c;
                else continue;
            }
            else gps_line[gps_index++] = c;
        }
        else gps_index =0;

        if(c=='\n'){
            gps_line[gps_index] = '\0';
            int len = gps_index;
            gps_index =0;
            if( len > 5 && gps_line[len-2] == '\r' && gps_line[len-5] == '*' ){
                gps_line[len-2] = '\0';
                len -= 2;

                if( gps_line[1] == 'G' && gps_line[2] == 'P' && gps_line[3] == 'R' && gps_line[4] == 'M' && gps_line[5] == 'C' ){
                    if( is_nmea_valid( gps_line,len) ){
                        char *tp[20];
                        split(gps_line, ',', tp, 20);

                        if( unixtime == 0 && strlen(tp[9]) == 6 ){
                            struct tm tm;
                            uint32_t time = atoi(tp[1]);
                            uint32_t date = atoi(tp[9]);
                            tm.tm_year = (date)%100 + 100;
                            tm.tm_mon = (date/100)%100 - 1;
                            tm.tm_mday = (date/10000)%100;
                            tm.tm_hour = (time/10000)%100;
                            tm.tm_min = (time/100)%100;
                            tm.tm_sec = (time)%100;
                            unixtime = mktime(&tm);
                            rtc_cnt = 0;
                        }

                        //if GPRMC status is Active
                        if(tp[2][0]=='A'){
                            //tp[3]: Latitude
                            //tp[5]: Lngitude
                            char *p = tp[3];
                            lat = ((int)p[0]-'0')*10 + ((int)p[1]-'0') + atof(p+2)/60;
                            p = tp[5];
                            lng = ((int)p[0]-'0')*100 + ((int)p[1]-'0')*10 + ((int)p[2]-'0') + atof(p+3)/60;

                            gps_active = true;
                        }
                        else{
                            gps_active = false;
                            lat = lng = 0.0;
                        }
                    }

                }
            }
        }
    }

}


/**
 * @brief  Configure the current time and date.
 * @param  None
 * @retval None
 */
static void RTC_AlarmConfig(void)
{
    RTC_DateTypeDef  sdatestructure;
    RTC_TimeTypeDef  stimestructure;
    RTC_AlarmTypeDef salarmstructure;

    /*##-1- Configure the Date #################################################*/
    /* Set Date: Tuesday February 18th 2014 */
    sdatestructure.Year = 0x14;
    sdatestructure.Month = RTC_MONTH_FEBRUARY;
    sdatestructure.Date = 0x18;
    sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

    if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Configure the Time #################################################*/
    /* Set Time: 00:00:00 */
    stimestructure.Hours = 0x00;
    stimestructure.Minutes = 0x00;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&hrtc,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-3- Configure the RTC Alarm peripheral #################################*/
    /* Set Alarm to every second (RTC_ALARMMASK_ALL) */
    salarmstructure.Alarm = RTC_ALARM_A;
    salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
    salarmstructure.AlarmMask = RTC_ALARMMASK_ALL;
    salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    salarmstructure.AlarmTime.Hours = 0x00;
    salarmstructure.AlarmTime.Minutes = 0x00;
    salarmstructure.AlarmTime.Seconds = 0x00;
    salarmstructure.AlarmTime.SubSeconds = 0x00;

    if(HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

/**
 * @brief  Alarm callback
 * @param  hrtc : RTC handle
 * @retval None
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    rtc_cnt++;
    if( unixtime > 0 ){
        unixtime++;
    }

    speed = reed_cnt;
    reed_cnt = 0;
    distance += speed;
}

/**
 * @brief EXTI line detection callback.
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == REED_SWITCH_Pin)
    {
        reed_cnt++;
    }
    else if(GPIO_Pin == V_USB_Pin)
    {

    }

}

