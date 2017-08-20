/*
 * ciot.c
 *
 *  Created on: 2017/08/20
 *      Author: spira
 */

#include "ciot.h"
#include "stm32l0xx_hal.h"
#include <ctype.h>
#include "uart_support.h"

#include <bsp_pressure.h>
#include <bsp_temperature.h>

static PRESSURE_Drv_t *LPS25HB_P_handle = NULL;
static TEMPERATURE_Drv_t *LPS25HB_T_handle = NULL;

char gps_line[512];
int gps_index =0;
uint8_t gps_active =false;

float lat=0.0f, lng=0.0f;

void ciot_init(){
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

    GPS_POWER_ON();
    conio_init();
}

void ciot_main(){
    for(;;){
        float press, temp;
        parse_gps();
        if( BSP_PRESSURE_Get_Press(LPS25HB_P_handle, (float *)&press) == COMPONENT_OK && BSP_TEMPERATURE_Get_Temp(LPS25HB_T_handle, (float *)&temp) == COMPONENT_OK ){

        }
        HAL_Delay(100);
    }
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
            gps_index =0;

            if( gps_line[1] == 'G' && gps_line[2] == 'P' && gps_line[3] == 'R' && gps_line[4] == 'M' && gps_line[5] == 'C' ){
                char *tp[20];
                int num_token = split(gps_line, ',', tp, 20);

                //if GPRMC status is Active
                if(num_token==13 && tp[2][0]=='A'){
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
                }
            }

        }
    }

}
