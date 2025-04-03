/*
 * @Author: frozen-fire 2812643217@qq.com
 * @Date: 2025-03-21 20:35:17
 * @LastEditors: frozen-fire 2812643217@qq.com
 * @LastEditTime: 2025-04-03 20:05:26
 * @FilePath: \PSP_supercapacitor\USER\Inc\dcdc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __DCDC__H
#define __DCDC__H

#include "adc.h"
#include "gpio.h"
#include "iwdg.h"
#include "hrtim.h"
#include "usart.h"
#include <stdint.h>

#define DCDC_PERIOD 15542
#define DCDC_COMPARE_MINVAL (700)

#define toCompareVal(p) ((int)((p/100.0f)*(float)DCDC_PERIOD))

/************SAFETY SETTINGS**********/
#define BUS_UVP_THRE 18.0f
#define BUS_OVP_THRE 28.5f
#define BAT_OVP_THRE 30.0f
#define BAT_FULL_VOL 26.0f
#define BAT_UVP_STARTUP_THRE 10.0f

#define PROTECTION_RECOVERY_TIME 2000

#define CAP_MAX_CURRENT 13.8f

#define POWER_LIMIT_MINIMUM 15.0f
#define POWER_LIMIT_MAXIMUM 350.0f

#define ONTIME_FILTERSTABLE_DELAY 5
/*************************************/

#define IIR_V 0.1f
#define IIR_C 0.2f
#define INA181_REF 1.25f
#define IMOTOR_CAL 0.89f
#define ITOT_CAL 0.98f

#define V_REF (3.00f)

#define DT (1.0f/350000.0f)

typedef struct pwr_adc_t{
    uint16_t i_motor;   //ADC1_IN3 (PA2)
    uint16_t v_bus;     //ADC1_IN4 (PA3)
    uint16_t i_dcdc;    //ADC2_IN13(PA5)
    uint16_t i_tot;     //ADC2_IN5 (PC4)
    uint16_t v_cap;     //ADC2_IN12(PB2)
    uint16_t i_fw2;     //ADC3_IN12(PB0)
    uint16_t i_fw1;     //ADC3_IN1 (PB1)
}pwr_adc_t;

#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7f}
typedef struct pwr_data_t
{
    float v_bus;    //总线电压
    float v_cap;    //电容电压
    float i_tot;    //总线输入电流
    float i_dcdc;   //DCDC电流
    float i_motor;  //负载电流
    float i_allow;  //允许的电流
    float state;   //工作状态
    unsigned char tail[4];
}pwr_data_t;

typedef struct pid_t
{
    const float p;
    const float i;
    const float d;
    const float i_max;
    float errm1;
    float err_i;
}PID_t;

enum Cap_states{
    CAP_OFF,
    CAP_READY,
    CAP_ON,
    VBUS_OVP,
    VBUS_UVP,
    VBAT_OVP,
};

void dcdc_init(void);
void dcdc_setphase(float percentage);
void dcdc_setduty(float duty);

void dcdc_update_power_limit(float limit);

void dcdc_mainISR(void);




#endif