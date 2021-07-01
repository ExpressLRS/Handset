#pragma once

#include "user_config.h"

// which board are we using

// #define LONGAN_NANO
// #define T_DISPLAY
#define PCB_V1_0


#ifdef ELRS_OG_COMPATIBILITY
    #define Regulatory_Domain_ISM_2400
#else
    #define Regulatory_Domain_ISM_2400_NA
#endif


#ifdef RADIO_E28_12
// E28-12 and both GNICERF modules can use the full output range
#define MAX_PRE_PA_POWER 13
#define DISARM_POWER (0)
#elif defined(RADIO_E28_20)
#define MAX_PRE_PA_POWER (-2)
#define DISARM_POWER (-12)
#elif defined(RADIO_E28_27)
#define MAX_PRE_PA_POWER 0
#define DISARM_POWER (-15)
#endif


// how many switches do we have?
// hybrid 8 currently deals with up to 8 switches, but if we only have 4 fitted we can save some time by
// setting the lower value here
#define MAX_SWITCHES    4


// =====================

#ifdef LONGAN_NANO

// #define RADIO_BUSY_PORT GPIOA
// #define RADIO_BUSY_PIN  GPIO_PIN_11

// #define RADIO_RESET_PORT GPIOA
// #define RADIO_RESET_PIN GPIO_PIN_12

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11


// rotary encoder button
// #define RE_BUTTON_PORT GPIOA
// #define RE_BUTTON_PIN GPIO_PIN_4

#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13

// for testing on the nano
// #define SWB_TMP PA14

#define SWA_LOW  PC15
#define SWA_HIGH PC14

#define SWB_LOW  PA13
#define SWB_HIGH PA14

#define SWC_LOW PA15
#define SWC_HIGH PB3

#define SWD_LOW  PB5
#define SWD_HIGH PB4

// need adc scaling constants even if we don't have anything connected
#define ADC_PITCH_REVERSED false
#define ADC_PITCH_MIN 726u
#define ADC_PITCH_CTR 2080u
#define ADC_PITCH_MAX 3497u

#define ADC_ROLL_REVERSED true
#define ADC_ROLL_MIN 592u
#define ADC_ROLL_CTR 2083u
#define ADC_ROLL_MAX 3547u

#define ADC_THROTTLE_REVERSED true
#define ADC_THROTTLE_MIN 561u
#define ADC_THROTTLE_MAX 3504u

#define ADC_YAW_REVERSED false
#define ADC_YAW_MIN 741u
#define ADC_YAW_CTR 2081u
#define ADC_YAW_MAX 3436u


#elif defined(T_DISPLAY)

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

#define RADIO_RXEN_PORT GPIOB
#define RADIO_RXEN_PIN GPIO_PIN_11

#define RADIO_TXEN_PORT GPIOA
#define RADIO_TXEN_PIN GPIO_PIN_8

// rotary encoder button
#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13

// RC switches
#define SWA_LOW  PC15
#define SWA_HIGH PC14

// #define SWB_TMP PA13
#define SWB_LOW  PA13
#define SWB_HIGH PA14

#define SWC_LOW PA15
#define SWC_HIGH PB3

#define SWD_LOW  PB5
#define SWD_HIGH PB4

// Gimbal calibration values
#define ADC_PITCH_REVERSED true
#define ADC_PITCH_MIN 913u
#define ADC_PITCH_CTR 2404u
#define ADC_PITCH_MAX 3864u

#define ADC_ROLL_REVERSED false
#define ADC_ROLL_MIN 57u
#define ADC_ROLL_CTR 1892u
#define ADC_ROLL_MAX 3763u

#define ADC_THROTTLE_REVERSED false
#define ADC_THROTTLE_MIN 820u
#define ADC_THROTTLE_MAX 3860u

#define ADC_YAW_REVERSED true
#define ADC_YAW_MIN 343u
#define ADC_YAW_CTR 1838u
#define ADC_YAW_MAX 3435u

#elif defined(PCB_V1_0)

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

#define RADIO_RXEN_PORT GPIOB
#define RADIO_RXEN_PIN GPIO_PIN_11

#define RADIO_TXEN_PORT GPIOA
#define RADIO_TXEN_PIN GPIO_PIN_8

// rotary encoder button
#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13

// RC switches
#define SWA_LOW  PA14
#define SWA_HIGH PA13

#define SWB_LOW  PC14
#define SWB_HIGH PC15

#define SWC_LOW  PB3
#define SWC_HIGH PA15

#define SWD_LOW  PB5
#define SWD_HIGH PB4

// Buzzer
#define GPIO_BUZZER PA4




#else
#error "define the board type in config.h"
#endif

// common to all boards

#define RADIO_NSS_PORT GPIOB
#define RADIO_NSS_PIN GPIO_PIN_12

#define RADIO_DIO1_PORT GPIOB
#define RADIO_DIO1_PIN GPIO_PIN_8

#define RADIO_DIO2_PORT GPIOB
#define RADIO_DIO2_PIN GPIO_PIN_9

