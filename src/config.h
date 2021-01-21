
// which board are we using

// #define LONGAN_NANO
// #define T_DISPLAY
#define PCB_V1_0

// TODO make this runtime dynamic
// define the type of radio module being used 

#define RADIO_E28_12
// #define RADIO_E28_20
// #define RADIO_E28_27

#ifdef RADIO_E28_12
// E28-12 and both GNICERF modules can use the full output range
#define MAX_PRE_PA_POWER 13
#elif defined(RADIO_E28_20)
#define MAX_PRE_PA_POWER (-2)
#elif defined(RADIO_E28_27)
#define MAX_PRE_PA_POWER 0
#endif



// how many switches do we have?
// hybrid 8 currently deals with up to 8 switches, but if we only have 4 fitted we can save some time by
// setting the lower value here
#define MAX_SWITCHES    4


// definitions for simpler gpio naming scheme

enum ports {
    PORTA,
    PORTB,
    PORTC
};

#define PA1  (PORTA << 16 | GPIO_PIN_1)
#define PA2  (PORTA << 16 | GPIO_PIN_2)
#define PA3  (PORTA << 16 | GPIO_PIN_3)
#define PA4  (PORTA << 16 | GPIO_PIN_4)
#define PA5  (PORTA << 16 | GPIO_PIN_5)
#define PA6  (PORTA << 16 | GPIO_PIN_6)
#define PA7  (PORTA << 16 | GPIO_PIN_7)
#define PA8  (PORTA << 16 | GPIO_PIN_8)
#define PA9  (PORTA << 16 | GPIO_PIN_9)
#define PA10 (PORTA << 16 | GPIO_PIN_10)
#define PA11 (PORTA << 16 | GPIO_PIN_11)
#define PA12 (PORTA << 16 | GPIO_PIN_12)
#define PA13 (PORTA << 16 | GPIO_PIN_13)
#define PA14 (PORTA << 16 | GPIO_PIN_14)
#define PA15 (PORTA << 16 | GPIO_PIN_15)

#define PB1  (PORTB << 16 | GPIO_PIN_1)
#define PB2  (PORTB << 16 | GPIO_PIN_2)
#define PB3  (PORTB << 16 | GPIO_PIN_3)
#define PB4  (PORTB << 16 | GPIO_PIN_4)
#define PB5  (PORTB << 16 | GPIO_PIN_5)
#define PB6  (PORTB << 16 | GPIO_PIN_6)
#define PB7  (PORTB << 16 | GPIO_PIN_7)
#define PB8  (PORTB << 16 | GPIO_PIN_8)
#define PB9  (PORTB << 16 | GPIO_PIN_9)
#define PB10 (PORTB << 16 | GPIO_PIN_10)
#define PB11 (PORTB << 16 | GPIO_PIN_11)
#define PB12 (PORTB << 16 | GPIO_PIN_12)
#define PB13 (PORTB << 16 | GPIO_PIN_13)
#define PB14 (PORTB << 16 | GPIO_PIN_14)
#define PB15 (PORTB << 16 | GPIO_PIN_15)

#define PC1  ((PORTC << 16) | GPIO_PIN_1)
#define PC2  ((PORTC << 16) | GPIO_PIN_2)
#define PC3  ((PORTC << 16) | GPIO_PIN_3)
#define PC4  ((PORTC << 16) | GPIO_PIN_4)
#define PC5  ((PORTC << 16) | GPIO_PIN_5)
#define PC6  ((PORTC << 16) | GPIO_PIN_6)
#define PC7  ((PORTC << 16) | GPIO_PIN_7)
#define PC8  ((PORTC << 16) | GPIO_PIN_8)
#define PC9  ((PORTC << 16) | GPIO_PIN_9)
#define PC10 ((PORTC << 16) | GPIO_PIN_10)
#define PC11 ((PORTC << 16) | GPIO_PIN_11)
#define PC12 ((PORTC << 16) | GPIO_PIN_12)
#define PC13 ((PORTC << 16) | GPIO_PIN_13)
#define PC14 ((PORTC << 16) | GPIO_PIN_14)
#define PC15 ((PORTC << 16) | GPIO_PIN_15)


#define PIN(x) (x & 0xFFFF)

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
#define RE_BUTTON_PORT GPIOA
#define RE_BUTTON_PIN GPIO_PIN_4

// for testing on the nano
// #define SWB_TMP PA14

#define SWA_LOW  PC15
#define SWA_HIGH PC14

#define SWB_LOW  PA13
#define SWB_HIGH PA14   // PA14 is stuck low, need to add a pullup of 4k7 or a little more

#define SWC_LOW PA15
#define SWC_HIGH PB3

#define SWD_LOW  PB5
#define SWD_HIGH PB4


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

// Gimbal calibration values
#define ADC_PITCH_REVERSED false
#define ADC_PITCH_MIN 744u
#define ADC_PITCH_CTR 1262u
#define ADC_PITCH_MAX 1824u

#define ADC_ROLL_REVERSED true
#define ADC_ROLL_MIN 702u
#define ADC_ROLL_CTR 1207u
#define ADC_ROLL_MAX 1809u

#define ADC_THROTTLE_REVERSED true
#define ADC_THROTTLE_MIN 742u
#define ADC_THROTTLE_MAX 1906u

#define ADC_YAW_REVERSED false
#define ADC_YAW_MIN 586u
#define ADC_YAW_CTR 1220u
#define ADC_YAW_MAX 1890u

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


// Pins with LEDS are being used for other things.

// #define LED_PIN GPIO_PIN_13
// #define LED_GPIO_PORT GPIOC
// #define LED_GPIO_CLK RCU_GPIOC

// #define GREEN_LED GPIO_PIN_1
// #define BLUE_LED GPIO_PIN_2
