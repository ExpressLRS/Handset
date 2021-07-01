#pragma once

// constants for the supported compatibility levels
#define COMPAT_LEVEL_1_0_0_RC3 (2)  // RC2 and RC3 are equivalent
#define COMPAT_LEVEL_1_0_0_RC2 (2)
#define COMPAT_LEVEL_DEV_16fbd1d011d060f56dcc9b3a33d9eead819cf440 (1)


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
