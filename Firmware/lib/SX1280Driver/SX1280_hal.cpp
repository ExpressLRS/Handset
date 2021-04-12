/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project 
*/

#include "../../src/config.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include <stdio.h>
// #include <SPI.h>
#include "../../src/utils.h"
#include <string.h> // for memcpy

extern "C" {
#include "../../include/systick.h"
}

SX1280Hal *SX1280Hal::instance = nullptr;

// void  SX1280Hal::nullCallback(void){};

// void (*SX1280Hal::TXdoneCallback)() = &nullCallback;
// void (*SX1280Hal::RXdoneCallback)() = &nullCallback;

SX1280Hal::SX1280Hal()
{
    instance = this;
}

void SX1280Hal::end()
{
    // XXX todo
    // SPI.end(); 
    // detachInterrupt(GPIO_PIN_DIO1);
}

void SX1280Hal::init()
{
    // all pin/spi setup done in main.cpp
}

void  SX1280Hal::reset(void)
{
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);
    gpio_bit_reset(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(100);
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);

    if (WaitOnBusy()) {
        printf("WARNING SX1280 busy didn't go low\n\r");
    } else {
        printf("SX1280 Ready!\n\r");
    }
}

void SX1280Hal::WriteCommand(const SX1280_RadioCommands_t command, const uint8_t val)
{
    uint8_t buffer[2] = {command, val};

    WaitOnBusy();

    spi1_transferBytes(buffer, 2);
}

// TODO add a fastWrite command that just takes a buffer and size
void SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 1];

    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, size+1);
}

/** faster version of Writecommand.
 * The command is passed in the first byte of buffer
 * size includes the command
 * contents of buffer will be overwritten
*/
void SX1280Hal::fastWriteCommand(uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();

    spi1_transferBytes(buffer, size);
}


// TODO add fast read without the memory copying
void SX1280Hal::ReadCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 2];

    WaitOnBusy();

    if (command == SX1280_RADIO_GET_STATUS)
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        OutBuffer[2] = 0x00;
        spi1_transferBytes(OutBuffer, 3);
        buffer[0] = OutBuffer[0];
    }
    else
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        memcpy(OutBuffer + 2, buffer, size);
        spi1_transferBytes(OutBuffer, sizeof(OutBuffer));
        memcpy(buffer, OutBuffer + 2, size);
    }
}

void  SX1280Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1280_RADIO_WRITE_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0x00FF;

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy();
    // digitalWrite(GPIO_PIN_NSS, LOW);
    spi1_transferBytes(OutBuffer, (uint8_t)sizeof(OutBuffer));
    // digitalWrite(GPIO_PIN_NSS, HIGH);
}

void  SX1280Hal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

void  SX1280Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 4];

    OutBuffer[0] = SX1280_RADIO_READ_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0x00FF;
    OutBuffer[3] = 0x00;

    memcpy(OutBuffer + 4, buffer, size);

    WaitOnBusy();
    // digitalWrite(GPIO_PIN_NSS, LOW);

    spi1_transferBytes(OutBuffer, uint8_t(sizeof(OutBuffer)));
    memcpy(buffer, OutBuffer + 4, size);

    // digitalWrite(GPIO_PIN_NSS, HIGH);
}

uint8_t  SX1280Hal::ReadRegister(uint16_t address)
{
    uint8_t data=0;
    ReadRegister(address, &data, 1);
    return data;
}

void  SX1280Hal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    // uint8_t localbuf[size];

    // for (int i = 0; i < size; i++) // todo check if this is the right want to handle volatiles
    // {
    //     localbuf[i] = buffer[i];
    // }

    uint8_t OutBuffer[size + 2];

    OutBuffer[0] = SX1280_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;

    memcpy(OutBuffer + 2, (void *)buffer, size);

    WaitOnBusy();

    // digitalWrite(GPIO_PIN_NSS, LOW);
    spi1_transferBytes(OutBuffer, (uint8_t)sizeof(OutBuffer));
    // digitalWrite(GPIO_PIN_NSS, HIGH);
}

void  SX1280Hal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1280_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;

    memset(OutBuffer + 3, 0, size); // XXX is this needed?

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, uint8_t(sizeof(OutBuffer)));

    memcpy((void *)buffer, OutBuffer + 3, size);
}

/** Wait for the SX1280 busy flag to be low
 * Returns true if we reach the timeout before busy goes low
 * TODO pass in the timeout
 */
bool  SX1280Hal::WaitOnBusy()
{
    // printf("%s \r\n", "waitOnBusy...");
    const uint MAX_WAIT = 1000; // in us
    const unsigned long t0 = micros();
    while (gpio_input_bit_get(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == SET)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout \n");
            return true;
        }
    }
    // printf("waitOnBusy done in %lu us\n", micros()-t0);
    return false;
}

// void  SX1280Hal::dioISR()
// {
//     if (instance->InterruptAssignment == SX1280_INTERRUPT_RX_DONE)
//     {
//         //Serial.println("HalRXdone");
//         RXdoneCallback();
//     }
//     else if (instance->InterruptAssignment == SX1280_INTERRUPT_TX_DONE)
//     {
//         //Serial.println("HalTXdone");
//         TXdoneCallback();
//     }
// }

void  SX1280Hal::TXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_set(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1280Hal::RXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_set(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1280Hal::TXRXdisable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

// void  SX1280Hal::setIRQassignment(SX1280_InterruptAssignment_ newInterruptAssignment)
// {

//     // if (InterruptAssignment == newInterruptAssignment)
//     // {
//     //     return;
//     // }
//     // else
//     // {
//     if (newInterruptAssignment == SX1280_INTERRUPT_TX_DONE)
//     {
//         this->InterruptAssignment = SX1280_INTERRUPT_TX_DONE;
//     }
//     else if (newInterruptAssignment == SX1280_INTERRUPT_RX_DONE)
//     {
//         this->InterruptAssignment = SX1280_INTERRUPT_RX_DONE;
//     }
//     //}
// }