
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "utils.h"
#include "SX1280.h"
#include "SX1280_hal.h" //  only needed for MAX_PRE_PA_POWER
#include "FHSS.h"
#include "SimpleStore.h"

// Next:
//    UI improvements
//    Better stick calibration

//    msp for vtx channel?
   // Info from BF msp.c:
   // uint16_t newFrequency = sbufReadU16(src);
   // if (newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL) {  // Value is band and channel
   //     const uint8_t newBand = (newFrequency / 8) + 1;
   //     const uint8_t newChannel = (newFrequency % 8) + 1;
//

// Prevents some debug print statements when uncommented
#define DEBUG_SUPPRESS

// Uncomment to enable some debug output on the LCD
#define DEBUG_STATUS

extern "C" {

#include "gd32vf103.h"
#include "systick.h"

#ifdef LONGAN_NANO
#include "lcd/lcd.h"
#elif defined(T_DISPLAY) || defined(PCB_V1_0)
#include "lcd-tdisplay/lcd.h"
#else
#error "Define the board type before compiling"
#endif

} // extern "C"

// find a better place for these
#define RC_DATA_PACKET  0b00
#define MSP_DATA_PACKET 0b01
#define SYNC_PACKET     0b10
#define TLM_PACKET      0b11

#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14

#define LCD_PWM_MAX     124
#define LCD_PWM_CHARGING  5
#define LCD_PWM_ARMED    20

enum SWITCH_POSITIONS
{
   SWITCH_LOW,
   SWITCH_MID,
   SWITCH_HIGH
};

// params
enum EDITABLE_PARAMS {
   PARAM_NONE,    // must be first
   PARAM_POWER,
   PARAM_RATE,
   PARAM_TLM_INT,
   PARAM_FILTER_MODE,
   PARAM_SYNC_WHEN_ARMED,
   PARAM_N_PARAMS // must be last
};

uint8_t paramToChange = PARAM_NONE;

bool syncWhenArmed = true; // need this on for testing with no switches attached

// current and sent switch values, used for prioritising sequential switch transmission
uint8_t currentSwitches[MAX_SWITCHES] = {SWITCH_LOW};
uint8_t sentSwitches[MAX_SWITCHES] = {SWITCH_LOW};

uint8_t nextSwitchIndex = 0; // for round-robin sequential switches


// Methods that need to be callable from the C interrupt handlers
extern "C" {
   void SendRCdataToRF();
   void HandleTLM();
   void HandleFHSS();
   void ProcessTLMpacket();
   void SetRFLinkRate(uint8_t rate);
}

// debug baud rate
#define BAUDRATE (460800U)

// low battery warning threshold in tenths of a volt
#define LOW_BAT_THRESHOLD (34)

bool batteryLow = false;

unsigned long interval, adcInterval;
unsigned int nSamples=0;
char debugStr[64];
bool debugReady = false;

// static bool led=0;
unsigned long lastButtonTime = 0;
bool buttonMoved = false;
bool doTX = false;

// storage for DMA ouput from ADC
#define DMA_BUFFER_LEN (4)          // maybe double buffer?
uint16_t adc_value[DMA_BUFFER_LEN];

// Storage format for persistent params
struct persistentData_s {
   int8_t txPower;      // signed so that we don't have to add an offset when saving
   uint8_t airmodeIndex;
   uint8_t telemRatio;
   uint8_t sync;
   uint8_t filterMode;
};

typedef persistentData_s persistentData_t;

#define CONFIG_STORE_ID 1


// filters

#include "1AUDfilterInt.h"

struct filterSpec_s {
   uint16_t minCutoff;
   uint16_t maxCutoff;
   float beta;
};

filterSpec_s raceFilter =      {200, 500, 0.01f};
filterSpec_s freestyleFilter = {100, 250, 0.01f};
filterSpec_s cinematicFilter = {10,   50, 0.01f};

enum FILTER_MODE {
   FILTER_RACE,
   FILTER_FREESTYLE,
   FILTER_CINEMATIC,
   FILTER_NUMBER_MODES
};

uint8_t currentFilterMode = FILTER_FREESTYLE;
filterSpec_s currentFilter = freestyleFilter;

// float minCutoff = 150;  // Hz
// float maxCutoff = 500;  // Hz
// float beta = 0.01f;  // did changing to .02 cause the bumps on the stick bounce?

// slewLimit is now steps per second and gets auto converted when sample rate changes
// NB too high a slew limit will overflow the fixed point math in the filter and cause unstable results
float slewLimit = 150000.0f;  // 10000 limits rate on stick bounce test, 20000 doesn't
// float slewLimit = 0;

// This is actually the rate at which the filters are updated which can be (and is)
// different to the rate the ADCs are running at when using oversampling.
float sampleRate = 8000;    // Hz   XXX keep this in step with the ADC setup

float dCutoff = 50.0f;      // Hz

// OneAUDfilter aud_rollF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_pitchF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_yawF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_throttleF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);

// last param is a guess at the initial value
OneAUDfilterInt aud_roll(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta), sampleRate, dCutoff, slewLimit, 2000);
OneAUDfilterInt aud_pitch(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta), sampleRate, dCutoff, slewLimit, 2000);
OneAUDfilterInt aud_yaw(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta), sampleRate, dCutoff, slewLimit, 2000);
OneAUDfilterInt aud_throttle(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta), sampleRate, dCutoff, slewLimit, 300);

// power values in prePA dBm (i.e. what the sx1280 is outputting to the PA or antenna if no PA present)
// int radioPower = -18; // low power testing to check crc/fec behaviour using e28-12
int radioPower = -10; // default for e28-27 (50mW)
// int radioPower = -8; // safe for any module
// int radioPower = -2; // dBm. Max for e28-20
// int radioPower = 0; // dBm. Max for e28-27
// int radioPower = 3; // around 100mW on gnice-27
// int radioPower = 12; // dBm. Testing with e28-12
// gnice-27 max is 13


#define RX_CONNECTION_LOST_TIMEOUT 3000 // Time in ms of no TLM response to consider that we have lost connection

bool isRXconnected = false;

volatile uint8_t NonceTX = 0;

uint32_t SyncPacketLastSent = 0;
uint32_t LastTLMpacketRecvMillis = 0;

SX1280Driver radio;

// telemetry values
int8_t rssi;
uint8_t snr, lq;

// For deferred packet rate change
int8_t nextLinkRate = -1;
uint8_t syncSpamCounter = 0;
uint32_t lastLinkRateChangeTime = 0;

bool lcdRedrawNeeded = false;


#include "crc.h"

#if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
#elif defined(ELRS_OG_COMPATIBILITY)
GENERIC_CRC8 ota_crc(ELRS_CRC_POLY);
#endif



// ======== Interrupt Handlers ============

// Handlers have to be in the C namespace
extern "C" {

// callled at the packet interval to trigger sending rf packets
void TIMER2_IRQHandler(void)
{
   static unsigned long tLast = 0;

   if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_CH0))
   {
      unsigned long now = micros();

      // clear interrupt bit or mcu will busy hang
      timer_interrupt_flag_clear(TIMER2, TIMER_INT_CH0);
      interval = tLast == 0 ? 0 : now - tLast;
      tLast = now;

      NonceTX++; // New - just increment the nonce on every frame

      #ifdef ELRS_OG_COMPATIBILITY

      HandleFHSS(); // experimental - can we do fhss here?

      // is there a deferred packet rate change?
      if (nextLinkRate != -1 && syncSpamCounter == 0) {
         // printf("nextLinkRate set %d\n\r", nextLinkRate);
         SetRFLinkRate(nextLinkRate);
         nextLinkRate = -1;
         lcdRedrawNeeded = true;
      }

      #else
      // fhss is in txdone
      #endif // ELRS_OG_COMPATIBILITY

      SendRCdataToRF();
   }
}

/**
 *    This is where we get notification that there is ADC data ready to be processed
 *    with the 1AUD filter.
 */
unsigned long filterUpdateTime = 0; // for debugging the cpu usage of the filters
void DMA0_Channel0_IRQHandler(void)
{
   static unsigned long last = 0;

   if (SET == dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF))
   {
      unsigned long now = micros();
      adcInterval = now - last;
      last = now;

      // clear interrupt bit or mcu will busy hang
      dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);

      #ifndef ADC_A_CH
      #error "need to define the ADC channel mapping"
      #endif

      aud_roll.update(adc_value[ADC_A_CH]);
      aud_pitch.update(adc_value[ADC_E_CH]);
      aud_throttle.update(adc_value[ADC_T_CH]);
      aud_yaw.update(adc_value[ADC_R_CH]);

      nSamples++;
      now = micros();
      filterUpdateTime += (now - last);
   }   
}

// interrupt handler for rotary encoder push button
void EXTI10_15_IRQHandler(void)
{
   if (RESET != exti_interrupt_flag_get(EXTI_13)){
      exti_interrupt_flag_clear(EXTI_13);
      // if(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_13)) XXX event loop needs to know about both up and down events
      {
         buttonMoved = true;
         lastButtonTime = millis();
      }
   }
}


void HandleFHSS()
{
   #ifdef ELRS_OG_COMPATIBILITY
   uint8_t modresult = (NonceTX+1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
   #else
   uint8_t modresult = (NonceTX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
   #endif // ELRS_OG_COMPATIBILITY


   if (modresult == 0) // if it time to hop, do so.
   {
      uint32_t f = FHSSgetNextFreq();
      // printf("f %lu\n\r", f);
      radio.SetFrequency(f);
   }
}

void TXdoneISR()
{
   // printf("TXdone!\n\r");

   // TODO do fhss from timer for both cases
   #ifdef ELRS_OG_COMPATIBILITY
   // fhss moved to timer
   #else
   HandleFHSS();
   #endif

   HandleTLM();
}


// use this for rx and tx done isrs on B8 and B9 (connected to radio DIOs)
void EXTI5_9_IRQHandler(void)
{
   if (exti_interrupt_flag_get(EXTI_8) == SET) {
      exti_interrupt_flag_clear(EXTI_8);
      // printf("RXdone!\n\r");

      // radio.ClearIrqStatus(SX1280_IRQ_RADIO_ALL);

      // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // rx has finished, so clear the debug pin

      // TODO move to the event loop?
      radio.readRXData();  // get the data from the sx1280 chip
      ProcessTLMpacket();

   } else if (exti_interrupt_flag_get(EXTI_9) == SET) {
      exti_interrupt_flag_clear(EXTI_9);
      // printf("TXdone!\n\r");

      // radio.ClearIrqStatus(SX1280_IRQ_RADIO_ALL);

      TXdoneISR();
   }
}

// something like this for switches - or maybe just poll them
// void EXTI10_15_IRQHandler(void)
// {

// }

} // extern "C"

// ==================================================

/** extract the gpio port value from a composite port/pin value
 * 
 */
uint32_t PORT(uint32_t compositeGPIOid) {
    uint32_t p = compositeGPIOid >> 16;
    switch (p) {
        case PORTA:
            return GPIOA;
        case PORTB:
            return GPIOB;
        case PORTC:
            return GPIOC;
        default:
            printf("XXX bad port id %lu\n\r", p);
            while(true);
    }

    // can't get here, but suppresses compiler warning
    return 0;
}



/** Sound the beeper
 * NB blocking for duration ms! Will need an async beeper function
 * @param duration time in ms to beep for
 * 
 */
void beep(uint32_t duration)
{
   #ifdef GPIO_BUZZER
   gpio_bit_set(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
   delay(duration);
   gpio_bit_reset(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
   #endif
}


/** Handle sequences of beeps
 * 
 *  Remember when the current/last beep started,
 *    if now is after beep start + duration, stop the beep
 * 
 *    if now is after beep start + interval, start a new beep
 * 
 * @param interval the delay between beeps in ms
 * @param duration the length of each beep in ms
 * @param currentTime approximate current time in ms
 * 
 * @return true if beeping, false if quiet. Is it useful?
 * 
 * duration should be less than interval. duration of 0 disables beeping
 * 
 * Passes the current time as a param as the call to millis() is quite expensive
 * and we already have a recent value in the caller.
 * 
 */
bool handleBeeps(uint16_t interval, uint16_t duration, uint32_t currentTime)
{
   #ifdef GPIO_BUZZER
   // Static variables to track state across calls
   static uint32_t beepStartTime = 0;
   static bool beeperActive = false;

   if (duration == 0) {
      // make sure the beeper is off
      if (beeperActive) {
         gpio_bit_reset(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
         beeperActive = false;
      }
      return false;
   }

   // uint32_t now = millis();

   if (currentTime > (beepStartTime + interval)) {
      if (!beeperActive) {
         // start the beeper
         beepStartTime = currentTime;
         gpio_bit_set(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
         beeperActive = true;
      }
      return true;
   }

   if (currentTime > (beepStartTime + duration)) {
      if (beeperActive) {
         // stop the beeper
         gpio_bit_reset(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
         beeperActive = false;
      }
      return false;
   }
   #endif // GPIO_BUZZER

   return false;
}


/**
 * Test for our view of if the drone is armed. (armed as in enabled/ready to fly!)
 * Requires that the first switch is the arm switch.
 * Conservatively assume that either middle or high position means armed.
 */
bool isArmed()
{
   return currentSwitches[0] != SWITCH_LOW;
}

/**
 * Determine which switch to send next.
 * If any switch has changed since last sent, we send the lowest index changed switch
 * and set nextSwitchIndex to that value + 1.
 * If no switches have changed then we send nextSwitchIndex and increment the value.
 * Switch 0 is sent with every packet and the rest of the switches
 * are in the round-robin.
 */
uint8_t getNextSwitchIndex()
{
   int firstSwitch = 1; // skip 0 since it is sent on every packet

   // look for a changed switch
   int i;
   for (i = firstSwitch; i < MAX_SWITCHES; i++)
   {
      if (currentSwitches[i] != sentSwitches[i])
         break;
   }
   // if we didn't find a changed switch, we get here with i==N_SWITCHES
   if (i == MAX_SWITCHES)
   {
      i = nextSwitchIndex;
   }

   // keep track of which switch to send next if there are no changed switches
   // during the next call.
   nextSwitchIndex = (i + 1) % MAX_SWITCHES;

   // for hydrid switches 0 is sent on every packet, so we can skip
   // that value for the round-robin
   if (nextSwitchIndex == 0)
   {
      nextSwitchIndex = 1;
   }

   return i;
}

/**
 * Record the value of a switch that was sent to the rx
 */
void setSentSwitch(uint8_t index, uint8_t value)
{
   if (index<MAX_SWITCHES) {
      sentSwitches[index] = value;
   } else {
      printf("XXX setSentSwitch: bad index\n\r");
   }
}


void ProcessTLMpacket()
{
   #ifndef DEBUG_SUPPRESS
   printf("TLMpacket\n\r");
   #endif

   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
   uint16_t inCRC = radio.RXdataBuffer[0] & 0b11111100;
   inCRC = (inCRC << 6) + radio.RXdataBuffer[7];
   radio.RXdataBuffer[0] &= 0b11;   // remove the OTA crc bits before calculating the crc
   uint16_t calculatedCRC = ota_crc.calc(radio.RXdataBuffer, 7);
   #elif (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_DEV_16fbd1d011d060f56dcc9b3a33d9eead819cf440)
   uint8_t calculatedCRC = ota_crc.calc(radio.RXdataBuffer, 7) + CRCCaesarCipher;
   uint8_t inCRC = radio.RXdataBuffer[7];
   #else // not ELRS_OG_COMPATIBILITY
   uint8_t calculatedCRC = CalcCRC(radio.RXdataBuffer, 7) + CRCCaesarCipher;
   uint8_t inCRC = radio.RXdataBuffer[7];
   #endif // ELRS_OG_COMPATIBILITY

   if ((inCRC != calculatedCRC))
   {
      #ifndef DEBUG_SUPPRESS
      printf("TLM sw crc!\n\r");
      #endif
      return;
   }

   uint8_t type = radio.RXdataBuffer[0] & TLM_PACKET;
   uint8_t TLMheader = radio.RXdataBuffer[1];

   #if (ELRS_OG_COMPATIBILITY != COMPAT_LEVEL_1_0_0_RC2) // no packetAddr from V1 onwards
   uint8_t packetAddr = (radio.RXdataBuffer[0] & 0b11111100) >> 2;
   if (packetAddr != DeviceAddr)
   {
      #ifndef DEBUG_SUPPRESS
      printf("TLM device address error\n\r");
      #endif
      return;
   }
   #endif

   //   packetCounteRX_TX++;

   if (type != TLM_PACKET)
   {
      #ifndef DEBUG_SUPPRESS
      printf("TLM type error %u\n\r", type);
      #endif
      return;
   }

   isRXconnected = true;
   LastTLMpacketRecvMillis = millis();

   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_DEV_16fbd1d011d060f56dcc9b3a33d9eead819cf440 || ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
   if (TLMheader == 1) // they changed the header value for new telem support
   #else
   if (TLMheader == CRSF_FRAMETYPE_LINK_STATISTICS)
   #endif
   {
      rssi = radio.RXdataBuffer[2];
      snr = radio.RXdataBuffer[4];
      lq = radio.RXdataBuffer[5];
      // printf("T: %d %u %u\n\r", rssi, snr, lq);
   } else {
      #ifndef DEBUG_SUPPRESS
      printf("TLMheader wrong %u\n\r", TLMheader);
      #endif
   }
}

bool nextFrameIsTelemetry()
{
   #ifdef ELRS_OG_COMPATIBILITY
   uint8_t modresult = (NonceTX+2) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
   #else
   // use nonce+1 because we're getting ready for the next frame
   uint8_t modresult = (NonceTX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
   #endif // ELRS_OG_COMPATIBILITY

   return modresult == 0;
}

bool thisFrameIsTelemetry()
{
   #ifdef ELRS_OG_COMPATIBILITY
   uint8_t modresult = (NonceTX+1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
   #else
   uint8_t modresult = (NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
   #endif // ELRS_OG_COMPATIBILITY

   return modresult == 0;
}


void HandleTLM()
{
   if (ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
   {
      if (nextFrameIsTelemetry()) {
         // receive tlm response because it's time
         //  printf("starting rx\n\r");
         // gpio_bit_set(LED_GPIO_PORT, LED_PIN); // for debugging, set the led pin high when listening
         radio.RXnb();
         // WaitRXresponse = true;
      }
   }
}

// freq in Hz
void setTimerISRFrequency(uint32_t freq)
{
   // counter runs at 100000 ticks/s
   uint16_t period = (100000 / freq) - 1;
   timer_autoreload_value_config(TIMER2, period);
}

// interval in us
void setTimerISRInterval(uint32_t interval)
{
   // counter runs at 100000 ticks/s, or 0.1 ticks per us
   uint16_t period = (interval / 10) - 1;
   timer_autoreload_value_config(TIMER2, period);
}

void GenerateSyncPacketData()
{
   uint8_t PacketHeaderAddr;
   PacketHeaderAddr = (DeviceAddr << 2) + SYNC_PACKET; // addr isn't used in compatibility mode from V1, but will get overwritten by the crc14 anyway
   radio.TXdataBuffer[0] = PacketHeaderAddr;
   uint8_t fhssIndex = FHSSgetCurrIndex();
   // printf("fhss %u\n\r", fhssIndex);

   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
   radio.TXdataBuffer[1] = fhssIndex+1;   // V1 shifted the fhss index by 1
   #else
   radio.TXdataBuffer[1] = fhssIndex;
   #endif

   #ifdef ELRS_OG_COMPATIBILITY
   radio.TXdataBuffer[2] = NonceTX+1;
   #else
   radio.TXdataBuffer[2] = NonceTX;
   #endif //ELRS_OG_COMPATIBILITY

   #ifdef ELRS_OG_COMPATIBILITY

   uint8_t SwitchEncMode = 0b01; // handset only supports hybrid8, so this is the only possible value
   uint8_t Index;
   if (syncSpamCounter != 0 && nextLinkRate != -1) {
      // Notify the RX that we're going to change packet rate
      Index = (nextLinkRate & 0b11);
      syncSpamCounter--;
   } else {
      // Send the current packet rate
      Index = (ExpressLRS_currAirRate_Modparams->index & 0b11);
   }
   uint8_t TLMrate = (ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111);
   radio.TXdataBuffer[3] = (Index << 6) + (TLMrate << 3) + (SwitchEncMode << 1);

   #else // not ELRS_OG_COMPATIBILITY
   radio.TXdataBuffer[3] = ((ExpressLRS_currAirRate_Modparams->index & 0b111) << 5) + ((ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111) << 2);
   #endif // ELRS_OG_COMPATIBILITY

   radio.TXdataBuffer[4] = UID[3];
   radio.TXdataBuffer[5] = UID[4];
   radio.TXdataBuffer[6] = UID[5];
}

// ===============================================
// config routines

// turn on the systems that are going to be used
// NB LCD does it's own hardware setup
void clock_config()
{
   // enable the GPIO clocks
   rcu_periph_clock_enable(RCU_GPIOA);
   rcu_periph_clock_enable(RCU_GPIOB);
   rcu_periph_clock_enable(RCU_GPIOC);

   // for alternate function outputs
   rcu_periph_clock_enable(RCU_AF);

   // enable the timers we're going to use
   rcu_periph_clock_enable(RCU_TIMER1);
   rcu_periph_clock_enable(RCU_TIMER2);
   rcu_periph_clock_enable(RCU_TIMER3);

   // clocks for gimbal ADC - using GPIOA pins, already enabled above
   rcu_periph_clock_enable(RCU_ADC0);
   rcu_periph_clock_enable(RCU_DMA0);
   rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8); // 13.5 MHz?

   // ADC for battery voltage monitoring
   rcu_periph_clock_enable(RCU_ADC1);

   // enable USART clock
   rcu_periph_clock_enable(RCU_USART0);

   // enable SPI1
	rcu_periph_clock_enable(RCU_SPI1);
}

/**
   timer1 channel1 can be used to trigger the adc
          TODO ch2 for backlight pwm on t-display?
   timer2 channel0 for radio TX interval
   timer3 ch0 & 1 for rotary encoder quadrature decoding

   NB If you add a timer, make sure you turn it on in clock_config!
  */
void timer_config(void)
{
   timer_oc_parameter_struct timer_ocinitpara;
   timer_parameter_struct timer_initpara;

   // config for timer1 - ADC trigger source
 
   timer_deinit(TIMER1);
   timer_struct_para_init(&timer_initpara);

   timer_initpara.prescaler = 107;   // 1 MHz clock
   timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
   timer_initpara.counterdirection = TIMER_COUNTER_UP;
   // NB if the period is changed, remember to update the 1AUD filter samplerate
   // timer_initpara.period = 99;    // 10kHz period
   timer_initpara.period = 124;    // 8kHz period for 16x oversampling
   timer_initpara.clockdivision = TIMER_CKDIV_DIV1;   // XXX what does this do?
   timer_initpara.repetitioncounter = 0;
   timer_init(TIMER1, &timer_initpara);

   // This is required to make the interrupts work even though we're not connected to a physical pin
   timer_channel_output_struct_para_init(&timer_ocinitpara);
   timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
   timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
   timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
   timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
   timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
   timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
   timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocinitpara);

   timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 49);
   timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
   timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

   // CH2 for LCD backlight
   timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocinitpara);
   timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 124); // max brightness
   timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM1); // mode 1 for longer pulse values -> brigher light
   timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);


   timer_auto_reload_shadow_enable(TIMER1);
   timer_enable(TIMER1);

   // config for timer2
   // Generate interrupts to trigger radio TX

   timer_struct_para_init(&timer_initpara);

   timer_initpara.prescaler = 1079;   // 100 kHz clock
   timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
   timer_initpara.counterdirection = TIMER_COUNTER_UP;
   timer_initpara.period = 99;    // 1kHz period (will get overridden with radio packet interval)
   timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
   timer_initpara.repetitioncounter = 0;
   timer_init(TIMER2, &timer_initpara);

   timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_TIMING);
   timer_channel_output_shadow_config(TIMER2, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

   timer_auto_reload_shadow_enable(TIMER2);
   timer_interrupt_enable(TIMER2, TIMER_INT_CH0);

   // rotary encoder - quadrature decoding
   timer_struct_para_init(&timer_initpara);
   // any initpara need to be set?
   timer_init(TIMER3, &timer_initpara);
   timer_quadrature_decoder_mode_config(TIMER3, TIMER_ENCODER_MODE1,  TIMER_IC_POLARITY_FALLING,  TIMER_IC_POLARITY_FALLING);

   uint16_t initialValue = 100 + radioPower + 18; // TODO check if this is meaningful, I think it gets reset when we enter edit mode

   timer_counter_value_config(TIMER3, initialValue);
   timer_enable(TIMER3);
}


/**
 *    DMA0 for ADC0
 */
void dma_config(void)
{
   dma_parameter_struct dma_data_parameter;
   
   dma_deinit(DMA0, DMA_CH0);
   
   // initialize DMA single data mode
   dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
   dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
   dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
   dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
   dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
   dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
   dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
   dma_data_parameter.number       = DMA_BUFFER_LEN;
   dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
   dma_init(DMA0, DMA_CH0, &dma_data_parameter);
   dma_circulation_enable(DMA0, DMA_CH0);

   dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_FTF);

   // enable DMA channel
   dma_channel_enable(DMA0, DMA_CH0);
}

/**
 *    Setup ADCs
 *    ADC0 for gimbals with DMA
 *    ADC1 for battery monitor with simple software triggering from the event loop
 *    Pins are a problem, there are only 3 adc pins that don't conflict with other hardware.
 *    Currently using 0-3, with 1 and 2 conflicted by leds. Depopulate the led? Seems to work ok with it still in place
 *    Trigger from TIMER1 CH1
 */
void adc_config(void)
{
   // ADC0 for gimbals
   adc_deinit(ADC0);

   adc_mode_config(ADC_MODE_FREE);
   adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
   adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
   adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
   adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 4);

   adc_oversample_mode_config(ADC0, ADC_OVERSAMPLING_ALL_CONVERT, ADC_OVERSAMPLING_SHIFT_4B, ADC_OVERSAMPLING_RATIO_MUL16);
   adc_oversample_mode_enable(ADC0);

   // sampletime 28 -> 333333 sps good for 8x oversampling, 10k rate
   // 13 -> 529411 sps, 16x ovs, 8k rate

   #define SAMPLE_TIME (ADC_SAMPLETIME_13POINT5)    // XXX check that the sampling time is long enough

   adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, SAMPLE_TIME);
   adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_1, SAMPLE_TIME);
   adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_2, SAMPLE_TIME);
   adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_3, SAMPLE_TIME);

   adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);
   adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

   adc_enable(ADC0);
   delay(1);
   adc_calibration_enable(ADC0);

   adc_dma_mode_enable(ADC0);

   // ADC1 for battery monitor
   adc_deinit(ADC1);

   adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
   adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);
   adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
   adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);

   // might be nice to have over-sampling on the battery - how does that work with manual triggering?
   // probably the easiest way is to put the ADC into continuous mode and just let it run
   adc_oversample_mode_config(ADC1, ADC_OVERSAMPLING_ALL_CONVERT, ADC_OVERSAMPLING_SHIFT_8B, ADC_OVERSAMPLING_RATIO_MUL256);
   adc_oversample_mode_enable(ADC1);

   #define BAT_SAMPLE_TIME (ADC_SAMPLETIME_239POINT5)    // any advantage to using a shorter sampling time? Reads are low freq on battery

   adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_6, BAT_SAMPLE_TIME);

   // trigger by software
   adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
   adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
   // needed?
   adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);

   adc_enable(ADC1);
   delay(1);
   adc_calibration_enable(ADC1);
}

/**   TODO implement buffered DMA based uart handling
 */
void uart_config(void)
{
   // USART configure
   usart_deinit(USART0);
   usart_baudrate_set(USART0, BAUDRATE);
   usart_word_length_set(USART0, USART_WL_8BIT);
   usart_stop_bit_set(USART0, USART_STB_1BIT);
   usart_parity_config(USART0, USART_PM_NONE);
   usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
   usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
   usart_receive_config(USART0, USART_RECEIVE_ENABLE);
   usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
   usart_enable(USART0);
}

/**
 * Set up SPI1 for talking to the radio module
 */
void spi1_config(void)
{
   spi_parameter_struct spi_init_struct;

   gpio_bit_set(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS high
   spi_struct_para_init(&spi_init_struct);

   /* SPI0 parameter config */
   spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
   spi_init_struct.device_mode          = SPI_MASTER;
   spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
   spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE; // not certain which setting to use
   spi_init_struct.nss                  = SPI_NSS_SOFT;

   // TODO - confirm these clock rates are what they claim to be
   // spi_init_struct.prescale             = SPI_PSC_32; // 1.6 MHz - must be running off a ~50MHz clock
   // spi_init_struct.prescale             = SPI_PSC_16; // 3.2 MHz
   // spi_init_struct.prescale             = SPI_PSC_8; // 6.4 MHz
   spi_init_struct.prescale             = SPI_PSC_4; // 12.8 MHz -- works in v3 dag prototype, works on spring board test setup

   spi_init_struct.endian               = SPI_ENDIAN_MSB;
   spi_init(SPI1, &spi_init_struct);

   spi_enable(SPI1);
}


void setup() {

   // turn on the various peripherals
   clock_config();

   #if defined(T_DISPLAY) || defined(PCB_V1_0)
   // lcd backlight connected to PB10 which we can control with timer1 ch2   
   // remap B10 to the timer
   gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP1, ENABLE);

   // Connect the pin to the (remapped) timer
   gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
   #endif

   // need to remap jtag off
   // default call isn't disabling all the pins, leaving PA14 stuck with an internal pull down resistor
   // that has to be fought with a stronger external pull-up.
   // The following link suggests that a different disable value is needed, bits 100 instead of bits 010
   // https://blog.csdn.net/zoomdy/article/details/101386700
   
   // gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE); // ENABLE to disable jtag - how very confusing!

   // work-around - set the bits directly
   AFIO_PCF0 = (AFIO_PCF0 & 0xF8FFFFFF) | 0x04000000;

   // connect port to USARTx_Tx
   gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

   // connect port to USARTx_Rx
   gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

   // configure led GPIO port - being used for rot enc button now
   // gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED_PIN);
   // GPIO_BOP(LED_GPIO_PORT) = LED_PIN;

   // these are in the ADC channels so can't be used :(
   // gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GREEN_LED | BLUE_LED);
   // GPIO_BOP(GPIOA) = GREEN_LED | BLUE_LED;

   // configure the ADC inputs
   // Need to avoid pins A1 & 2 because of leds, and A5,6,7 and B0,1,2 because of LCD
   // But that only leaves 3 ADC pins, so might as well use 0-3. Maybe depopulate the led? Seems to work ok with it still there.
   // Using PA0 to PA3
   // PA6 added for battery monitoring via second ADC
   gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);

   // Configure the radio SPI
   // SPI1 - PB12 = NSS, PB13 = SCK, PB14 = MISO, PB15 = MOSI
   gpio_init(RADIO_NSS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RADIO_NSS_PIN);
   gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_15);
   gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);

   // radio reset and busy pins
   gpio_init(RADIO_RESET_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RADIO_RESET_PIN);
   gpio_init(RADIO_BUSY_PORT, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, RADIO_BUSY_PIN);

   // radio DIO1 and DIO2 to B8 and B9
   // XXX assumes they are on the same port
   gpio_init(RADIO_DIO1_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, RADIO_DIO1_PIN | RADIO_DIO2_PIN);

   // radio rxen and txen
   #ifdef RADIO_RXEN_PORT
   gpio_init(RADIO_RXEN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RADIO_RXEN_PIN);
   gpio_init(RADIO_TXEN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RADIO_TXEN_PIN);
   // init to low states
   gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
   gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
   #endif

   // pins for the rotary encoder
   gpio_init(RE_BUTTON_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, RE_BUTTON_PIN);   // push button
   // RB6 and 7 are connected to the encoder so that we can read them with TIMER3
   gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

   // pins for the rc switches

   #ifdef SWA_HIGH
   gpio_init(PORT(SWA_HIGH), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWA_HIGH));
   gpio_init(PORT(SWA_LOW), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWA_LOW));

   gpio_init(PORT(SWB_HIGH), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWB_HIGH));
   gpio_init(PORT(SWB_LOW), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWB_LOW));

   gpio_init(PORT(SWC_HIGH), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWC_HIGH));
   gpio_init(PORT(SWC_LOW), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWC_LOW));

   gpio_init(PORT(SWD_HIGH), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWD_HIGH));
   gpio_init(PORT(SWD_LOW), GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, PIN(SWD_LOW));
   #else
   #error "Switch defs needed"
   #endif

   #ifdef GPIO_BUZZER
   gpio_init(PORT(GPIO_BUZZER), GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PIN(GPIO_BUZZER));
   gpio_bit_reset(PORT(GPIO_BUZZER), PIN(GPIO_BUZZER));
   #endif

   uart_config();

   // dma for adc
   dma_config();

   adc_config();

   // enable SPI for radio
   spi1_config();

   // enable interrupts
   eclic_global_interrupt_enable();
   eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
   eclic_irq_enable(TIMER2_IRQn,2,3); // XXX new, was 1, 0, increasing level should allow the timer interrupt to pre-empt lower level interrupts
                                       // such as the ADC interrupt. Increasing the priority should make the timer interrupt be processed first
                                       // if multiple interrupts are pending
   eclic_irq_enable(DMA0_Channel0_IRQn,1,0);

   // enable and set enc button interrupt to the specified priority
   eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
   eclic_irq_enable(EXTI10_15_IRQn, 1, 1);

   // connect key EXTI line to rotary enc button GPIO pin
   gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_13);

   // configure key EXTI line
   exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_BOTH); // was EXTI_TRIG_FALLING, but we need to know about both directions for long press
   exti_interrupt_flag_clear(EXTI_13);

   // enable DIO interrupts
   eclic_irq_enable(EXTI5_9_IRQn, 1, 1);
   gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_8);
   gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_9);
   exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_RISING);
   exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_RISING);
   exti_interrupt_flag_clear(EXTI_8);
   exti_interrupt_flag_clear(EXTI_9);

   timer_config();
}

// ==============================

#define CYCLES 2
#define SHORT_DELAY  100
#define LONG_DELAY  1000

void blink(uint32_t port, uint32_t pin)
{
   for (int i=0; i<CYCLES; i++) {
     GPIO_BC(port) = pin;  // Bit Clear - leds are active low
     delay(SHORT_DELAY);
     GPIO_BOP(port) = pin; // sets pins high, turns off the leds
     delay(SHORT_DELAY);
   }    
}

// ==============================

/**
 * Hybrid switches packet encoding for sending over the air
 *
 * Analog channels are reduced to 10 bits to allow for switch encoding
 * Switch[0] is sent on every packet.
 * A 3 bit switch index and 2 bit value is used to send the remaining switches
 * in a round-robin fashion.
 * If any of the round-robin switches have changed
 * we take the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 * 
 * Inputs: crsf.ChannelDataIn, crsf.currentSwitches
 * Outputs: Radio.TXdataBuffer, side-effects the sentSwitch value
 */
#define RC_DATA_PACKET 0b00

void ICACHE_RAM_ATTR GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, uint16_t *adcData, uint8_t addr)
{
   uint8_t PacketHeaderAddr;
   PacketHeaderAddr = (addr << 2) + RC_DATA_PACKET; // ELRS V1 RC2 doesn't use addr, but it will be overwritten by crc anyway
   Buffer[0] = PacketHeaderAddr;
   Buffer[1] = ((adcData[0]) >> 3);
   Buffer[2] = ((adcData[1]) >> 3);
   Buffer[3] = ((adcData[2]) >> 3);
   Buffer[4] = ((adcData[3]) >> 3);
   Buffer[5] = ((adcData[0] & 0b110) << 5) +
               ((adcData[1] & 0b110) << 3) +
               ((adcData[2] & 0b110) << 1) +
               ((adcData[3] & 0b110) >> 1);


   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)

   // ELRS compatibility V1 RC2 switch format
   // find the next switch to send
   uint8_t nextSwitchIndex = getNextSwitchIndex();

   uint8_t sentValue = currentSwitches[nextSwitchIndex];

   // Actually send switchIndex - 1 in the packet, to shift down 1-7 (0b111) to 0-6 (0b110)
   // If the two high bits are 0b11, the receiver knows it is the last switch and can use
   // that bit to store data
   uint8_t bitclearedSwitchIndex = nextSwitchIndex - 1;
   // currentSwitches[] is 0-15 for index 1, 0-2 for index 2-7 // XXX this seems wrong. most switches have 3 bits available, and last switch is the multiway?
   // Rely on currentSwitches to *only* have values in that range
   // TODO ok, so the new switch position values are _slightly_ counterintuitive, and will need mapping from the existing 0,1,2 encoding.
   //   case 0: return CRSF_CHANNEL_VALUE_1000;
   //   case 5: return CRSF_CHANNEL_VALUE_2000;
   //   case 6: // fallthrough
   //   case 7: return CRSF_CHANNEL_VALUE_MID;
   
   uint8_t value = 0; // default to low position
   switch(currentSwitches[nextSwitchIndex]) {
      case 1:  // middle position
        value = 7;
        break;
      case 2: // high position
        value = 5;
        break;
   }

   Buffer[6] =
         // switch 0 is one bit sent on every packet - intended for low latency arm/disarm
         (currentSwitches[0] / 2) << 6 |   // down-convert the arm switch to on/off, using only the highest setting for on
         // tell the receiver which switch index this is
         bitclearedSwitchIndex << 3 |
         // include the switch value
         value;

   #else

   // switch 0 is sent on every packet - intended for low latency arm/disarm
   Buffer[6] = (currentSwitches[0] & 0b11) << 5; // note this leaves the top bit of byte 6 unused

   // find the next switch to send
   uint8_t nextSwitchIndex = getNextSwitchIndex() & 0b111;     // mask for paranoia
   uint8_t sentValue = currentSwitches[nextSwitchIndex];       // avoid the possibility that the mask changes the value
   uint8_t value = sentValue & 0b11; // mask for paranoia

   // put the bits into buf[6]. nextSwitchIndex is in the range 1 through 7 so takes 3 bits
   // currentSwitches[nextSwitchIndex] is in the range 0 through 2, takes 2 bits.
   Buffer[6] += (nextSwitchIndex << 2) + value;

   #endif // ELRS compatibility for V1 RC2

   // update the sent value
   setSentSwitch(nextSwitchIndex, sentValue);
}

// crsf uses a reduced range, and BF expects to see it.
const static uint32_t MAX_OUT = 1811;
const static uint32_t MID_OUT =  992;
const static uint32_t MIN_OUT =  172;

uint32_t scaleADCtoCRSF(const uint32_t min, const uint32_t centre, const uint32_t max, 
                        const uint32_t adcValue, const bool reversed)
{
   uint32_t res;
   if (adcValue <= min)
   {
      res = MIN_OUT;
   }
   else if (adcValue >= max)
   {
      res = MAX_OUT;
   }
   else if (adcValue > centre)
   {
      // high half scaling
      res = MID_OUT + (adcValue - centre) * (MID_OUT - MIN_OUT) / (max - centre);
   }
   else
   {
      // low half scaling
      res = MID_OUT - ((centre - adcValue) * (MAX_OUT - MID_OUT) / (centre - min));
   }

   if (reversed) {
      res = 1983 - res;
   }

   return res;
}

uint32_t scaleYawData(uint16_t raw_adc_yaw)
{
   uint32_t yaw = scaleADCtoCRSF(ADC_YAW_MIN, ADC_YAW_CTR, ADC_YAW_MAX, raw_adc_yaw, ADC_YAW_REVERSED);

   return yaw;
}

uint32_t scaleThrottleData(uint16_t raw_adc_throttle)
{
   // fake up a midpoint since it's not critical for throttle
   const static uint16_t ADC_THROTTLE_MID = (ADC_THROTTLE_MAX-ADC_THROTTLE_MIN)/2 + ADC_THROTTLE_MIN;

   uint32_t throttle = scaleADCtoCRSF(ADC_THROTTLE_MIN, ADC_THROTTLE_MID, ADC_THROTTLE_MAX, raw_adc_throttle, ADC_THROTTLE_REVERSED);
   return throttle;
}

uint32_t scalePitchData(uint16_t raw_adc_pitch)
{
   uint32_t pitch = scaleADCtoCRSF(ADC_PITCH_MIN, ADC_PITCH_CTR, ADC_PITCH_MAX, raw_adc_pitch, ADC_PITCH_REVERSED);

   return pitch;
}


uint32_t scaleRollData(uint16_t raw_adc_roll)
{
   uint32_t roll = scaleADCtoCRSF(ADC_ROLL_MIN, ADC_ROLL_CTR, ADC_ROLL_MAX, raw_adc_roll, ADC_ROLL_REVERSED);

   return roll;
}


void ICACHE_RAM_ATTR SendRCdataToRF()
{
   // printf("n %u\n\r", NonceTX);
   /////// This Part Handles the Telemetry Response ///////
   if (((uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval > 0) && thisFrameIsTelemetry())
   { // This frame is for telem, so don't attempt to transmit
      // printf("t skip\n\r");
      return;
   }

  uint32_t SyncInterval;
  if (isRXconnected)
  {
    SyncInterval = ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected;
  } else {
    SyncInterval = ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  }


  if ((syncWhenArmed || !isArmed()) &&    // send sync packets when not armed, or always if syncWhenArmed is true
         (  (syncSpamCounter != 0 && (millis() - lastLinkRateChangeTime > 300)) ||  // if we're trying to notify the rx of changing packet rate, just do it, otherwise...
            (
               ((millis() - SyncPacketLastSent) > SyncInterval) && // ...has enough time passed?
               (radio.currFreq == GetInitialFreq()) &&             // we're on the sync frequency
               // sync just after we changed freqs (helps with hwTimer.init() being in sync from the get go)
               (NonceTX % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 1)
            )
         )
      )
  {
      GenerateSyncPacketData();
      SyncPacketLastSent = millis();

    //Serial.println("sync");
    //Serial.println(Radio.currFreq);
  }
  else
  {
      #ifdef USE_ADC_COPRO
      // Get gimbal data from ADC copro

      uint16_t buffer[4];

      digitalWrite(GPIO_PIN_ADCCP_SS, LOW);

      // mosi is not connected, so it doesn't matter what we use as output
      SPI.transferBytes((uint8_t *)buffer, (uint8_t *)buffer, 2 * 4);

      digitalWrite(GPIO_PIN_ADCCP_SS, HIGH);

      crsf.ChannelDataIn[0] = scaleRollData(buffer[0]);
      crsf.ChannelDataIn[1] = scalePitchData(buffer[1]);
      crsf.ChannelDataIn[2] = scaleThrottleData(buffer[2]);
      crsf.ChannelDataIn[3] = scaleYawData(buffer[3]);

      // Serial.print(buffer[0]); Serial.print(' ');
      // Serial.print(crsf.ChannelDataIn[0]); Serial.println();

      #endif // USE_ADC_COPRO

      // TODO make this a runtime mode that displays on the lcd
      // lcd is 135x240, chars are 8x16, so we have 16x15 to work with.
      // Apparently not, looks like the last char isn't usable, so 15x15

      // TODO this seems like a really ugly place to have this code, move it somewhere else?

      // TODO auto calibration mode
      #ifdef STICK_CALIBRATION
      #define SC_UPDATE_INTERVAL 100
      static uint32_t lastCalib = 0;
      u8 buffer[32];
      u16 foreground;
      if (millis() > (lastCalib + SC_UPDATE_INTERVAL))
      {
         lastCalib = millis();
         static uint16_t rollMin = 9999, rollMax = 0, pitchMin = 9999, pitchMax = 0,
                        thrMin = 9999, thrMax = 0, yawMin = 9999, yawMax = 0;

         foreground = WHITE;
         BACK_COLOR = DARKBLUE;

         LCD_ShowString(0, 0, (const u8*)"  MIN  CUR  MAX", foreground);

         uint16_t roll = aud_roll.getCurrent();
         if (roll < rollMin)
            rollMin = roll;
         if (roll > rollMax)
            rollMax = roll;

         sprintf((char*)buffer, "A%4u %4u %4u", rollMin, roll, rollMax);
         printf("%s\n\r", buffer);
         LCD_ShowString(0, 32, buffer, foreground);

         uint16_t pitch = aud_pitch.getCurrent();
         if (pitch < pitchMin)
            pitchMin = pitch;
         if (pitch > pitchMax)
            pitchMax = pitch;

         sprintf((char*)buffer, "E%4u %4u %4u", pitchMin, pitch, pitchMax);
         printf("%s\n\r", buffer);
         LCD_ShowString(0, 48, buffer, foreground);

         uint16_t throttle = aud_throttle.getCurrent();
         if (throttle < thrMin)
            thrMin = throttle;
         if (throttle > thrMax)
            thrMax = throttle;

         sprintf((char*)buffer, "T%4u %4u %4u", thrMin, throttle, thrMax);
         printf("%s\n\r", buffer);
         LCD_ShowString(0, 64, buffer, foreground);

         uint16_t yaw = aud_yaw.getCurrent();
         if (yaw < yawMin)
            yawMin = yaw;
         if (yaw > yawMax)
            yawMax = yaw;

         sprintf((char*)buffer, "R%4u %4u %4u", yawMin, yaw, yawMax);
         printf("%s\n\r", buffer);
         LCD_ShowString(0, 80, buffer, foreground);

      }
      #endif // STICK_CALIBRATION

      uint16_t scaledADC[4];
      scaledADC[0] = scaleRollData(aud_roll.getCurrent());
      scaledADC[1] = scalePitchData(aud_pitch.getCurrent());
      scaledADC[2] = scaleThrottleData(aud_throttle.getCurrent());
      scaledADC[3] = scaleYawData(aud_yaw.getCurrent());

      GenerateChannelDataHybridSwitch8(radio.TXdataBuffer, scaledADC, DeviceAddr);
   }

   ///// Next, Calculate the CRC and put it into the buffer /////

   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)

   // need to clear the crc bits of the first byte before calculating the crc
   radio.TXdataBuffer[0] &= 0b11;

   uint16_t crc = ota_crc.calc(radio.TXdataBuffer, 7);
   radio.TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
   radio.TXdataBuffer[7] = crc & 0xFF;

   #elif (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_DEV_16fbd1d011d060f56dcc9b3a33d9eead819cf440)

   uint8_t crc = ota_crc.calc(radio.TXdataBuffer, 7) + CRCCaesarCipher;
   radio.TXdataBuffer[7] = crc;

   #else // not ELRS_OG_COMPATIBILITY

   // TODO change to crc14
   uint8_t crc = CalcCRC(radio.TXdataBuffer, 7) + CRCCaesarCipher;
   radio.TXdataBuffer[7] = crc;

   #endif // ELRS_OG_COMPATIBILITY


   // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // clear the rx debug pin as we're definitely not listening now

   radio.TXnb(radio.TXdataBuffer, 8);

}

// Update the radio and timer for the specified air rate
void SetRFLinkRate(uint8_t index)
{
   bool invertIQ = false;

   #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
   invertIQ = (bool)(UID[5] & 0x01);
   printf("invertIQ %d\n", invertIQ);
   #endif

   expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
   expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

   #ifdef USE_FLRC
   if (index == 0) { // special case FLRC for testing
      radio.ConfigFLRC(GetInitialFreq());
   } else 
   #endif
   {
      radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
   }


   setTimerISRInterval(ModParams->interval);

   ExpressLRS_currAirRate_Modparams = ModParams;
   ExpressLRS_currAirRate_RFperfParams = RFperf;

   isRXconnected = false;

   // if (UpdateRFparamReq)
   //    UpdateRFparamReq = false;

}


/**   Read the pins for a 3 position switch and return a low/mid/high status
 * 
 *    inputs are my combined port/pin ids
 */
uint32_t getSwitchState(uint32_t pin_high, uint32_t pin_low)
{
   uint8_t hi = gpio_input_bit_get(PORT(pin_high), PIN(pin_high));
   uint8_t low = gpio_input_bit_get(PORT(pin_low), PIN(pin_low));
   uint32_t s = SWITCH_MID;   // if neither pin is pulled low it's in the middle position
   if (hi == 0) {       // switch positions are active low
      s = SWITCH_HIGH;
   } else if (low == 0) {
      s = SWITCH_LOW;
   }
   return s;
}

// ============================

uint32_t loopCounter = 0; // just a debug counter to see what rate we are getting around the event loop at


void updateDisplayNormal()
{
   u8 buffer[32];

   // filterTime = 0;
   // nSamples = 0;

   #ifndef DEBUG_SUPPRESS
   printf("%d %u %u\n\r", rssi, snr, lq);
   #endif

   // Power
   // Packet Rate
   // Telem
   sprintf((char*)buffer, "Pwr  %4u", radio.getPowerMw());
   LCD_ShowString(0, 0, buffer, WHITE);

   sprintf((char*)buffer, "Rate %4lu", 1000000 / ExpressLRS_currAirRate_Modparams->interval);
   LCD_ShowString(0, 16, buffer, WHITE);

   bool telemetryEnabled = ExpressLRS_currAirRate_Modparams->TLMinterval != expresslrs_tlm_ratio_e::TLM_RATIO_NO_TLM;

   if (telemetryEnabled) {
      sprintf((char*)buffer, "Tlm  %4u", TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));
   } else {
      sprintf((char*)buffer, "Tlm   off");
   }
   LCD_ShowString(0, 32, buffer, WHITE);


   if (isRXconnected) {
      sprintf((char*)buffer, "LQ   %4u ", lq);
      LCD_ShowString(0, 64, buffer, WHITE);
      sprintf((char*)buffer, "RSSI %4d ", rssi);
      LCD_ShowString(0, 80, buffer, WHITE);
      sprintf((char*)buffer, "SNR  %4u ", snr);
      LCD_ShowString(0, 96, buffer, WHITE);
   } else if (telemetryEnabled) {
      LCD_ShowString(0, 64, (const u8*)"         ", WHITE);
      LCD_ShowString(0, 80, (const u8*)"NO TELEM ", WHITE);
      LCD_ShowString(0, 96, (const u8*)"         ", WHITE);
   }

   // // debug for a13-15
   // uint8_t p13 = gpio_input_bit_get(PORT(PA13), PIN(PA13));
   // uint8_t p14 = gpio_input_bit_get(PORT(PA14), PIN(PA14));
   // uint8_t p15 = gpio_input_bit_get(PORT(PA15), PIN(PA15));

   // printf("pa13-15: %u %u %u\n\r", p13, p14, p15);

   // printf("%lu %lu %lu %lu\n\r", aud_roll.getCurrent(), aud_pitch.getCurrent(), aud_throttle.getCurrent(), aud_yaw.getCurrent());
   // printf("%lu %lu %lu %lu\n\r", scaleRollData(aud_roll.getCurrent()), 
   //             scalePitchData(aud_pitch.getCurrent()), 
   //             scaleThrottleData(aud_throttle.getCurrent()), 
   //             scaleYawData(aud_yaw.getCurrent()));


   // TTGO has a bigger display, the nano can just use the stuff above
   #if defined(T_DISPLAY) || defined(PCB_V1_0)

   // ARM flag
   if (isArmed()) {
      LCD_ShowString(LCD_H/2 - 25, LCD_W-16, (const u8*)"ARMED", WHITE);
   } else {
      LCD_ShowString(LCD_H/2 - 25, LCD_W-16, (const u8*)"     ", DARKBLUE);
   }

   // is the battery ADC ready (the answer is always going to be yes given the interval on this lcd update code)
   if (SET == adc_flag_get(ADC1, ADC_FLAG_EOC))
   {
      adc_flag_clear(ADC1, ADC_FLAG_EOC);

      // read adc
      uint16_t rawBat = adc_regular_data_read(ADC1);
      // printf("rawBat: %u\n\r", rawBat);

      // max reading is 4096, represents 3V3 at the pin. Ratio on the resistor divider is about 4.
      // Includes a factor of 10 for 1 place of fixed point precision and a fudge factor for calibration
      uint16_t inputVoltage = rawBat*135 / 4096;
      // printf("inputV %u\n\r", inputVoltage);

      if (inputVoltage < LOW_BAT_THRESHOLD) {
         BACK_COLOR = RED;
         batteryLow = true;
      }

      sprintf((char*)buffer, "BAT %u.%u", inputVoltage / 10, inputVoltage % 10);
      LCD_ShowString(35, 144, buffer, WHITE);

      BACK_COLOR = DARKBLUE;
   } // battery ADC ready to read

   #ifdef DEBUG_STATUS
   // get the current status of the switches
   uint32_t newSWA = getSwitchState(SWA_HIGH, SWA_LOW);
   uint32_t newSWB = getSwitchState(SWB_HIGH, SWB_LOW);
   uint32_t newSWC = getSwitchState(SWC_HIGH, SWC_LOW);
   uint32_t newSWD = getSwitchState(SWD_HIGH, SWD_LOW);

   sprintf((char*)buffer, "%lu %lu %lu %lu", newSWA, newSWB, newSWC, newSWD);
   LCD_ShowString(0, 176, buffer, WHITE);


   // cpu usage for filters
   sprintf((char*)buffer, "f %6lu us", filterUpdateTime);
   LCD_ShowString(0, 208, buffer, WHITE);
   filterUpdateTime = 0;

   // show the loop counter so we can see what the loop rate is
   sprintf((char*)buffer, "loops %8lu", loopCounter);
   LCD_ShowString(0, 192, buffer, WHITE);
   loopCounter = 0;
   #endif // DEBUG_STATUS

   #endif // T_DISPLAY
}

/**   Special display mode for changing settings
 * 
 */
void updateDisplayEdit()
{
   u8 buffer[32];

   LCD_ShowString(LCD_H/2 - 32, 0, (const u8*)"Settings", WHITE);

   uint16_t foreground;

   sprintf((char*)buffer, "Pwr  %4u", radio.convertPowerToMw(radioPower));
   if (paramToChange == PARAM_POWER) {
      foreground = DARKBLUE;
      BACK_COLOR = YELLOW;
   } else {
      foreground = WHITE;
      BACK_COLOR = DARKBLUE;
   }
   LCD_ShowString(0, 32, buffer, foreground);

   sprintf((char*)buffer, "Rate %4lu", 1000000 / ExpressLRS_currAirRate_Modparams->interval);
   if (paramToChange == PARAM_RATE) {
      foreground = DARKBLUE;
      BACK_COLOR = YELLOW;
   } else {
      foreground = WHITE;
      BACK_COLOR = DARKBLUE;
   }
   LCD_ShowString(0, 48, buffer, foreground);

   sprintf((char*)buffer, "Tlm  %4u", TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));
   if (paramToChange == PARAM_TLM_INT) {
      foreground = DARKBLUE;
      BACK_COLOR = YELLOW;
   } else {
      foreground = WHITE;
      BACK_COLOR = DARKBLUE;
   }
   LCD_ShowString(0, 64, buffer, foreground);

   if (paramToChange == PARAM_FILTER_MODE) {
      foreground = DARKBLUE;
      BACK_COLOR = YELLOW;
   } else {
      foreground = WHITE;
      BACK_COLOR = DARKBLUE;
   }
   LCD_ShowString(0, 80, (const u8*)"Filt ", foreground);
   switch (currentFilterMode) {
      case FILTER_RACE:
         LCD_ShowString(40, 80, (const u8*)"RACE     ", foreground);
         break;
      case FILTER_FREESTYLE:
         LCD_ShowString(40, 80, (const u8*)"FREESTYLE", foreground);
         break;
      case FILTER_CINEMATIC:
         LCD_ShowString(40, 80, (const u8*)"CINEMATIC", foreground);
         break;
   }

   if (paramToChange == PARAM_SYNC_WHEN_ARMED) {
      foreground = DARKBLUE;
      BACK_COLOR = YELLOW;
   } else {
      foreground = WHITE;
      BACK_COLOR = DARKBLUE;
   }
   LCD_ShowString(0, 96, (const u8*)"Sync ", foreground);
   if (syncWhenArmed) {
         LCD_ShowString(48, 96, (const u8*)"Always", foreground);
   } else {
         LCD_ShowString(48, 96, (const u8*)"Disarm", foreground);
   }

   foreground = WHITE;
   BACK_COLOR = DARKBLUE;

   LCD_ShowString(LCD_H/2 - 40, 200, (const u8*)"Long press", foreground);
   LCD_ShowString(LCD_H/2 - 32, 216, (const u8*)"to save", foreground);


   // TODO exit mechanism
}

// ============================

void updateFilterSettings(uint8_t filterMode)
{
   switch (filterMode) {
      case FILTER_RACE:
         currentFilter = raceFilter;
         break;
      case FILTER_FREESTYLE:
         currentFilter = freestyleFilter;
         break;
      case FILTER_CINEMATIC:
         currentFilter = cinematicFilter;
         break;
      default:
         printf("bad filterMode %u\n\r", filterMode);
         return;
   }

   aud_roll.setNewFilterParams(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta));
   aud_pitch.setNewFilterParams(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta));
   aud_yaw.setNewFilterParams(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta));
   aud_throttle.setNewFilterParams(currentFilter.minCutoff, currentFilter.maxCutoff, int(1.0f/currentFilter.beta));

}

// ==========================

/**
 * Ideally this would use a gpio to detect charging status, but for now we will
 * just check SWD. If the switch is in the high position at startup we will go
 * into a low power "charging mode" display and stay there until reboot.
 */
void checkForChargeMode()
{
   #define CHG_BACKGROUND_COLOUR (0x3 << 11 | 0x3 << 5 | 0x5)

   if (getSwitchState(SWD_HIGH, SWD_LOW) == SWITCH_HIGH) {
      LCD_Clear(CHG_BACKGROUND_COLOUR);
      BACK_COLOR = CHG_BACKGROUND_COLOUR;

      LCD_ShowString(LCD_H/2 - 44, 0, (u8 const *)"CHARGE MODE", WHITE);
      timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, LCD_PWM_CHARGING); // low brightness

      while(true)
      {
         // read battery voltage
         uint16_t rawBat = adc_regular_data_read(ADC1);

         // max reading is 4096, represents 3V3 at the pin. Ratio on the resistor divider is about 4.
         // Includes a factor of 100 for 2 places of fixed point precision and a fudge factor for calibration
         uint16_t inputVoltage = rawBat*1350 / 4096;

         u8 buffer[16];
         sprintf((char*)buffer, "BAT %u.%02u", inputVoltage / 100, inputVoltage % 100);
         LCD_ShowString(31, 144, buffer, WHITE);

         // if user presses the rotary encoder button then cancel the safety check
         if(RESET == gpio_input_bit_get(RE_BUTTON_PORT, RE_BUTTON_PIN)) {
            timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, LCD_PWM_MAX); // low brightness
            break;
         }

         delay(200);
      }
   }
}


/**
 * Startup safety checks. 
 * Don't proceed until all switches are in off position and the throttle is at min.
*/
void runSafetyChecks()
{
   // throttle threshold in the range MIN_OUT to MAX_OUT (i.e. post scaling)
   #define THROTTLE_SAFETY_THRESHOLD 200

   bool firstSafety = true;

   while (getSwitchState(SWA_HIGH, SWA_LOW) != SWITCH_LOW ||
       getSwitchState(SWB_HIGH, SWB_LOW) != SWITCH_LOW ||
       getSwitchState(SWC_HIGH, SWC_LOW) != SWITCH_LOW ||
       getSwitchState(SWD_HIGH, SWD_LOW) != SWITCH_LOW ||
       scaleThrottleData(aud_throttle.getCurrent()) > THROTTLE_SAFETY_THRESHOLD)
   {
      if (firstSafety) {
         LCD_Clear(RED);
         BACK_COLOR = RED;
         firstSafety = false;
         LCD_ShowString(LCD_H/2 - 44, 0, (u8 const *)" MAKE SAFE ", WHITE);
      }
      if (getSwitchState(SWA_HIGH, SWA_LOW) != SWITCH_LOW) {
         LCD_ShowString(LCD_H/2 - 32, 32, (u8 const *)"SWITCH A", WHITE);
      } else {
         LCD_ShowString(LCD_H/2 - 32,32, (u8 const *)"        ", WHITE);
      }
      if (getSwitchState(SWB_HIGH, SWB_LOW) != SWITCH_LOW) {
         LCD_ShowString(LCD_H/2 - 32,48, (u8 const *)"SWITCH B", WHITE);
      } else {
         LCD_ShowString(LCD_H/2 - 32,48, (u8 const *)"        ", WHITE);
      }
      if (getSwitchState(SWC_HIGH, SWC_LOW) != SWITCH_LOW) {
         LCD_ShowString(LCD_H/2 - 32,64, (u8 const *)"SWITCH C", WHITE);
      } else {
         LCD_ShowString(LCD_H/2 - 32,64, (u8 const *)"        ", WHITE);
      }
      if (getSwitchState(SWD_HIGH, SWD_LOW) != SWITCH_LOW) {
         LCD_ShowString(LCD_H/2 - 32,80, (u8 const *)"SWITCH D", WHITE);
      } else {
         LCD_ShowString(LCD_H/2 - 32,80, (u8 const *)"        ", WHITE);
      }
      if (scaleThrottleData(aud_throttle.getCurrent()) > THROTTLE_SAFETY_THRESHOLD) {
         LCD_ShowString(LCD_H/2 - 32,96, (u8 const *)"THROTTLE", WHITE);
      } else {
         LCD_ShowString(LCD_H/2 - 32,96, (u8 const *)"        ", WHITE);
      }

      // if user presses the rotary encoder button then cancel the safety check
      if(RESET == gpio_input_bit_get(RE_BUTTON_PORT, RE_BUTTON_PIN)) {
         break;
      }

      delay(100); // useful or not?
   } // while (not safe)

}


// ============================

int main(void)
{
   uint8_t linkRateIndex = 0; // default value in case we don't have anything in persistant storage

   // uint32_t t0, t1=0;
   // unsigned int nSamples = 0;

   setup();

   Lcd_Init();
   LCD_Clear(DARKBLUE);

   #if defined(T_DISPLAY) || defined(PCB_V1_0)
   setRotation(2);
   #endif

   // startup splash
   BACK_COLOR = DARKBLUE;
   const char * msg = "ExpressLRS";
   int l = strlen(msg);
   int x = LCD_H/2 - l*8/2;
   int y = LCD_W/2 - 8;
   LCD_ShowString(x, y, (u8 const *) msg, WHITE);

   // startup beep?
   #ifdef GPIO_BUZZER
   // beep(30);
   #endif

   delay(500);

   // start the battery monitor ADC (it runs in continuous mode)
   // need to do this before calling checkForChargeMode()
   adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);

   checkForChargeMode();

   runSafetyChecks();

   LCD_Clear(DARKBLUE);
   BACK_COLOR = DARKBLUE;

   printf("in main\n\r");

   FHSSrandomiseFHSSsequence();

   // Try connecting to the radio
   radio.currFreq = GetInitialFreq();
   radio.Begin();

   #ifdef DISARM_POWER
   radio.SetOutputPower(DISARM_POWER);
   #else
   radio.SetOutputPower(radioPower);
   #endif

   // check if we have saved settings in the persistent store
   persistentData_t persistentData = {0,};

   uint8_t res = SimpleStore::read(CONFIG_STORE_ID, sizeof(persistentData), &persistentData);
   if (res == SS_OK) {
      printf("found settings in persistent store\n\r");

      // unpack the settings from the saved data
      radioPower = persistentData.txPower;
      #ifndef DISARM_POWER
      radio.SetOutputPower(radioPower);
      #endif

      linkRateIndex = persistentData.airmodeIndex;

      syncWhenArmed = (bool)persistentData.sync;

      currentFilterMode = persistentData.filterMode;
      updateFilterSettings(currentFilterMode);

      ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)persistentData.telemRatio;

   } else {
      printf("persistent data not found\n\r");
   }

   SetRFLinkRate(linkRateIndex);

   // enable the timer for the tx interrupt
   timer_enable(TIMER2);


   // === Next section is the event loop ===

   unsigned long lastDebug = 0;
   int lastEncValue = 100;
   uint8_t lastSWA = SWITCH_LOW;
   uint8_t lastSWB = SWITCH_LOW;
   uint8_t lastSWC = SWITCH_LOW;
   uint8_t lastSWD = SWITCH_LOW;
   unsigned long lastSwitchMoved = 0;
   bool switchesHaveMoved = false;
   uint8_t lastParamToChange = PARAM_NONE;
   uint8_t encRange = 0;
   bool displayNormal = true; // when true, current display is for 'normal' mode, when false, display is for edit mode
   unsigned long uiButtonDown = 0; // timer for long press detection
   bool statusIsArmed = false;
   while(true) 
   {
      // If there is debug info send it to the uart
      // if (debugReady) {
      //    printf("%s\n", debugStr);
      //    debugReady = false;
      // }

      unsigned long now = millis();

      // RC switches

      // get the current status of the switches
      uint32_t newSWA = getSwitchState(SWA_HIGH, SWA_LOW);
      uint32_t newSWB = getSwitchState(SWB_HIGH, SWB_LOW);
      uint32_t newSWC = getSwitchState(SWC_HIGH, SWC_LOW);
      uint32_t newSWD = getSwitchState(SWD_HIGH, SWD_LOW);

      // if any switch has moved, update the timestamp
      if (newSWA != lastSWA || newSWB != lastSWB || newSWC != lastSWC || newSWD != lastSWD) {
         lastSwitchMoved = now;
         lastSWA = newSWA;
         lastSWB = newSWB;
         lastSWC = newSWC;
         lastSWD = newSWD;
         switchesHaveMoved = true; // TODO rename as rcSwitchesHaveMoved to distuinguish from UI stuff?
      }

      // if a switch has moved, but not for more than the debounce time, then update the active state
      if (switchesHaveMoved && ((now - lastSwitchMoved) > 2)) {
         currentSwitches[0] = newSWA;
         currentSwitches[1] = newSWB;
         currentSwitches[2] = newSWC;
         currentSwitches[3] = newSWD;

         printf("switches: %lu %lu %lu %lu\n\r", newSWA, newSWB, newSWC, newSWD);

         // clear the flag so we don't spam this code
         switchesHaveMoved = false;

         // cancel edit mode if switches are being moved
         if (paramToChange != PARAM_NONE) {
            lastParamToChange = paramToChange;
            paramToChange = PARAM_NONE;
         }

         // since a switch change might cause a display change, force a quick update
         lcdRedrawNeeded = true;
      }

      // Timeout edit mode if UI hasn't been touched
      // Don't worry about wrap around, it's not worth the effort
      if ((paramToChange != PARAM_NONE) && (now  > (lastButtonTime + 15000))) {
         lastParamToChange = paramToChange;
         paramToChange = PARAM_NONE;
         lcdRedrawNeeded = true;
      }

      // Check for long press on the ui button
      // Don't worry about wrap around
      if (uiButtonDown && (now > (uiButtonDown + 1000))) {
         // long press
         printf("long press detected\n\r");
         uiButtonDown = 0; // clear the timer so we don't spam through this code

         LCD_Clear(DARKBLUE);

         BACK_COLOR = YELLOW;
         const char * msg = "SAVED";
         int l = strlen(msg);
         int x = LCD_H/2 - l*8/2;
         int y = LCD_W/2 - 8;
         LCD_ShowString(x, y, (u8 const *) msg, BLACK);

         // populate the data structure
         persistentData.txPower = radioPower;
         persistentData.airmodeIndex = ExpressLRS_currAirRate_Modparams->index;
         persistentData.sync = syncWhenArmed;
         persistentData.filterMode = currentFilterMode;
         persistentData.telemRatio = ExpressLRS_currAirRate_Modparams->TLMinterval;

         SimpleStore::write(CONFIG_STORE_ID, sizeof(persistentData), &persistentData);

         // give the user time to bask in his success
         delay(500);

         LCD_Clear(DARKBLUE);         
         BACK_COLOR = DARKBLUE;
      }

      // UI button (press down on the rotary enc)
      // TODO add double click, use to exit the settings screen (or maybe change between screens, or action a menu item?)
      if (buttonMoved && ((now - lastButtonTime) > 5)) { // time for the button to stop bouncing
         // check the current button state, pressed is pin low
         if(RESET == gpio_input_bit_get(RE_BUTTON_PORT, RE_BUTTON_PIN))
         {
            // printf("ui button pressed\n\r");

            // save the button down time for long press detection
            uiButtonDown = now;

            if (paramToChange == PARAM_NONE && lastParamToChange != PARAM_NONE) {
               paramToChange = lastParamToChange;
               lastParamToChange = PARAM_NONE; // to make sure we only use this once after a timeout
            } else {
               paramToChange = (paramToChange + 1) % PARAM_N_PARAMS;
               // skip PARAM_NONE as that causes exit from edit mode
               if (paramToChange == PARAM_NONE) paramToChange++;
            }
            lcdRedrawNeeded = true; // update the screen immediately 

            // setup the rotary encoder for the selected param
            // The encoder count changes by 2 for each click, so ranges have a *2 multiplier
            switch (paramToChange) {
               case PARAM_POWER:
                  encRange = ((MAX_PRE_PA_POWER + 18) * 2); // MAX_PRE_PA_POWER is the last usable value, not the number of values, so don't subtract 1
                  lastEncValue = (radioPower + 18) * 2 + 100;
                  // printf("currP %d lastEnc %u\n\r", radio.currPWR, lastEncValue);
                  break;

               case PARAM_RATE:
                  encRange = (RATE_MAX - 1) * 2;
                  lastEncValue = ExpressLRS_currAirRate_Modparams->index * 2 + 100;
                  break;

               case PARAM_TLM_INT:
                  encRange = (TLM_RATIO_NUM_VALUES - 1) * 2;
                  lastEncValue = ExpressLRS_currAirRate_Modparams->TLMinterval * 2 + 100;
                  break;

               case PARAM_FILTER_MODE:
                  encRange = (FILTER_NUMBER_MODES - 1) * 2;
                  lastEncValue = currentFilterMode * 2 + 100;
                  break;

               case PARAM_SYNC_WHEN_ARMED:
                  encRange = 2;
                  lastEncValue = syncWhenArmed ? 102 : 100;
            }

            // printf("setting lastEncValue %u\n\r", lastEncValue);
            timer_counter_value_config(TIMER3, lastEncValue);

         } else {
            uiButtonDown = 0; // cancel long button detection if the button is now up
         }
         buttonMoved = false;
      }

      int encValue = timer_counter_read(TIMER3);


      if (lastEncValue != encValue) {
         if (isArmed()) {
            // when armed we want to ignore the rotary encoder, and we also want
            // to prevent any changes to the encValue from suddenly taking effect when the quad is disarmed,
            // so push the last value back into the counter
            timer_counter_value_config(TIMER3, lastEncValue);
         } else {
            if (encValue < 100) {
               encValue = 100;
               timer_counter_value_config(TIMER3, encValue);
            } else if (encValue > encRange + 100) {
               encValue = encRange + 100;
               timer_counter_value_config(TIMER3, encValue);
            }
            // repeat the test now that we've limited the value
            if (lastEncValue != encValue) {
               switch (paramToChange) {
                  case PARAM_POWER:
                     radioPower = (encValue - 100) / 2 - 18;
                     radio.SetOutputPower(radioPower); // update immediately so the user can override low power disarm if needed
                     break;
                  case PARAM_RATE:
                     #ifdef ELRS_OG_COMPATIBILITY
                     // defer the change until we've sent some sync packets to kick the rx to the new setting
                     nextLinkRate = (encValue - 100) / 2;
                     syncSpamCounter = 3;
                     lastLinkRateChangeTime = millis();
                     #else
                     SetRFLinkRate((encValue - 100) / 2);
                     #endif // ELRS_OG_COMPATIBILITY
                     break;
                  case PARAM_TLM_INT:
                     ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)((encValue - 100) / 2);
                     break;
                  case PARAM_FILTER_MODE:
                     currentFilterMode = (encValue - 100) / 2;
                     updateFilterSettings(currentFilterMode);
                     break;
                  case PARAM_SYNC_WHEN_ARMED:
                     syncWhenArmed = (encValue - 100) / 2;
                     break;
               }
               lastEncValue = encValue;
               lcdRedrawNeeded = true;
            }
         }
         // keep track of UI movement for the edit mode timeout
         // re-using the UI button time value, but should be ok
         lastButtonTime = now;
      }

      // have we lost connection with the quad?
      uint32_t localLastTelem = LastTLMpacketRecvMillis; // avoid race conditions
      if (isRXconnected && ((millis() - localLastTelem) > RX_CONNECTION_LOST_TIMEOUT))
      {
         isRXconnected = false;
      }

      // update LCD
      // in stick calibration mode the lcd is used for calibration info
      #ifndef STICK_CALIBRATION
      if (lcdRedrawNeeded || ((now - lastDebug) > 1000)) {
         lastDebug = now;
         lcdRedrawNeeded = false;

         if (paramToChange == PARAM_NONE) {
            // normal display mode
            if (!displayNormal) {
               // clear the screen for mode change
               LCD_Clear(DARKBLUE);
               displayNormal = true;
            }
            updateDisplayNormal();

         } else {
            // edit display mode
            if (displayNormal) {
               // clear the screen for mode change
               LCD_Clear(DARKBLUE);
               displayNormal = false;
            }
            updateDisplayEdit();
         }

         // printf("T %ld Y %ld\n\r", aud_throttle.getCurrent(), aud_yaw.getCurrent());

      } // LCD update/debug
      #endif // STICK_CALIBRATION

      // debug gimbal data
      // if (nSamples != 0) {
      //    printf("%lu %lu %lu %lu\n\r", aud_roll.getCurrent(), aud_pitch.getCurrent(), aud_throttle.getCurrent(), aud_yaw.getCurrent());
      //    nSamples = 0;
      // }

      // React to changes of the armed state
      if (isArmed() && !statusIsArmed) {
         // we have become armed - dim the display and increase tx power
         timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, LCD_PWM_ARMED); // low brightness
         #ifdef DISARM_POWER
         radio.SetOutputPower(radioPower);
         #endif
         statusIsArmed = true;
      } else if (statusIsArmed && !isArmed()) {
         // we have become disarmed - brighten the display, reduce tx power
         timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, LCD_PWM_MAX); // low brightness
         #ifdef DISARM_POWER
         radio.SetOutputPower(DISARM_POWER);
         #endif
         statusIsArmed = false;
      }

      // provide warning beeps if the battery is getting low
      const uint16_t duration = batteryLow ? 25 : 0;
      const uint16_t interval = 5000;
      handleBeeps(interval, duration, now);

      loopCounter++;
   } // event loop
}