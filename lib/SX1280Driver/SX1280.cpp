// get access to gnu specific pow10 function
#define _GNU_SOURCE

#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include "SX1280.h"

extern "C" {
#include "../../include/systick.h"
}
#include "../../src/utils.h"
#include "../../src/config.h"

#include <stdio.h>
#include <math.h>

SX1280Hal hal;
/////////////////////////////////////////////////////////////////
SX1280Driver *SX1280Driver::instance = NULL;
//InterruptAssignment_ InterruptAssignment = NONE;

//uint8_t SX127xDriver::_syncWord = SX127X_SYNC_WORD;

//uint8_t SX127xDriver::currPWR = 0b0000;
//uint8_t SX127xDriver::maxPWR = 0b1111;

/* Steps for startup 

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/



uint32_t beginTX;
uint32_t endTX;

void ICACHE_RAM_ATTR SX1280Driver::nullCallback(void){return;};

SX1280Driver::SX1280Driver()
{
    instance = this;
}

void SX1280Driver::End()
{
    hal.end();
    instance->TXdoneCallback = &nullCallback; // remove callbacks
    instance->RXdoneCallback = &nullCallback;
}

// flrc specific setup
void SX1280Driver::setupFLRC()
{
    this->SetMode(SX1280_MODE_STDBY_RC);                                    //step 1 put in STDBY_RC mode
    hal.WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_FLRC); //Step 2: set packet type
    // this->ConfigModParamsFLRC(FLRC_BR_0_325_BW_0_3, FLRC_CR_1_2, BT_DIS);   //Step 5: Configure Modulation Params
    this->ConfigModParamsFLRC(FLRC_BR_1_300_BW_1_2, FLRC_CR_1_2, BT_DIS);   //Step 5: Configure Modulation Params
    hal.WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS

    // setpacketparams for flrc mode
    SetPacketParamsFLRC();

    // setup the syncword - currently have it disabled in void SetPacketParamsFLRC, anything needed here?
}

// lora specific setup
void SX1280Driver::setupLora()
{
    this->SetMode(SX1280_MODE_STDBY_RC);                                    //step 1 put in STDBY_RC mode
    hal.WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    this->ConfigModParams(currBW, currSF, currCR);                          //Step 5: Configure Modulation Params
    hal.WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS
    #ifdef USE_HARDWARE_CRC
    this->SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL);
    #else
    // TODO this ignores the UID based setup of IQ. Doesn't seem to matter, but seems like a problem waiting to happen
    this->SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);
    #endif
}

void SX1280Driver::Begin()
{
    hal.init();
    // hal.TXdoneCallback = &SX1280Driver::TXnbISR;
    // hal.RXdoneCallback = &SX1280Driver::RXnbISR;

    printf("reset\n\r");
    hal.reset();

    // expected value is 43447 (A9B7) (TODO add list of other good values as we see them)
    uint16_t firmwareRev = (((hal.ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (hal.ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));
    printf("Firmware Revision: %u (%X)\n\r", firmwareRev, firmwareRev);
    if (firmwareRev != 0xA9B7) {
        printf("WARNING: firmware not the expected value of 0xA9B7\n\r");
    }

    #ifdef USE_FLRC
    setupFLRC();
    #else
    setupLora();
    #endif // USE_FLRC

    this->SetFrequency(this->currFreq); //Step 3: Set Freq
    this->SetFIFOaddr(0x00, 0x00);      //Step 4: Config FIFO addr

    // Using dual dios for rx and tx done
    this->SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_RX_DONE, SX1280_IRQ_TX_DONE, SX1280_IRQ_RADIO_NONE);
}


void ICACHE_RAM_ATTR SX1280Driver::ConfigFLRC(uint32_t freq)
{
    this->setupFLRC();
    SetFrequency(freq);
}


void ICACHE_RAM_ATTR SX1280Driver::Config(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, 
                                          SX1280_RadioLoRaCodingRates_t cr, uint32_t freq, const uint8_t PreambleLength, const bool invertIQ)
{
    SX1280_RadioLoRaIQModes_t iqMode;

    if (invertIQ) {
        iqMode = SX1280_LORA_IQ_INVERTED;
    } else {
        iqMode = SX1280_LORA_IQ_NORMAL;
    }

    this->SetMode(SX1280_MODE_STDBY_XOSC);
    ConfigModParams(bw, sf, cr);
    #ifdef USE_HARDWARE_CRC
    SetPacketParams(PreambleLength, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_ON, iqMode);
    #else
    SetPacketParams(PreambleLength, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, iqMode);
    #endif

    SetFrequency(freq);
}

/** convert prePA power to mW
* @param power the prePA power as used by currPWR
*/
uint16_t SX1280Driver::convertPowerToMw(int power)
{
    // convert from dBm to mW
    #if defined(RADIO_E28_27)
    // for e28-27, PA output is +27dBm of the pre-PA setting, up to a max of 0 input.
    uint16_t mw = pow10(float(power+27)/10.0f);
    #elif defined(RADIO_E28_20)
    // for e28-20, PA output is +22dBm of the pre-PA setting, up to a max of -2 input.
    uint16_t mw = pow10(float(power+22)/10.0f);
    #elif defined(RADIO_E28_12)
    // for e28-12 output is just the current setting
    uint16_t mw = pow10(float(power)/10.0f) + 0.5; // round to nearest
    #else
    #error("must define a radio module")
    #endif

    return mw;
}

// TODO - make subclasses of sx1280 that provide implementations of this method for each radio module 
// TODO = should this return a float to better represet output of low power modules?
uint16_t ICACHE_RAM_ATTR SX1280Driver::getPowerMw()
{
    return convertPowerToMw(currPWR);
}

/** Set the SX1280 power output
 *  @param power The output level of the sx1280 chip itself (pre any PA) in dBm,
 * range -18 to +13 for non-PA modules. The max power will be capped by MAX_PRE_PA_POWER
 * which must be correctly set for the module being used.
 */
void ICACHE_RAM_ATTR SX1280Driver::SetOutputPower(int8_t power)
{
    #ifndef MAX_PRE_PA_POWER
    #error "Must set MAX_PRE_PA_POWER for sx1280 modules"
    #endif

    if (power > MAX_PRE_PA_POWER) {
        printf("power capped for E28\n");
        power = MAX_PRE_PA_POWER;
    } else if (power < -18) {
        printf("power min limit\n");
        power = -18;
    }

    uint8_t buf[2];
    buf[0] = power + 18;
    buf[1] = (uint8_t)SX1280_RADIO_RAMP_04_US;
    hal.WriteCommand(SX1280_RADIO_SET_TXPARAMS, buf, 2);

    currPWR = power;

    // Serial.print("SetPower raw: ");
    // Serial.println(buf[0]);
    return;
}


void SX1280Driver::SetPacketParams(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, 
                                    SX1280_RadioLoRaCrcModes_t crc, SX1280_RadioLoRaIQModes_t InvertIQ)
{
    uint8_t buf[8];

    buf[0] = SX1280_RADIO_SET_PACKETPARAMS;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = crc;
    buf[5] = InvertIQ;
    buf[6] = 0x00;
    buf[7] = 0x00;

    hal.fastWriteCommand(buf, sizeof(buf));
}

/**
* packetParam1 = AGCPreambleLength
• packetParam2 = SyncWordLength
• packetParam3 = SyncWordMatch
• packetParam4 = PacketType
• packetParam5 = PayloadLength
• packetParam6 = CrcLength
• packetParam7 = Whitening
 */
void SX1280Driver::SetPacketParamsFLRC()
{
    uint8_t buf[8];

    buf[0] = SX1280_RADIO_SET_PACKETPARAMS;
    buf[1] = 0x30;  // PREAMBLE_LENGTH_16_BITS 0x30
    buf[2] = 0x00;  // SyncWordLength FLRC_SYNC_NOSYNC 0x00
    buf[3] = 0x00;  // SyncWordMatch RX_DISABLE_SYNC_WORD 0x00
    buf[4] = 0x00;  // PacketType PACKET_FIXED_LENGTH 0x00
    buf[5] = 8;     // PayloadLength
    buf[6] = 0x10;  // CrcLength Docs are contradictory, this may be 2 bytes: CRC_1_BYTE 0x10
    buf[7] = 0x08;  // 0x08 Whitening must be disabled for FLRC

    hal.fastWriteCommand(buf, sizeof(buf));
}


void SX1280Driver::SetMode(SX1280_RadioOperatingModes_t OPmode)
{

    //if (OPmode == currOpmode)
    //{
     //   return;
    //}

    uint8_t buf3[3]; //TODO make word alignmed

    switch (OPmode)
    {

    case SX1280_MODE_SLEEP:
        hal.WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;

    case SX1280_MODE_CALIBRATION:
        break;

    case SX1280_MODE_STDBY_RC:
        hal.WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
    case SX1280_MODE_STDBY_XOSC:
        hal.WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;

    case SX1280_MODE_FS:
        hal.WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;

    case SX1280_MODE_RX:

        buf3[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf3[1] = 0xFF;
        buf3[2] = 0xFF;
        hal.WriteCommand(SX1280_RADIO_SET_RX, buf3, sizeof(buf3));
        break;

    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf3[0] = 0x00; // periodBase = 1ms, page 71 datasheet
        buf3[1] = 0xFF; // no timeout set for now
        buf3[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        hal.WriteCommand(SX1280_RADIO_SET_TX, buf3, sizeof(buf3));
        break;

    case SX1280_MODE_CAD:
        break;

    default:
        break;
    }

    currOpmode = OPmode;
}

/** default is low power mode, switch to high sensitivity instead
 * */
void setHighSensitivity()
{
    hal.WriteRegister(0x0891, (hal.ReadRegister(0x0891) | 0xC0));
}

// XXX generalise to handle flrc or copy and specialise?

void SX1280Driver::ConfigModParams(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3]; //TODO make word alignmed

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;

    hal.WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    /**
     * If the Spreading Factor selected is SF5 or SF6, it is required to use WriteRegister( 0x925, 0x1E )
     • If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
     • If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister( 0x925, 0x32 ) must be used
    */
    switch (sf) {
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF5:
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF6:
            hal.WriteRegister(0x925, 0x1E); // for SF5 or SF6
            break;
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF7:
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF8:
            hal.WriteRegister(0x925, 0x37); // for SF7 or SF8
            break;
        default:
            hal.WriteRegister(0x925, 0x32); // for SF9 and above
    }

    setHighSensitivity();
}

void SX1280Driver::ConfigModParamsFLRC(SX1280_RadioFLRCBandwidths_t bw, SX1280_RadioFLRCCodingRates_t cr, SX1280_RadioFLRCBTFilter_t bt)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3]; //TODO make word aligned

    rfparams[0] = (uint8_t)bw;
    rfparams[1] = (uint8_t)cr;
    rfparams[2] = (uint8_t)bt;

    hal.WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    setHighSensitivity();
}


void SX1280Driver::SetFrequency(uint32_t Reqfreq)
{
    //Serial.println(Reqfreq);
    uint8_t buf[3]; //TODO make word alignmed

    uint32_t freq = (uint32_t)((double)Reqfreq / (double)SX1280_FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    hal.WriteCommand(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
    currFreq = Reqfreq;
}

int32_t SX1280Driver::GetFrequencyError()
{
    uint8_t efeRaw[3] = {0}; //TODO make word alignmed
    uint32_t efe = 0;
    double efeHz = 0.0;

    efeRaw[0] = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    efeRaw[1] = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
    efeRaw[2] = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
    efe = (efeRaw[0] << 16) | (efeRaw[1] << 8) | efeRaw[2];

    efe &= SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

    printf("GetFrequencyError IMPL NEEDED\n");

    //efeHz = 1.55 * (double)complement2(efe, 20) / (1600.0 / (double)GetLoRaBandwidth() * 1000.0); XXX wuuuuut?
    return efeHz;
}

void SX1280Driver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

// TODO use bigger buffer and a fastWrite
void SX1280Driver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal.WriteCommand(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void SX1280Driver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal.WriteCommand(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

// void SX1280Driver::TXnbISR()
// {
//     //endTX = micros();
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     instance->currOpmode = SX1280_MODE_FS; // radio goes to FS
//     //Serial.print("TOA: ");
//     //Serial.println(endTX - beginTX);
//     //instance->GetStatus();

//     // Serial.println("TXnbISR!");
//     //instance->GetStatus();
    
//     //instance->GetStatus();
//     instance->TXdoneCallback();
// }

// TODO - how does the syncword get set on TX?

void SX1280Driver::TXnb(volatile uint8_t *data, uint8_t length)
{
    instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    hal.TXenable(); // do first to allow PA stablise
    instance->SetFIFOaddr(0x00, 0x00);   // not 100% sure if needed again
    hal.WriteBuffer(0x00, data, length); //todo fix offset to equal fifo addr
    instance->SetMode(SX1280_MODE_TX);
    // beginTX = micros();
}

// void SX1280Driver::RXnbISR()
// {
//     instance->currOpmode = SX1280_MODE_FS; // XXX is this true? Unless we're doing single rx with timeout, we'll still be in rx mode

//     // Need to check for hardware crc error
//     #ifdef USE_HARDWARE_CRC
//     // grab the status before we clear it
//     uint16_t irqStat = GetIrqStatus();
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     if (irqStat & 0b1000000) {
//         // Serial.println("bad hw crc");
//         return;
//     }
//     #else
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     #endif

//     uint8_t FIFOaddr = instance->GetRxBufferAddr();
//     hal.ReadBuffer(FIFOaddr, instance->RXdataBuffer, 8);
//     instance->RXdoneCallback();
// }

void SX1280Driver::readRXData()
{
    uint8_t FIFOaddr = instance->GetRxBufferAddr();
    hal.ReadBuffer(FIFOaddr, instance->RXdataBuffer, 8);
}

void SX1280Driver::RXnb()
{
    //Serial.println("Start RX nb");
    hal.RXenable();
    instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    //instance->SetFIFOaddr(0x00, 0x00);
    instance->SetMode(SX1280_MODE_RX);
}

uint8_t ICACHE_RAM_ATTR SX1280Driver::GetRxBufferAddr()
{
    uint8_t status[2];
    hal.ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void ICACHE_RAM_ATTR SX1280Driver::GetStatus()
{
    uint8_t status = 0;

    uint8_t stat1;
    uint8_t stat2;
    bool busy;

    hal.ReadCommand(SX1280_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    stat1 = (0b11100000 & status) >> 5;
    stat2 = (0b00011100 & status) >> 2;
    busy = 0b00000001 & status;
    printf("Status: %u, %u, %u (%X)\n", stat1, stat2, busy, status);
}

bool ICACHE_RAM_ATTR SX1280Driver::GetFrequencyErrorbool()
{
    printf("GetFrequencyErrorbool IMPL NEEDED\n");
    //uint8_t val = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    //uint8_t val1 = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
    //uint8_t val2 = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
    // uint8_t regEFI[3];

    // hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, regEFI, 3);

    //Serial.println(val);
    //Serial.println(val1);
    //Serial.println(val2);

    // Serial.println(regEFI[0]);
    // Serial.println(regEFI[1]);
    // Serial.println(regEFI[2]);

    //bool result = (val & 0b00001000) >> 3;
    //return result; // returns true if pos freq error, neg if false
    return 0;
}

void ICACHE_RAM_ATTR SX1280Driver::SetPPMoffsetReg(int32_t offset) { return; };


// TODO get rssi/snr can be made more efficient by using a single call to get
// both values.
int8_t ICACHE_RAM_ATTR SX1280Driver::GetLastPacketRSSI()
{
    uint8_t status[2];

    hal.ReadCommand(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    LastPacketRSSI = -(int8_t)(status[0]/2);
    // Serial.print("rssi read "); Serial.println(LastPacketRSSI);

    // grab snr while we have the buffer
    LastPacketSNR = (int8_t)(status[1]/4);

    return LastPacketRSSI;
}

int8_t ICACHE_RAM_ATTR SX1280Driver::GetLastPacketSNR()
{
    uint8_t status[2];

    hal.ReadCommand(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    LastPacketSNR = ((int8_t)status[1])/4;

    // grab rssi while we have the buffer
    LastPacketRSSI = -(int8_t)(status[0]/2);

    return LastPacketSNR;
}

uint16_t ICACHE_RAM_ATTR SX1280Driver::GetIrqStatus()
{
    uint8_t status[2];

    hal.ReadCommand(SX1280_RADIO_GET_IRQSTATUS, status, 2);

    return (((uint16_t)status[0]) << 8) + status[1];
}
