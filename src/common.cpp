#include "common.h"

// TODO: Validate values for RFmodeCycleAddtionalTime and RFmodeCycleInterval for rates lower than 50HZ

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)

#include "SX127xDriver.h"
extern SX127xDriver Radio;

expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0, RATE_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 2, 8},
    {1, RATE_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 2, 8},
    {2, RATE_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_NO_TLM, 2, 10},
    {3, RATE_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_NO_TLM, 2, 10},
    {4, RATE_4HZ, SX127x_BW_500_00_KHZ, SX127x_SF_12, SX127x_CR_4_7, 250000, TLM_RATIO_1_4, 2, 10}}; // for model recovery

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0, RATE_200HZ, -112, 4380, 3500, 2000, 2000, 5000}, // ~ 3 sync packets
    {1, RATE_100HZ, -117, 8770, 3500, 4000, 2000, 5000},
    {2, RATE_50HZ, -120, 17540, 3500, 6000, 2000, 5000},
    {3, RATE_25HZ, -123, 17540, 3500, 12000, 2000, 5000},
    {4, RATE_4HZ, -131, 239620, 30000, 60000, 0, 250}}; // this means always send sync on ch[0] as soon as we can
#endif

#if defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)

#include "SX1280.h"
extern SX1280Driver Radio;

expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // enum_rate,       bw,                 sf,                 cr,            interval, TLMinterval, FHSShopInterval, PreambleLen
    {0, RATE_1KHZ,  SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 1000,  TLM_RATIO_1_128,     8,          12},   // 1000Hz
    // {1, RATE_800HZ, SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 1250,  TLM_RATIO_1_128,     8,          12},   //  800Hz

    {1, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_4_5,    2000,  TLM_RATIO_1_128,     8,          12},
    {2, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000,  TLM_RATIO_1_64,      8,          12},
    {3, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 7692,  TLM_RATIO_1_32,      4,          12},  // 130Hz
    {4, RATE_50HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7,13333,  TLM_RATIO_1_32,      2,          12}}; //  75Hz

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {

    // NB RFmodeCycleInterval is used both for the time between switch packet rates when searching for a connection, and also as a
    //    timeout for deciding the link has failed!
    //    RFmodeCycleAddtionalTime is used to timeout a connection that can't get out of tentative state

    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected
    // {0, RATE_1KHZ,   -99,  753,  500,               1000,                       100,                       5000}, // with hw crc

    // long rx cycle times
    {0, RATE_1KHZ,   -99,  675,  500,               1000,                       100,                       5000}, // no hw crc
    {1, RATE_500HZ, -105, 1626,  500,               1000,                       100,                       5000},
    {2, RATE_250HZ, -108, 3567, 1000,               1000,                       100,                       5000},
    {3, RATE_150HZ, -112, 6660, 1000,               4000,                       100,                       5000},   // todo, see if the large RFmodeCycleAddtionalTime can be reduced
    {4, RATE_50HZ,  -120,12059, 1000,               6000,                       133,                       5000}};

#endif

expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index);
//const expresslrs_mod_settings_s * ExpressLRS_nextAirRate;
expresslrs_mod_settings_s *ExpressLRS_currAirRate;
expresslrs_mod_settings_s *ExpressLRS_prevAirRate;

ICACHE_RAM_ATTR expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index)
{
    // Protect against out of bounds rate
    if (index < 0)
    {
        // Set to first entry in the array (fastest)
        return &ExpressLRS_AirRateConfig[0];
    }
    else if (index > (RATE_MAX - 1))
    {
        // Set to last usable entry in the array (slowest)
        return &ExpressLRS_AirRateConfig[RATE_MAX - 1];
    }
    return &ExpressLRS_AirRateConfig[index];
}

ICACHE_RAM_ATTR expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index)
{
    // Protect against out of bounds rate
    if (index < 0)
    {
        // Set to first entry in the array (200HZ)
        return &ExpressLRS_AirRateRFperf[0];
    }
    else if (index > (RATE_MAX - 1))
    {
        // Set to last usable entry in the array (currently 50HZ)
        return &ExpressLRS_AirRateRFperf[RATE_MAX - 1];
    }
    return &ExpressLRS_AirRateRFperf[index];
}

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

//expresslrs_mod_settings_s *ExpressLRS_nextAirRate;
//expresslrs_mod_settings_s *ExpressLRS_prevAirRate;
bool ExpressLRS_AirRateNeedsUpdate = false;

connectionState_e connectionState = disconnected;
connectionState_e connectionStatePrev = disconnected;

#ifndef MY_UID
//uint8_t UID[6] = {48, 174, 164, 200, 100, 50};
//uint8_t UID[6] = {180, 230, 45, 152, 126, 65}; //sandro unique ID
// uint8_t UID[6] = {180, 230, 45, 152, 125, 173}; // Wez's unique ID
#error "Must define MY_UID"
#else
uint8_t UID[6] = {MY_UID};
#endif

uint8_t CRCCaesarCipher = UID[4];
uint8_t DeviceAddr = UID[5] & 0b111111; // temporarily based on uid until listen before assigning method merged

#define RSSI_FLOOR_NUM_READS 5 // number of times to sweep the noise foor to get avg. RSSI reading
#define MEDIAN_SIZE 20

uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(expresslrs_tlm_ratio_e enumval)
{
    switch (enumval)
    {
    case TLM_RATIO_NO_TLM:
        return 0;
        break;
    case TLM_RATIO_1_2:
        return 2;
        break;
    case TLM_RATIO_1_4:
        return 4;
        break;
    case TLM_RATIO_1_8:
        return 8;
        break;
    case TLM_RATIO_1_16:
        return 16;
        break;
    case TLM_RATIO_1_32:
        return 32;
        break;
    case TLM_RATIO_1_64:
        return 64;
        break;
    case TLM_RATIO_1_128:
        return 128;
        break;
    default:
        return 0;
    }
}
