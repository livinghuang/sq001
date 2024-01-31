#ifndef LORAWAN_H
#define LORAWAN_H

#include "global.h"

#define LORA_RTS LORA_STA
#define LORA_CTS LORA_READY

// /* OTAA para*/
// extern uint8_t devEui[];
// extern uint8_t appEui[];
// extern uint8_t appKey[];

// /* ABP para*/
// extern uint8_t nwkSKey[];
// extern uint8_t appSKey[];
// extern uint32_t devAddr;

// /*LoraWan channelsmask, default channels 0-7*/
// extern uint16_t userChannelsMask[];

// /*LoraWan region,
//     LORAMAC_REGION_AS923        //  AS band on 923MHz
//     LORAMAC_REGION_AU915        //  Australian band on 915MHz
//     LORAMAC_REGION_CN470        //  Chinese band on 470MHz
//     LORAMAC_REGION_CN779        //  Chinese band on 779MHz
//     LORAMAC_REGION_EU433        //  European band on 433MHz
//     LORAMAC_REGION_EU868        //  European band on 868MHz
//     LORAMAC_REGION_KR920        //  South korean band on 920MHz
//     LORAMAC_REGION_IN865        //  India band on 865MHz
//     LORAMAC_REGION_US915        //  North american band on 915MHz
//     LORAMAC_REGION_US915_HYBRID //  North american band on 915MHz with a maximum of 16 channels
//     LORAMAC_REGION_AU915_SB2    //  Australian band on 915MHz Subband 2
//     LORAMAC_REGION_AS923_AS1    //  AS band on 922.0-923.4MHz
//     LORAMAC_REGION_AS923_AS2    //  AS band on 923.2-924.6MHz
// */
// extern LoRaMacRegion_t loraWanRegion;

// /*LoraWan Class, Class A and Class C are supported*/
// extern DeviceClass_t loraWanClass;

// /*the application data transmission duty cycle.  value in [ms].*/
// extern uint32_t appTxDutyCycle;

// /*OTAA or ABP*/
// extern bool overTheAirActivation;

// /*ADR enable*/
// extern bool loraWanAdr;

// /* Indicates if the node is sending confirmed or unconfirmed messages */
// extern bool isTxConfirmed;

// /* Application port */
// extern uint8_t appPort;
// /*!
//  * Number of trials to transmit the frame, if the LoRaMAC layer did not
//  * receive an acknowledgment. The MAC performs a datarate adaptation,
//  * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
//  * to the following table:
//  *
//  * Transmission nb | Data Rate
//  * ----------------|-----------
//  * 1 (first)       | DR
//  * 2               | DR
//  * 3               | max(DR-1,0)
//  * 4               | max(DR-1,0)
//  * 5               | max(DR-2,0)
//  * 6               | max(DR-2,0)
//  * 7               | max(DR-3,0)
//  * 8               | max(DR-3,0)
//  *
//  * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
//  * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
//  */
// extern uint8_t confirmedNbTrials;
void lorawan_init(void);
void lorawan_process(void);
void generate_lorawan_parameters_by_chip_id(void);
#endif