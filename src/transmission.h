#include "Arduino.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x21, 0x93, 0x89, 0x92, 0x07, 0xC3, 0x02, 0x29, 0x49, 0xE7, 0x20, 0xF8, 0x9D, 0x53, 0x15, 0x32 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0x88, 0xAE, 0x5D, 0x73, 0x4E, 0x23, 0xD1, 0x71, 0xC7, 0x16, 0xFA, 0x58, 0xD4, 0x67, 0xB3, 0x77 };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x260626DF;

static osjob_t initjob,sendjob,blinkjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned INTERVAL_STATUS = 30;
const unsigned INTERVAL_DATA = 10;
