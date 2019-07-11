#include "Arduino.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x61, 0x0A, 0x12, 0xBB, 0x27, 0x84, 0xDD, 0x3F, 0xCA, 0xE8, 0x13, 0xEC, 0x0B, 0x6B, 0x84, 0xCC };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0x40, 0x41, 0xC6, 0x7D, 0x13, 0x66, 0x59, 0x72, 0x43, 0x53, 0xCC, 0x3E, 0x18, 0x3F, 0xE1, 0x52 };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x26061C53;

static osjob_t initjob,sendjob,blinkjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned INTERVAL_STATUS = 10;
const unsigned INTERVAL_DATA = 10;
