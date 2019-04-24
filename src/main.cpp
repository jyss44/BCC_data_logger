/*******************************************************************************
* Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
*  0.1% in g2).
*
* Change DEVADDR to a unique address!
* See http://thethingsnetwork.org/wiki/AddressSpace
*
* Do not forget to define the radio type correctly in config.h.
*
*
* Required Library:
*    * https://github.com/matthijskooijman/arduino-lmic
*
* Require Hardware:
*    * LoRa Shield + Arduino
*    * LoRa GPS Shield + Arduino
*    * LoRa Mini etc.
*******************************************************************************/
#include "Arduino.h"
#include <lmic.h>                       // https://github.com/matthijskooijman/arduino-lmic
#include <hal/hal.h>
#include <SPI.h>
#include <CircularBuffer.h>             // https://github.com/rlogiacco/CircularBuffer
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg
//#include <MCP9800.h>                    // https://github.com/JChristensen/MCP9800

#include "main.h"
#include "setup.h"
#include "transmission.h"

static uint8_t mydata[12] = "abcdefg";
enum sendStates sendState;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println("OP_TXRXPEND, not sending");
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      Serial.println("Packet queued");
      Serial.println(LMIC.freq);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println(ev);
  switch(ev) {
      case EV_SCAN_TIMEOUT:
          Serial.println("EV_SCAN_TIMEOUT");
          break;
      case EV_BEACON_FOUND:
          Serial.println("EV_BEACON_FOUND");
          break;
      case EV_BEACON_MISSED:
          Serial.println("EV_BEACON_MISSED");
          break;
      case EV_BEACON_TRACKED:
          Serial.println("EV_BEACON_TRACKED");
          break;
      case EV_JOINING:
          Serial.println("EV_JOINING");
          break;
      case EV_JOINED:
          Serial.println("EV_JOINED");
          break;
      case EV_RFU1:
          Serial.println("EV_RFU1");
          break;
      case EV_JOIN_FAILED:
          Serial.println("EV_JOIN_FAILED");
          break;
      case EV_REJOIN_FAILED:
          Serial.println("EV_REJOIN_FAILED");
          break;
      case EV_TXCOMPLETE:
          Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
          if(LMIC.dataLen) {
              // data received in rx slot after tx
              Serial.print("Data Received: ");
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
          }
          // Schedule next transmission
          os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
          break;
      case EV_LOST_TSYNC:
          Serial.println("EV_LOST_TSYNC");
          break;
      case EV_RESET:
          Serial.println("EV_RESET");
          break;
      case EV_RXCOMPLETE:
          // data received in ping slot
          Serial.println("EV_RXCOMPLETE");
          break;
      case EV_LINK_DEAD:
          Serial.println("EV_LINK_DEAD");
          break;
      case EV_LINK_ALIVE:
          Serial.println("EV_LINK_ALIVE");
          break;
       default:
          Serial.println("Unknown event");
          break;
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Starting");

  // Setup constants
  sendState = SEND_STATUS;

  // Setup arduino hardware
  SetupPins();
  SetupInterrerupt();

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

// Timer 3 Interrupt
ISR(TIMER3_COMPB_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
  if (sendState == SEND_STATUS) {
    // gather data

    // Switch machine state, if failure conditions met
    if ((!digitalRead(MAINS) && !digitalRead(CONTACT))){
      // Set LED_OK to on
      digitalWrite(LED_OK, LOW);
      digitalWrite(LED_FAIL, HIGH);
      sendState = SEND_DATA;
    }
  }
}
