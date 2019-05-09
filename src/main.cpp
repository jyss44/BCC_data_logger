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
#include <TimerThree.h>

#include "main.h"
#include "setup.h"
#include "transmission.h"
#include "messages.h"

using namespace std;

// data
CircularBuffer<uint8_t, NO_SAMPLES> LoadV;
CircularBuffer<uint8_t, NO_SAMPLES> LoadI;
CircularBuffer<uint8_t, NO_SAMPLES> LeakI;
movingAvg avgLoadV(FS);

// Messages
uint8_t* myData = (uint8_t*) malloc(MESSAGE_SIZE * sizeof(uint8_t));
message myMessage;
int sequenceNo = 0;
enum sendStates sendState;
bool full = false;
bool empty = false;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

void do_send(osjob_t* j){
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println("Sending data...");
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println("OP_TXRXPEND, not sending");
  } else {
    messageType type = CHUNK;

    // Get message information
    bool status = sendState == SEND_STATUS; // Status of device
    empty = (LoadV.size() - 1 <= 1) && (LoadI.size() - 1) <= (1 && LeakI.size() - 1 <= 1);

    // Insert data
    if (sendState == SEND_DATA) {
      // Ensure that data is not empty
      for (int i = 0; i < 3; i++){
        if (!LoadV.isEmpty() && !LoadI.isEmpty() && !LeakI.isEmpty()) {
            myMessage.loadV[i] = LoadV.pop();
            myMessage.loadI[i] = LoadI.pop();
            myMessage.leakI[i] = LoadI.pop();
        } else {
          Serial.println("Data buffer emptied");
          myMessage.loadV[i] = 0;
          myMessage.loadI[i] = 0;
          myMessage.leakI[i] = 0;

          sendState == SEND_STATUS;
          type = CHUNK_END;
        }
      }
    }

    // Create message header
    setMessage(&myMessage, type, status, digitalRead(MAINS), digitalRead(CONTACT), avgLoadV.getAvg() < THRESH);


    CreateMessageBytes(&myMessage, sequenceNo);
    uint8_t* ass = myMessage.messageBytes;

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, myData, sizeof(myData)-1, 0);
    PrintMessage(ass);
    Serial.println("Packet sent");
    Serial.println(LMIC.freq);
    sequenceNo++;
    Serial.println();
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

        if (sendState == SEND_STATUS) {
          os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(INTERVAL_STATUS), do_send);
          Serial.println("Status packet queued");
        } else if (sendState == SEND_DATA) {
          os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(INTERVAL_DATA), do_send);
          Serial.println("Data packet queued");
        }

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
  Serial.println("Starting...");

  // Setup constants
  sendState = SEND_STATUS;

  // Setup arduino hardware
  SetupPins();

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

  for (uint8_t i = 0; i < 9; i++) {
    if (i != CHANNEL) {
      LMIC_disableChannel(i);
    }
  }

  Timer3.initialize(PERIOD);
  Timer3.attachInterrupt(timerInterrupt);

  avgLoadV.begin(); // Start moving average

  // Start job
  do_send(&sendjob);
}

/**
 * Main loop
 */
void loop() {
  checkState();
  os_runloop_once();
}

// Timer 3 Interrupt
void timerInterrupt() {//timer1 interrupt 1Hz toggles pin 13 (LED)
  //Serial.println("Tick");
  readValues();
}


void readValues() {
  if (sendState == SEND_STATUS) {
    // Read Load V & I, and leakage I from analog pins.
    LoadV.push(analogRead(A2));
    LoadI.push(analogRead(A1));
    LeakI.push(analogRead(A0));

    avgLoadV.reading(abs(analogRead(A2) - BIAS));

    if(!full && LoadV.isFull() && LoadI.isFull() && LeakI.isFull()) {
      Serial.print(os_getTime());
      Serial.print(": ");
      Serial.println("Buffer is now full.");
      full = true;
    }

    //Serial.println(avgLoadV.getAvg());
  }
}

void checkState() {
  if (sendState == SEND_STATUS) {
    // Switch machine state, if failure conditions met
    bool ok = (digitalRead(MAINS) && !digitalRead(CONTACT) && avgLoadV.getAvg() < THRESH) || (digitalRead(MAINS) && digitalRead(CONTACT) && avgLoadV.getAvg() > THRESH);// && avgLoadV.getAvg() < THRESH);

    if (!ok){
      // Set LED_OK to on
      digitalWrite(LED_OK, LOW);
      digitalWrite(LED_FAIL, HIGH);

      // Clear queue & schedule message immediately
      os_clearCallback(&sendjob);
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(INTERVAL_DATA), do_send);

      // Print to Serial
      Serial.print(os_getTime());
      Serial.print(": ");
      Serial.println("Failure detected. Starting data stream...");
      sendState = SEND_DATA;
    }
  }
}
