#include "Arduino.h"
#include "setup.h"

// Initialize pins
void SetupPins() {
  // Digital input
  pinMode(MAINS, INPUT);
  pinMode(CONTACT, INPUT);

  // Digital output
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_FAIL, OUTPUT);

  // Set LED_OK to on
  digitalWrite(LED_OK, HIGH);
  digitalWrite(LED_FAIL, LOW);
}

/**
 * Sets up timer & enables interrupts
 */
void SetupInterrerupt() {
  cli(); // Disable interrupts

  // Setup timer
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // same for TCCR3B

  // Setup compare match register
  OCR3B =  CMP_FAST;

  // turn on CTC mode:
  //TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);

  sei(); // Enable interrupts

  Serial.println("Timers enabled");
}
