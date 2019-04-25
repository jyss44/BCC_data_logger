#include <messages.h>

void PrintMessage(uint8_t message[MESSAGE_SIZE]){
  Serial.print("{");
  for (int i = 0; i < MESSAGE_SIZE; i++) {
    Serial.print(" ");
    Serial.print(message[i]);
  }
  Serial.print(" } \n");
}

void setMessage(message* message, messageType type, bool mainsOn, bool contactOn, bool loadOn) {
  message->type = type;
  message->mainsOn = mainsOn;
  message->contactOn = contactOn;
  message->loadOn = loadOn;

  MakeHeader(message);
}

/**
 * Creates the header byte of the message.
 * Byte:
 * 7 - Ok/Fail
 * 6 - Chunk/Chunk end
 * 5 - Mains on
 * 4 - Contact on
 * 3 - Load on
 * @param message message struct to set header byte
 */
void MakeHeader(message* message) {
  // header structure:

  uint8_t header = 0;

  bool ok = message->mainsOn || (message->mainsOn && message->contactOn && message->loadOn);

  // Set the bits

  message->header = header;
}

void InsertData(message* message, uint8_t loadV[3], uint8_t loadI[3], uint8_t leakI[3]) {
  for (int i = 0; i < 3; i++) {
    message->loadV[i] = loadV[i];
    message->loadI[i] = loadI[i];
    message->leakI[i] = leakI[i];
  }
}

uint8_t* CreateMessageBytes(message* message, int sequenceNo) {
  uint8_t messageBytes[12];

  // Load headers and
  messageBytes[0] = message->header;
  messageBytes[1] = highByte(sequenceNo);
  messageBytes[2] = lowByte(sequenceNo);

  for(int i = 0; i < 3; i ++) {
    messageBytes[3 + i] = message->loadV[i];
    messageBytes[6 + i] = message->loadI[i];
    messageBytes[9 + i] = message->leakI[i];
  }

  return messageBytes;
}
