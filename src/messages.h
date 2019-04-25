#include "Arduino.h"

#define MESSAGE_SIZE  12

typedef enum messageType {
  STATUS,
  CHUNK,
  CHUNK_END
};

typedef struct {
  messageType type;
  bool mainsOn;
  bool contactOn;
  bool loadOn;

  uint8_t header;
  uint8_t loadV[3];
  uint8_t loadI[3];
  uint8_t leakI[3];
} message;

void PrintMessage(uint8_t message[MESSAGE_SIZE]);
void MakeHeader(message* message);
