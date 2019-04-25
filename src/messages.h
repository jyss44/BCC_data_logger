#include "Arduino.h"

#define MESSAGE_SIZE 12

// Struct definitions
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

// Function prototypes
void PrintMessage(uint8_t message[MESSAGE_SIZE]);
void setMessage(message* message, messageType type, bool mainsOn, bool contactOn, bool loadOn);
void MakeHeader(message* message);
void InsertData(message* message, uint8_t loadV[3], uint8_t loadI[3], uint8_t leakI[3]);
uint8_t* CreateMessageBytes(message* message, int sequenceNo);
