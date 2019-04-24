/// Digital input pins
const int MAINS = 3;
const int CONTACT = 4;
const int LED_OK = 14;
const int LED_FAIL = 15;
const int CMP_FAST = 43;
const int CMP_SLOW = 15625;

enum sendStates {
  SEND_DATA,
  SEND_STATUS
};

// Function prototypes
void SetupPins();
void SetupInterrerupt();
