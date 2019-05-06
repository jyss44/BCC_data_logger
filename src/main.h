// Constants
#define NO_SAMPLES 1428
#define THRESH 5
#define BIAS 400
#define PERIOD 1038
#define PERIOD_LONG 2000000
#define FS 760

// Function prototypes
void do_send(osjob_t* j);
void onEvent (ev_t ev);
void readValues();
void checkState();
void timerInterrupt();
