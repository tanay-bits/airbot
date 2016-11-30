#ifndef YAWCONTROL__H__
#define YAWCONTROL__H__


#include <Arduino.h>
#include <Servo.h>
#include "lighthouse.h"

#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define LEDPIN 13

const int MAX_SPEED = 160;    		// max ESC write value
const int CONTROL_PERIOD_MS = 22;	// controller fires up every these many milliseconds
const int CONTROL_FREQ = 45;  		// approximate control frequency (Hz)

// Create Servo objects: esc0 - CW wheel, esc1 - CCW wheel
Servo esc0, esc1;

// Create an IntervalTimer object for the controller
IntervalTimer myTimer;
// myTimer.priority(number);  // 0 highest and 255 lowest

// Union data structure to save incoming byte array as floats representing an angle (deg):
union u_tag {
  byte b_angle[4];
  float f_angle[1];
} u; 

// Enumerated type to represent different modes of operation
typedef enum {IDLE=0, READ=1, HOLD=2, TRACK=3, VIVE=4} Mode_datatype;                                  

volatile Mode_datatype mode;                       // declare global var which is the current mode 
volatile bool startup = false;
volatile bool synched = false;
volatile int yawTol = 0;                          // yaw error tolerance
volatile int yawNow = 0;                           // current (sensed) yaw
volatile int yawTarget = 0;                        // target yaw position
volatile float Kp = 0.1, Ki = 0, Kd = 2;           // yawdot = Awy * (wu - wnom)
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_yaw_prev = 0;                       // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int EINT_CAP = 50000;                     // execute slow retraction if Eint >= EINT_CAP
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)
volatile int REFtraj[500];                         // stores reference trajectory
volatile int ctr = 0;                              // counter to step through REFtraj array
volatile int refSize;                              // useful length of REFtraj array

void controller(void);								// ISR
void read_lighthouse(void);
int correct_yaw(float x);
int read_yaw(void);
int calc_error(int diff);
int saturateSpeed(float s);
bool need_to_retract(void);
void calc_send_control(int e_yaw);
void BTserial_clear(void);
void BTserial_block(void);
void IMUserial_clear(void);
void IMUserial_block(void);
void set_mode(Mode_datatype m);
Mode_datatype get_mode(void);
bool readToken(char* token);

#endif