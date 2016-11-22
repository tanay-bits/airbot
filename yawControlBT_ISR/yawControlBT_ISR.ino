/***
 * Positive yaw change -> CW robot rotation
 ***/

///////////////////////////
// IMPORTS AND CONSTANTS //
///////////////////////////

#include <Servo.h>
#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define LEDPIN 13
const unsigned short PRINTAFTER = 20000;
const float CONTROL_PERIOD_MS = 22;
const int CONTROL_FREQ = 45;  // approximate control frequency (Hz)


/////////////
// GLOBALS //
/////////////

// For serial data parsing in trajectory following mode
byte ind = 0;                    // index of integer array where time or ang data is stored

// Union data structure to save incoming byte array as floats representing an angle (deg):
union u_tag {
  byte b_angle[4];
  float f_angle[1];
} u; 

// Enumerated type to represent different modes of operation
typedef enum {IDLE=0, READ=1, HOLD=2, TRACK=3} Mode_datatype;

// Create Servo objects: esc0 - CW wheel, esc1 - CCW wheel
Servo esc0, esc1;                                   

// Create an IntervalTimer object 
IntervalTimer myTimer;
// myTimer.priority(number);  // 0 highest and 255 lowest

volatile Mode_datatype mode;                       // declare global var which is the current mode 
volatile bool startup = false;
volatile bool synched = false;
volatile int yawTol = 5;                          // yaw error tolerance
volatile int yawNow = 0;                           // current (sensed) yaw
volatile int yawTarget = 0;                        // target yaw position
volatile float Kp = 500, Ki = 100, Kd = 0;           // yawdot = Awy * (wu - wnom)
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_yaw_prev = 0;                       // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)
volatile int REFtraj[500];                         // stores reference trajectory
volatile int ctr = 0;                              // counter to step through REFtraj array
volatile int refSize;                              // useful length of REFtraj array

//////////////////////
// HELPER FUNCTIONS //
//////////////////////

// Maps yaw to (0:360) system
int correct_yaw(float x) {
  if (x < 0)
  {
    x = x + 360;
  }
  else if (x > 360)
  {
    x = x - 360;
  }
  return (int)x;
}

int read_yaw(void) {
  IMUserial_clear();  // Clear input buffer
  IMUSERIAL.write("#f");  // Request one output frame
  while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
  for (int i = 0; i < 4; i++)  // Read yaw angle
  {
    u.b_angle[i] = IMUSERIAL.read();    
  }
  return correct_yaw(u.f_angle[0]);
}

int calc_error(int diff) {
  if (diff > 180)
  {
    diff = diff - 360;
  }
  else if (diff < -180)
  {
    diff = 360 + diff;
  }
  return diff;
}

// Limits motor speeds to 0-150
int saturateSpeed(float s) {
  if (s > 150) {return 150;}
  else if (s < 0) {return 0;}
  else {return s;}
}

// calculate control signal and send to actuator, if error outside deadband
void calc_send_control(int e_yaw) {
  if (abs(e_yaw) > yawTol)
  {
    control_sig = Kp*e_yaw + Ki*Eint + Kd*(e_yaw - e_yaw_prev);  // ~ motor speed change
    vals[0] = saturateSpeed(vals[0] - control_sig);  // new speed for motor 0
    vals[1] = saturateSpeed(vals[1] + control_sig);  // new speed for motor 1
    esc0.write(vals[0]);
    esc1.write(vals[1]);
  }
}

void BTserial_clear(void) {
  while (BTSERIAL.available()) {BTSERIAL.read();}
}

void BTserial_block(void) {
  while (!BTSERIAL.available()) {;}
}

void IMUserial_clear(void) {
  while (IMUSERIAL.available()) {IMUSERIAL.read();}
}

void IMUserial_block(void) {
  while (!IMUSERIAL.available()) {;}
}

void set_mode(Mode_datatype m) {
  mode = m;
}

Mode_datatype get_mode(void) {
  return mode;
}

// Only needed if continuous output streaming is on
bool readToken(char* token) {
  IMUserial_clear(); // Clear input buffer
  
  IMUSERIAL.write("#s00"); // Request synch token
  delay(500);
  // Check if incoming bytes match token
  for (unsigned int i = 0; i < (sizeof(token) - (unsigned)1); i++)
    {
      if (IMUSERIAL.read() != token[i])
        return false;
    }
  return true;
}

// Generate trajectory array to follow
void genRef(volatile int *ref, int *times, int *angs) {
  int sample_list[3];

  for (int i = 0; i < 3; i++)
  {
    sample_list[i] = (int)(times[i] * CONTROL_FREQ);
  }

  refSize = sample_list[2]; // last element of sample_list (global)
  int j = 1;
  for (int i = 0; i < refSize; i++)
  {
   if (i == sample_list[j] - 1) {j++;}
   ref[i] = angs[j-1];  
  }
}

/////////
// ISR //
/////////

void controller(void)
{  
  int e_yaw;

  switch (get_mode()) {
    case IDLE:
    {
      break;
    }

    case READ:
    {
      yawNow = read_yaw();
      if (BTSERIAL.available())
      {
        set_mode(IDLE);
      }
      break;
    }
    
    case HOLD:
    {
      // calculate control error and integral of error
      yawNow = read_yaw();
      e_yaw = calc_error(yawTarget - yawNow);
      Eint = Eint + e_yaw;

      // calculate control signal and send to actuator, if error outside deadband
      calc_send_control(e_yaw);

      // update previous error for derivative calculation
      e_yaw_prev = e_yaw;

      if (BTSERIAL.available())
      {
        set_mode(IDLE);
      }
      break;
    }

    case TRACK:
    {
      yawNow = read_yaw();
      yawTarget = REFtraj[ctr];
      e_yaw = calc_error(yawTarget - yawNow);
      Eint = Eint + e_yaw;

      // calculate control signal and send to actuator, if error outside deadband
      calc_send_control(e_yaw);

      // update previous error for derivative calculation
      e_yaw_prev = e_yaw;

      ctr++;
      if (ctr == refSize)
      {
        yawTarget = REFtraj[refSize-1];  // set last ref as target for HOLD
        ctr = 0;  // reset counter
        set_mode(HOLD);
      }
      break;
    }

    default:
    {
      digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
      break;
    }
  }
}


////////////////////
// SETUP AND LOOP //
////////////////////

void setup() {
  delay(3000);  // Give enough time for Razor to auto-reset
  noInterrupts();
  
  // Initialize serial channels:
  BTSERIAL.begin(9600);
//  IMUSERIAL.begin(58824);
  IMUSERIAL.begin(57600);
//  delay(1000);
  
  // Arm the ESC's:
  esc0.attach(16);
  esc1.attach(17);
  esc0.write(vals[0]);
  esc1.write(vals[1]);
  
  // Set Razor output parameters:
  IMUSERIAL.write("#ob");  // Turn on binary output
  IMUSERIAL.write("#o0");  // Turn OFF continuous streaming output
  IMUSERIAL.write("#oe0"); // Disable error message output 

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);  // turn on LED

  interrupts();
  myTimer.begin(controller, CONTROL_PERIOD_MS * 1000);  // controller to run every 22ms (IMU sensing is at 20ms)
}

void loop() {
  // Wait for user to send something to indicate the connection is ready
  if (!startup)
  {
    if (BTSERIAL.available())
    {
      for (int i=10; i < 75; i = i+5)
      {
        vals[0] = i;
        vals[1] = i;
        esc0.write(vals[0]);
        esc1.write(vals[1]);
      }
      startup = true;
      BTSERIAL.println("Startup successful");
      digitalWrite(LEDPIN, LOW);  // turn off LED to indicate startup
      BTserial_clear();  // Empty the input buffer
      delay(200);    
    }
  }

  if (startup)
  {
    // Keep polling for user input
    if (BTSERIAL.available())
    {
      char input = BTSERIAL.read();
      
      switch (input)
      {
        // stop motors and reset
        case 'q':                      
        {
          noInterrupts();
          esc0.write(0);
          esc1.write(0);
          startup = false;
          digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate reset
          set_mode(IDLE);
          BTSERIAL.println("You've quit. Press any key to start up again");
          delay(1000);
          // BTserial_clear();
          interrupts();
          break;
        }

        // read and print current yaw (deg)
        case 'r':                      
        {
          noInterrupts();
          delay(1000);
          unsigned short out_counter = 1;  // counter for when to print
          set_mode(READ);
          BTSERIAL.println("You're in READ mode");
          delay(1000);
          BTserial_clear();
          interrupts();
          while (get_mode() == READ)
          {
            out_counter = out_counter + 1;
            if (out_counter % PRINTAFTER == 0)
            {
              BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
            }
          }
          BTSERIAL.println("You're in IDLE mode");
          delay(1000);
          break;
        }

        // manually change motor speeds
        case 'm':
        {
          noInterrupts();
          delay(1000);
          BTSERIAL.println("Enter motor speeds separated by whitespace:");
          BTserial_block();
          vals[0] = BTSERIAL.parseInt();
          vals[1] = BTSERIAL.parseInt();
          BTSERIAL.println("Speeds:");
          BTSERIAL.print(vals[0]);
          BTSERIAL.print(" ");
          BTSERIAL.println(vals[1]);
          esc0.write(vals[0]);
          esc1.write(vals[1]);
          set_mode(IDLE);
          BTSERIAL.println("You're in IDLE mode");
          delay(1000);
          interrupts();
          break;
        }

        // go to target yaw and hold
        case 'h':
        {
          noInterrupts();
          delay(1000);
          unsigned short out_counter = 1;  // counter for when to print
          e_yaw_prev = 0;
          Eint = 0;
          control_sig = 0;
          BTSERIAL.println("Enter desired yaw change (deg):");
          BTserial_block();
          int yawChange = BTSERIAL.parseInt();
          yawNow = read_yaw();                   
          yawTarget = correct_yaw(yawNow + yawChange);          
          BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
          BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
          set_mode(HOLD);
          BTSERIAL.println("You're in HOLD mode");
          delay(1000);
          interrupts();
          while (get_mode() == HOLD)
          {
            out_counter = out_counter + 1;
            if (out_counter % PRINTAFTER == 0)
            {
              // BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
              // BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
              BTSERIAL.println(Eint);
            }
          }
          BTSERIAL.println("You're in IDLE mode");
          delay(1000);
          break;
        }

        case 'f':                      // Follow reference trajectory
        {
          noInterrupts();
          delay(1000);

          // reset PID parameters
          e_yaw_prev = 0;
          Eint = 0;
          control_sig = 0;
          
          // take reference trajectory from user
          int refTimes[3], refAngs[3];
          refTimes[0] = 0;
          BTSERIAL.println("Time 1 is 0. Enter time 2:");
          BTserial_block();
          refTimes[1] = BTSERIAL.parseInt();
          BTSERIAL.println("Enter time 3:");
          BTserial_block();
          refTimes[2] = BTSERIAL.parseInt();
          BTSERIAL.println("Target times:");
          for (byte i = 0; i < 3; i++)
          {
            BTSERIAL.println(refTimes[i]);
          }
          BTSERIAL.println("Enter angle 1:");
          BTserial_block();
          refAngs[0] = BTSERIAL.parseInt();
          BTSERIAL.println("Enter angle 2:");
          BTserial_block();
          refAngs[1] = BTSERIAL.parseInt();
          BTSERIAL.println("Enter angle 3:");
          BTserial_block();
          refAngs[2] = BTSERIAL.parseInt();
          BTSERIAL.println("Target angles:");
          for (byte i = 0; i < 3; i++)
          {
            BTSERIAL.println(refAngs[i]);
          }
          BTSERIAL.println();

          // create array of reference angles indexed by sample number
          genRef(REFtraj, refTimes, refAngs);
          
          // track, then hold:
          set_mode(TRACK);
          BTSERIAL.println("You're in TRACK mode");
          unsigned short out_counter = 1;  // counter for when to print
          delay(1000);
          interrupts();
          while (get_mode() == TRACK)
          {
            out_counter = out_counter + 1;
            if (out_counter % PRINTAFTER == 0)
            {
              BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
              BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
            }
          }
          
          out_counter = 1;
          while (get_mode() == HOLD)
          {
            out_counter = out_counter + 1;
            if (out_counter % PRINTAFTER == 0)
            {
              BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
              BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
            }
          }
          BTSERIAL.println("You're in IDLE mode");
          delay(1000);
          break;
        }

        case 't':                      // Tune gains from disturbance response
        {
          noInterrupts();
          delay(1000);
          e_yaw_prev = 0;
          Eint = 0;
          control_sig = 0;
          yawNow = read_yaw();
          int yawStart = yawNow;                   
          yawTarget = correct_yaw(yawNow + 180);          
          set_mode(HOLD);
          BTSERIAL.println("You're in HOLD (tune) mode");
          delay(1000);
          interrupts();
          delay(1);
          yawTarget = yawStart;
          while (get_mode() == HOLD)
          {
            ;
          }
          BTSERIAL.println("You're in IDLE mode");
          delay(1000);
          break;
        }

        default:
        {
          digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
          break;
        }
      }
    }
  }
}
