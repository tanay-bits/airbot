/***
 * Positive yaw change -> CW robot rotation
 ***/

///////////////////////////
// IMPORTS AND CONSTANTS //
///////////////////////////

#include <Servo.h>
#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define PRINTAFTER 100
#define LEDPIN 13


/////////////
// GLOBALS //
/////////////

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
volatile float yawTol = 10;                        // yaw error tolerance
volatile float yawNow = 0;                         // current (sensed) yaw
volatile int yawTarget = 0;                        // target yaw position
volatile float Kp = 1, Ki = 0, Kd = 0, Awy = 1;    // yawdot = Awy * (wu - wnom)
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_prev = 0;                           // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)


//////////////////////
// HELPER FUNCTIONS //
//////////////////////

float read_yaw(void) {
  IMUserial_clear();  // Clear input buffer
  IMUSERIAL.write("#f");  // Request one output frame
  while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
  for (int i = 0; i < 4; i++)  // Read yaw angle
  {
    u.b_angle[i] = IMUSERIAL.read();    
  }
  return u.f_angle[0];
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

// Limits motor speeds to 0-150
int saturateSpeed(float s) {
  if (s > 150) {return 150;}
  else if (s < 0) {return 0;}
  else {return s;}
}

// Corrects for overflow in target yaw calculation
int valid_yaw(int x) {
  if (x >= 180) {
    int overflow = x - 180;
    x = -(180 - overflow);
  }
  else if (x <= -180) {
    int overflow = x + 180;
    x = 180 - overflow;
  }
  return x;
}


///////////////////////
// INTERRUPT ROUTINE //
///////////////////////

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
      e_yaw = yawTarget - yawNow;
      Eint = Eint + e_yaw;

      // calculate control signal and send to actuator
      control_sig = (Kp/Awy) * e_yaw;  // ~ motor speed change
      vals[0] = saturateSpeed(vals[0] - control_sig);  // new speed for motor 0
      vals[1] = saturateSpeed(vals[1] + control_sig);  // new speed for motor 1
      esc0.write(vals[0]);
      esc1.write(vals[1]);

      // update previous error for integral calculation
      e_prev = e_yaw;

      if (BTSERIAL.available())
      {
        set_mode(IDLE);
      }
      break;
    }

    case TRACK:
    {
      // yawNow = read_yaw();
      // yawTarget = REFtraj[ctr];
      // e_yaw = yawTarget - yawNow;
      // Eint = Eint + e_yaw;

      // control_sig = (Kp/Awy) * e_yaw;  // ~ motor speed change
      // vals[0] = saturateSpeed(vals[0] - control_sig);  // new speed for motor 0
      // vals[1] = saturateSpeed(vals[1] + control_sig);  // new speed for motor 1
      // esc0.write(vals[0]);
      // esc1.write(vals[1]);

      // // Store data for MATLAB:
      // SENtraj[ctr] = sensed_ang;

      // ctr++;
      // if (ctr == num_samples){
      //   ang_target = REFtraj[num_samples-1];  // set last ref as target for HOLD
      //   ctr = 0;                              // reset counter
      //   set_mode(HOLD);
      // }

      // ctrl_counter += 1;
      // if (ctrl_counter % PRINTAFTER == 0)
      // {
      //   BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
      //   BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
      // }

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
  myTimer.begin(controller, 30000);  // controller to run every 30 ms TODO: CHANGE TIMING?
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
          unsigned int out_counter = 0;  // counter for when to print
          set_mode(READ);
          BTSERIAL.println("You're in READ mode");
          delay(1000);
          BTserial_clear();
          interrupts();
          while (get_mode() == READ)
          {
            out_counter += 1;
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
          unsigned int out_counter = 0;  // counter for when to print
          e_prev = 0;
          Eint = 0;
          control_sig = 0;
          BTSERIAL.println("Enter desired yaw change (deg):");
          BTserial_block();
          int yawChange = BTSERIAL.parseInt();
          yawNow = read_yaw();                   
          yawTarget = valid_yaw(yawNow + yawChange);          
          BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
          BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
          set_mode(HOLD);
          BTSERIAL.println("You're in HOLD mode");
          delay(1000);
          interrupts();
          while (get_mode() == HOLD)
          {
            out_counter += 1;
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

        // case 't':                      // track a trajectory
        // {
        //   set_mode(TRACK);
        //   break;
        // }

        default:
        {
          digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
          break;
        }
      }
    }
  }
}
