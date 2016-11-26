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
const int MAX_SPEED = 160;    // max ESC write value
const int CONTROL_PERIOD_MS = 22;
const int CONTROL_FREQ = 45;  // approximate control frequency (Hz)


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
//      yawNow = read_yaw();
//      yawTarget = REFtraj[ctr];
//      e_yaw = calc_error(yawTarget - yawNow);
//      Eint = Eint + e_yaw;
//
//      // calculate control signal and send to actuator, if error outside deadband
//      calc_send_control(e_yaw);
//
//      // update previous error for derivative calculation
//      e_yaw_prev = e_yaw;
//
//      ctr++;
//      if (ctr == refSize)
//      {
//        yawTarget = REFtraj[refSize-1];  // set last ref as target for HOLD
//        ctr = 0;  // reset counter
//        set_mode(HOLD);
//      }
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
      startup = true;
      BTSERIAL.println("Start up successful");
      BTSERIAL.println();
      digitalWrite(LEDPIN, LOW);  // turn off LED to indicate startup
      BTserial_clear();  // Empty the input buffer
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
          BTSERIAL.println("You've quit. Press any key to start up again.");
          BTSERIAL.println();
          delay(1000);
          
          interrupts();
          break;
        }

        // Equilibrium mode (both motors ramp up to MAX_SPEED/2)
        case 'e':
        {         
          for (int i=10; i <= MAX_SPEED/2; i = i+1)
          {
            vals[0] = i;
            vals[1] = i;
            esc0.write(vals[0]);
            esc1.write(vals[1]);
//            delay(50);  // to slow down the ramp-up
          }
          BTSERIAL.println("Ramp-up to equilibrium complete.");
          BTSERIAL.println();   
          break;
        }
        
        // get parameters - PID gains, yawTol, EINT_CAP, current mode
        case '?':                      
        {
          noInterrupts();
          
          BTSERIAL.println("PID gains are:");
          BTSERIAL.println(Kp);
          BTSERIAL.println(Ki);
          BTSERIAL.println(Kd);
          BTSERIAL.println();

          BTSERIAL.println("yawTol is:");
          BTSERIAL.println(yawTol);
          BTSERIAL.println();

          BTSERIAL.println("EINT_CAP is:");
          BTSERIAL.println(EINT_CAP);
          BTSERIAL.println();

          BTSERIAL.print("You are in ");
          switch (get_mode())
          {
            case IDLE:
            {
              BTSERIAL.println("IDLE mode");
              break;
            }
            case READ:
            {
              BTSERIAL.println("READ mode");
              break;
            }
            case HOLD:
            {
              BTSERIAL.println("HOLD mode");
              break;
            }
            case TRACK:
            {
              BTSERIAL.println("TRACK mode");
              break;
            }
            default:
            {
              BTSERIAL.println("no valid mode!??");
              break; 
            }
          }
          BTSERIAL.println();
          
          interrupts();
          break;
        }

        // set PID gains
        case 's':                      
        {
          noInterrupts();
          delay(1000);

          BTSERIAL.println("Enter P gain:");
          BTserial_block();
          Kp = BTSERIAL.parseFloat();

          BTSERIAL.println("Enter I gain:");
          BTserial_block();
          Ki = BTSERIAL.parseFloat();

          BTSERIAL.println("Enter D gain:");
          BTserial_block();
          Kd = BTSERIAL.parseFloat();

          BTSERIAL.println("New gains are:");
          BTSERIAL.println(Kp);
          BTSERIAL.println(Ki);
          BTSERIAL.println(Kd);
          BTSERIAL.println();
          
          interrupts();
          break;
        }

        // set yawTol
        case 'b':                      
        {
          noInterrupts();
          delay(1000);

          BTSERIAL.println("Enter desired yawTol:");
          BTserial_block();
          yawTol = BTSERIAL.parseInt();
          BTSERIAL.println("New yawTol is:");
          BTSERIAL.println(yawTol);
          BTSERIAL.println();
          
          interrupts();
          break;
        }

        // set EINT_CAP for triggering retraction
        case 'c':                      
        {
          noInterrupts();
          delay(1000);

          BTSERIAL.println("Enter desired EINT_CAP:");
          BTserial_block();
          EINT_CAP = BTSERIAL.parseInt();
          BTSERIAL.println("New EINT_CAP is:");
          BTSERIAL.println(EINT_CAP);
          BTSERIAL.println();
          
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
          BTSERIAL.println();
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
          BTSERIAL.println();
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
          BTSERIAL.println();
          delay(1000);
          break;
        }

//        case 'f':                      // Follow reference trajectory
//        {
//          noInterrupts();
//          delay(1000);
//
//          // reset PID parameters
//          e_yaw_prev = 0;
//          Eint = 0;
//          control_sig = 0;
//          
//          // take reference trajectory from user
//          int refTimes[3], refAngs[3];
//          refTimes[0] = 0;
//          BTSERIAL.println("Time 1 is 0. Enter time 2:");
//          BTserial_block();
//          refTimes[1] = BTSERIAL.parseInt();
//          BTSERIAL.println("Enter time 3:");
//          BTserial_block();
//          refTimes[2] = BTSERIAL.parseInt();
//          BTSERIAL.println("Target times:");
//          for (byte i = 0; i < 3; i++)
//          {
//            BTSERIAL.println(refTimes[i]);
//          }
//          BTSERIAL.println("Enter angle 1:");
//          BTserial_block();
//          refAngs[0] = BTSERIAL.parseInt();
//          BTSERIAL.println("Enter angle 2:");
//          BTserial_block();
//          refAngs[1] = BTSERIAL.parseInt();
//          BTSERIAL.println("Enter angle 3:");
//          BTserial_block();
//          refAngs[2] = BTSERIAL.parseInt();
//          BTSERIAL.println("Target angles:");
//          for (byte i = 0; i < 3; i++)
//          {
//            BTSERIAL.println(refAngs[i]);
//          }
//          BTSERIAL.println();
//
//          // create array of reference angles indexed by sample number
//          genRef(REFtraj, refTimes, refAngs);
//          
//          // track, then hold:
//          set_mode(TRACK);
//          BTSERIAL.println("You're in TRACK mode");
//          unsigned short out_counter = 1;  // counter for when to print
//          delay(1000);
//          interrupts();
//          while (get_mode() == TRACK)
//          {
//            out_counter = out_counter + 1;
//            if (out_counter % PRINTAFTER == 0)
//            {
//              BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
//              BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
//            }
//          }
//          
//          out_counter = 1;
//          while (get_mode() == HOLD)
//          {
//            out_counter = out_counter + 1;
//            if (out_counter % PRINTAFTER == 0)
//            {
//              BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
//              BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
//            }
//          }
//          BTSERIAL.println("You're in IDLE mode");
//          delay(1000);
//          break;
//        }

        case 't':                      // Tune gains from disturbance response
        {
          noInterrupts();
          delay(1000);
          unsigned short out_counter = 1;  // counter for when to print
          e_yaw_prev = 0;
          Eint = 0;
          control_sig = 0;
          yawNow = read_yaw();
          yawTarget = yawNow;                   
          set_mode(HOLD);
          BTSERIAL.println("You're in HOLD (tune) mode. Disturb and observe.");
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

        default:
        {
          digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
          break;
        }
      }
    }
  }
}
