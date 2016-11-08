/***
 * Positive yaw change -> CW robot rotation
 ***/

/////////////////////////////
// IMPORTS AND DEFINITIONS //
/////////////////////////////

#include <Servo.h>
#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define PRINTAFTER 20


/////////////
// GLOBALS //
/////////////

Servo esc0, esc1;                         // esc0 - CW wheel, esc1 - CCW wheel 
bool startup = false;
bool synched = false;
float yawTol = 10, yawNow;
float Kp = 1, Ki = 0, Kd = 0, Awy = 1;    // yawdot = Awy * (wu - wnom)
int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)
unsigned short out_counter = 0;           // counter for when to print

// Union data structure to save incoming byte array as floats representing angles (deg):
union u_tag {
  byte b_angles[4];
  float f_angles[1];
} u; 


//////////////////////
// HELPER FUNCTIONS //
//////////////////////

// Only needed if continuous output streaming is on:
bool readToken(char* token) {
  while (IMUSERIAL.available()) {IMUSERIAL.read();} // Clear input buffer
  
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

int saturateSpeed(float s) {
  if (s > 150) {return 150;}
  else if (s < 0) {return 0;}
  else {return s;}
}


////////////////////
// SETUP AND LOOP //
////////////////////

void setup() {
  delay(3000);  // Give enough time for Razor to auto-reset
  
  // Initialize serial channels:
  BTSERIAL.begin(9600);
//  IMUSERIAL.begin(58824);
  IMUSERIAL.begin(57600);
  delay(1000);
  
  // Arm the ESC's:
  esc0.attach(16);
  esc1.attach(17);
  esc0.write(vals[0]);
  esc1.write(vals[1]);
  
  // Set Razor output parameters:
  IMUSERIAL.write("#ob");  // Turn on binary output
  IMUSERIAL.write("#o0");  // Turn OFF continuous streaming output
  IMUSERIAL.write("#oe0"); // Disable error message output 
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
      while (BTSERIAL.available()) {BTSERIAL.read();}  // Empty the input buffer
      delay(200);    
    }
  }

  if (startup)
  {   
    if (BTSERIAL.available())
    {
      char mode = BTSERIAL.read();
      
      if (mode == 't')
      {
        BTSERIAL.println("Enter motor speeds separated by whitespace:");
        while (!BTSERIAL.available()) {;}  // Block
        vals[0] = BTSERIAL.parseInt();
        vals[1] = BTSERIAL.parseInt();
        BTSERIAL.println("Vals: ");
        BTSERIAL.print(vals[0]);
        BTSERIAL.print(" ");
        BTSERIAL.println(vals[1]);
        esc0.write(vals[0]);
        esc1.write(vals[1]);
        while (BTSERIAL.available()) {BTSERIAL.read();}  // Empty the input buffer
        delay(10);
      }
      
      else if (mode == 'y')
      {        
        BTSERIAL.println("Enter desired yaw change (deg):");
        while (!BTSERIAL.available()) {;}  // Block
        int yawChange = BTSERIAL.parseInt();

        while (IMUSERIAL.available()) {IMUSERIAL.read();}  // Clear input buffer
        IMUSERIAL.write("#f");  // Request one output frame
        while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
        for (int i = 0; i < 4; i++)  // Read yaw angle
        {
          u.b_angles[i] = IMUSERIAL.read();    
        }
        yawNow = u.f_angles[0];
                 
        int yawTarget = yawNow + yawChange;
        BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
        BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);

        float wDel;
        unsigned short ctrl_counter = 0;
        // CONTROL LOOP:
        while (abs(yawTarget - yawNow) > yawTol)
        {
          wDel = (Kp/Awy) * (yawTarget - yawNow);   // Delta of w
          vals[0] = saturateSpeed(vals[0] - wDel);  // New speed for motor 0
          vals[1] = saturateSpeed(vals[1] + wDel);  // New speed for motor 1
          esc0.write(vals[0]);
          esc1.write(vals[1]);

          while (IMUSERIAL.available()) {IMUSERIAL.read();}  // Clear input buffer
          IMUSERIAL.write("#f");  // Request one output frame
          while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
          for (int i = 0; i < 4; i++)
          {
            u.b_angles[i] = IMUSERIAL.read();    
          }
          yawNow = u.f_angles[0];

          ctrl_counter += 1;
          if (ctrl_counter % PRINTAFTER == 0)
          {
            BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
            BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
          }
        }        
      }
    }

    while (IMUSERIAL.available()) {IMUSERIAL.read();}  // Clear input buffer
    IMUSERIAL.write("#f");  // Request one output frame
    while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
    for (int i = 0; i < 4; i++)  // Read yaw angle
    {
      u.b_angles[i] = IMUSERIAL.read();    
    }
    yawNow = u.f_angles[0];
    
    out_counter += 1;
    if (out_counter % PRINTAFTER == 0)
    {
      BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
    }
  }
}

