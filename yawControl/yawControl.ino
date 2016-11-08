/***
 * Positive yaw change -> CW robot rotation
 ***/

/////////////////////////////
// IMPORTS AND DEFINITIONS //
/////////////////////////////

#include <Servo.h>
#define IMUSERIAL Serial1
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
  Serial.begin(9600);
  IMUSERIAL.begin(58824);
  // IMUSERIAL.begin(57600);
  delay(500);
  
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
    if (Serial.available())
    {
      for (int i=10; i < 75; i = i+5)
      {
        vals[0] = i;
        vals[1] = i;
        esc0.write(vals[0]);
        esc1.write(vals[1]);
      }
      startup = true;
      Serial.println("Startup successful");
      while (Serial.available()) {Serial.read();}  // Empty the input buffer
      delay(200);    
    }
  }

  if (startup)
  {    
    if (Serial.available())
    {
      char mode = Serial.read();
      
      if (mode == 't')
      {
        Serial.println("Enter motor speeds separated by whitespace:");
        while (!Serial.available()) {;}  // Block
        vals[0] = Serial.parseInt();
        vals[1] = Serial.parseInt();
        Serial.println("Vals: ");
        Serial.print(vals[0]);
        Serial.print(" ");
        Serial.println(vals[1]);
        esc0.write(vals[0]);
        esc1.write(vals[1]);
        while (Serial.available()) {Serial.read();}  // Empty the input buffer
        delay(10);
      }
      
      else if (mode == 'y')
      {        
        Serial.println("Enter desired yaw change (deg):");
        while (!Serial.available()) {;}  // Block
        int yawChange = Serial.parseInt();

        while (IMUSERIAL.available()) {IMUSERIAL.read();}  // Clear input buffer
        IMUSERIAL.write("#f");  // Request one output frame
        while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
        for (int i = 0; i < 4; i++)  // Read yaw angle
        {
          u.b_angles[i] = IMUSERIAL.read();    
        }
        yawNow = u.f_angles[0];
                 
        int yawTarget = yawNow + yawChange;
        Serial.print("Current yaw: "); Serial.println(yawNow);
        Serial.print("Target yaw: "); Serial.println(yawTarget);

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
