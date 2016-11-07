/***
 * Positive yaw change -> CW robot rotation
 ***/

#include <Servo.h>

// Set this to the hardware serial port you wish to use
#define IMUSERIAL Serial1

Servo esc0, esc1;         // esc0 - CW wheel, esc1 - CCW wheel 
boolean startup = false;
boolean synched = false;
float yawTol = 10, yawNow;
float Kp = 1, Ki = 0, Kd = 0, Awy = 1;    // yawdot = Awy * (wu - wnom)
int vals[2] = {0, 0};     // w0 and w1 (motor speeds)

// Union data struction to save incoming byte array as floats representing angles (deg):
union u_tag {
  byte b_angles[12];
  float f_angles[3];
} u; 


boolean readToken(char* token) {
  // Clear input buffer:
//  IMUSERIAL.clear();       
  while (IMUSERIAL.available()) {IMUSERIAL.read();}
  
  IMUSERIAL.write("#s00"); // Request synch token
  delay(500);
  // Check if incoming bytes match token
  for (int i = 0; i < (sizeof(token) - (unsigned)1); i++)
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

void setup() {
  delay(3000);  // Give enough time for Razor to auto-reset
  
  // Initialize serial channels:
  Serial.begin(9600);
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
  IMUSERIAL.write("#o1");  // Turn on continuous streaming output
  IMUSERIAL.write("#oe0"); // Disable error message output 
}

void loop() {
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
    // Synch with Razor:
    while(!synched)
    {
      synched = readToken("#SYNCH00\r\n");  // look for synch token
    }
    
    if (Serial.available())
    {
      char mode = Serial.read();
      
      if (mode == 't')
      {
        Serial.println("Enter motor speeds separated by whitespace:");
        while (!Serial.available())
        {
          ;                     // Do nothing
        }
        vals[0] = Serial.parseInt();
        vals[1] = Serial.parseInt();
        Serial.println("Vals: ");
        Serial.print(vals[0]);
        Serial.print(" ");
        Serial.println(vals[1]);
        esc0.write(vals[0]);
        esc1.write(vals[1]);
        delay(10);
      }
      
      else if (mode == 'y')
      {
        IMUSERIAL.write("#o0");  // Turn off continuous streaming output
        delay(100);             // precautionary
        
        Serial.println("Enter desired yaw change (deg):");
        while (!Serial.available())
        {
          ;                     // Do nothing
        }
        int yawChange = Serial.parseInt();
        int yawTarget = yawNow + yawChange;
        Serial.print("Current yaw: "); Serial.println(yawNow);
        Serial.print("Target yaw: "); Serial.println(yawTarget);
        float wDel;
        
        // CONTROL LOOP:
        while (abs(yawTarget - yawNow) > yawTol)
        {
          wDel = (Kp/Awy) * (yawTarget - yawNow);   // Delta of w
          vals[0] = saturateSpeed(vals[0] - wDel);  // New speed for motor 0
          vals[1] = saturateSpeed(vals[1] + wDel);  // New speed for motor 1
          esc0.write(vals[0]);
          esc1.write(vals[1]);
  
          IMUSERIAL.write("#f");  // Request one output frame
          delay(100);             // precautionary        
          if (IMUSERIAL.available() >= 12)
          {
            for (int i = 0; i < 12; i++)
            {
              u.b_angles[i] = IMUSERIAL.read();    
            }
            yawNow = u.f_angles[0];
            Serial.print("Current yaw: "); Serial.println(yawNow);
            Serial.print("Target yaw: "); Serial.println(yawTarget);
          }
          else {Serial.println("COULD NOT READ YAW");}        
        }
      }
  
      // Synch with Razor again:
      IMUSERIAL.write("#o1");  // Turn on continuous streaming output
      synched = false;
      while(!synched)
      {
        synched = readToken("#SYNCH00\r\n");  // Look for synch token
      }
    }
          
    if (IMUSERIAL.available() >= 12)
    {
      for (int i = 0; i < 12; i++)
      {
        u.b_angles[i] = IMUSERIAL.read();    
      }
      yawNow = u.f_angles[0];
      Serial.print("YPR received: ");
      Serial.print(u.f_angles[0]); Serial.print(", ");
      Serial.print(u.f_angles[1]); Serial.print(", ");
      Serial.println(u.f_angles[2]);
    }
  }
}
