/***
 * Positive yaw change -> CW robot rotation
 ***/

#include <Servo.h>

// Set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

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
  HWSERIAL.clear();       // Clear input buffer up to here
  HWSERIAL.write("#s00"); // Request synch token
  delay(500);
  // Check if incoming bytes match token
  for (int i = 0; i < (sizeof(token) - (unsigned)1); i++)
    {
      if (HWSERIAL.read() != token[i])
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
  // Initialize serial channels:
  Serial.begin(9600);
  HWSERIAL.begin(58824);
  delay(1000);
  
  // Arm the ESC's:
  esc0.attach(16);
  esc1.attach(17);
  esc0.write(vals[0]);
  esc1.write(vals[1]);
  
  delay(3000);            // Give enough time for Razor to auto-reset
  
  // Set Razor output parameters:
  HWSERIAL.write("#ob");  // Turn on binary output
  HWSERIAL.write("#o1");  // Turn on continuous streaming output
  HWSERIAL.write("#oe0"); // Disable error message output

  // Synch with Razor:
  while(!synched)
  {
    synched = readToken("#SYNCH00\r\n");  // look for synch token
  }
  
//  Serial.println("Enter 't' or 'y' ");
}

void loop() {
  if (Serial.available())
  {
    if (!startup)
    {
      for (int i=10; i < 75; i = i+5)
      {
        vals[0] = i;
        vals[1] = i;
        esc0.write(vals[0]);
        esc1.write(vals[1]);
        startup = true;
        delay(200);
      }
    }
    
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
      HWSERIAL.write("#o0");  // Turn off continuous streaming output
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
      
//      // Synch with Razor before reading yaw again:
//      synched = false;
//      while(!synched)
//      {
//        synched = readToken("#SYNCH00\r\n");  // Look for synch token
//      }
      
      // CONTROL LOOP:
      while (abs(yawTarget - yawNow) > yawTol)
      {
        wDel = (Kp/Awy) * (yawTarget - yawNow);   // Delta of w
        vals[0] = saturateSpeed(vals[0] - wDel);  // New speed for motor 0
        vals[1] = saturateSpeed(vals[1] + wDel);  // New speed for motor 1
        esc0.write(vals[0]);
        esc1.write(vals[1]);

        HWSERIAL.write("#f");  // Request one output frame
        delay(100);            // precautionary        
        if (HWSERIAL.available() >= 12)
        {
          for (int i = 0; i < 12; i++)
          {
            u.b_angles[i] = HWSERIAL.read();    
          }
          yawNow = u.f_angles[0];
          Serial.print("Current yaw: "); Serial.println(yawNow);
          Serial.print("Target yaw: "); Serial.println(yawTarget);
        }
        else {Serial.println("COULD NOT READ YAW");}        
      }
    }

    // Synch with Razor again:
    HWSERIAL.write("#o1");  // Turn on continuous streaming output
    synched = false;
    while(!synched)
    {
      synched = readToken("#SYNCH00\r\n");  // Look for synch token
    }
  }
        
  if (HWSERIAL.available() >= 12)
  {
    for (int i = 0; i < 12; i++)
    {
      u.b_angles[i] = HWSERIAL.read();    
    }
    yawNow = u.f_angles[0];
    Serial.print("YPR received: ");
    Serial.print(u.f_angles[0]); Serial.print(", ");
    Serial.print(u.f_angles[1]); Serial.print(", ");
    Serial.println(u.f_angles[2]);
  }
}
