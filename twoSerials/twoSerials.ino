#include <Servo.h>

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

Servo esc1, esc2;
int vals[2] = {0, 0};
boolean startup = false;
boolean synched = false;

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

void setup() {
  // initialize serial channels:
  Serial.begin(9600);
  HWSERIAL.begin(58824);
  delay(1000);
  
  // arm the ESC's:
  esc1.attach(16);
  esc2.attach(17);
  esc1.write(vals[0]);
  esc2.write(vals[1]);
  
  delay(3000);            // give enough time for Razor to auto-reset
  
  // set Razor output parameters:
  HWSERIAL.write("#ob");  // Turn on binary output
  HWSERIAL.write("#o1");  // Turn on continuous streaming output
  HWSERIAL.write("#oe0"); // Disable error message output

  // synch with Razor:
  while(!synched)
  {
    synched = readToken("#SYNCH00\r\n");  // look for synch token
  }
  
//  Serial.println("Enter any key except 't' to start, then enter 't' to tune motor speeds");
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
        esc1.write(vals[0]);
        esc2.write(vals[1]);
        startup = true;
        delay(200);
      }
    }
    if (Serial.read()=='t')
    {
      Serial.println("Enter motor speeds separated by whitespace:");
      while (!Serial.available())
      {
        ;
      }
      vals[0] = Serial.parseInt();
      vals[1] = Serial.parseInt();
      Serial.println("Vals: ");
      Serial.print(vals[0]);
      Serial.print(" ");
      Serial.println(vals[1]);
      esc1.write(vals[0]);
      esc2.write(vals[1]);
      delay(10);
    }
    synched = false;
    // synch with Razor:
    while(!synched)
    {
      synched = readToken("#SYNCH00\r\n");  // look for synch token
    }
  }
  
  union u_tag {
    byte b_angles[12];
    float f_angles[3];
  } u;        
  if (HWSERIAL.available() >= 12) {
    for (int i = 0; i < 12; i++)
    {
      u.b_angles[i] = HWSERIAL.read();    
    }
    Serial.print("YPR received: ");
    Serial.print(u.f_angles[0]); Serial.print(", ");
    Serial.print(u.f_angles[1]); Serial.print(", ");
    Serial.println(u.f_angles[2]);
  }
}
