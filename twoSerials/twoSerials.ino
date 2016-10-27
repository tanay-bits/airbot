#include <Servo.h>

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

Servo esc1, esc2;
int vals[2] = {0, 0};
bool STARTUP = false;

void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(58824);
  delay(1000);
  esc1.attach(16);
  esc2.attach(17);
  // arm the ESC's:
  esc1.write(vals[0]);
  esc2.write(vals[1]);
  delay(3000);
  Serial.println("Enter any key except 't' to start, then enter 't' to tune motor speeds");
}

void loop() {
  if (Serial.available())
  {
    if (!STARTUP)
    {
      for (int i=10; i < 75; i = i+5)
      {
        vals[0] = i;
        vals[1] = i;
        esc1.write(vals[0]);
        esc2.write(vals[1]);
        STARTUP = true;
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
  }
  
  int incomingByte;
        
  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
    HWSERIAL.print("UART received:");
    HWSERIAL.println(incomingByte, DEC);
  }
}
