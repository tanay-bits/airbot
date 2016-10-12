#include <Servo.h>

Servo esc1, esc2;
int vals[2] = {0, 0};
bool STARTUP = false;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("This program will arm the ESC.");

  esc1.attach(9);
  esc2.attach(16);

  Serial.println("Now writing 0 0");
  esc1.write(vals[0]);
  esc2.write(vals[1]);
  delay(3000);
  Serial.println("Enter any key except 't' to start, then enter 't' to tune motor speeds");
}

void loop()
{
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
}
