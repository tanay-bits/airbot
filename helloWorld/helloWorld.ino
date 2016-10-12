#include <Servo.h>

Servo esc1, esc2;
int vals[2] = {0, 0};

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("This program will arm the ESC.");

  esc1.attach(9);
  esc2.attach(16);

  Serial.println("Now writing 0.");
}

void loop()
{
  esc1.write(vals[0]);
  esc2.write(vals[1]);
  if (Serial.available())
  {
      vals[0] = Serial.parseInt();
      vals[1] = Serial.parseInt();
      Serial.println("Vals: ");
      Serial.print(vals[0]);
      Serial.print(" ");
      Serial.println(vals[1]);
//      delay(1);
  }
}
