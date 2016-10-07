#include <Servo.h>

Servo myESC;
int val;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("This program will arm the ESC.");

  myESC.attach(9);

  Serial.println("Now writing 0.");
  val = 0;
}

void loop()
{
  myESC.write(val);
  if (Serial.available())
  {
      val = Serial.parseInt();
      Serial.println("Val: ");
      Serial.println(val);
//      delay(1);
  }
}
