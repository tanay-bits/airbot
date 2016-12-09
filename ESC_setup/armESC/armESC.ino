#include <Servo.h>

Servo myESC;
int val;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("This program will arm the ESC.");

  myESC.attach(16);

  Serial.println("Now writing 0.");
  val = 0;
  myESC.write(val);
}

void loop()
{
  if (Serial.available())
  {
      val = Serial.parseInt();
      Serial.println("Val: ");
      Serial.println(val);
      myESC.write(val);
  }
}
