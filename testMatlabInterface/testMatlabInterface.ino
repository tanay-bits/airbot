#define BTSERIAL Serial2

void setup() {
	BTSERIAL.begin(9600);
}

void loop() {
	// keep polling for user input
	if (BTSERIAL.available())
	{
		char input = BTSERIAL.read();
		switch (input)
		{
			case 'l':
			{
				float f_val = 42.01;
				char str[10];
				dtostrf(f_val, 4, 2, str);
				// sprintf(str, "%f", val);

				// print or write?
				BTSERIAL.write(f_val);
				BTSERIAL.write("\r\n");

				// float f2_val = 1.00;
				// dtostrf(f2_val, 4, 2, str);

				// BTSERIAL.write(str);
				// BTSERIAL.write("\r\n");

				break;
			}

			default:
			{
				break;
			}
		}

	}
}
