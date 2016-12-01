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
			case 'a':
			{
				int REFtraji = 34;
				int SENtraji = 449;
				char str[15];
				sprintf(str, "%d %d\r\n", REFtraji, SENtraji);

				BTSERIAL.write(str);


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
