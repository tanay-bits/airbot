#define SIG_A 4
#define SIG_B 5
#define SIG_C 6
#define H_HIT 10
#define V_HIT 20

const float deg_per_us = 0.0216;	// 180 deg / 8333 us

volatile byte flag = 0;				// 0 (no laser pulse) or H_HIT or V_HIT
volatile boolean pulse = false;
volatile boolean state_now, state_prev;
volatile unsigned long time_now, time_prev, delta, ts, tl, sync_to_laser;
volatile char pulse_type_now = '?', pulse_type_prev = '?';	// 'S' (sync) or 'H' (horizontal laser) or 'V' (vertical laser)

void isr_lighthouse() {
	time_now = micros();
	state_now = digitalReadFast(SIG_B);

	if ((state_now == HIGH) && (state_prev == LOW))
	{
		pulse = true;
		delta = time_now - time_prev;
		if (delta > 50)
		{
			pulse_type_now = 'S';
			ts = time_prev;

			// reset other stuff
			tl = 0;
			sync_to_laser = 0;
			flag = 0;
		}
		else if (delta > 15)
		{
			pulse_type_now = 'H';
			tl = time_prev;
			if (pulse_type_prev == 'S')
			{
				sync_to_laser = tl - ts;
				flag = H_HIT;
			}
		}
		else if (delta > 7)
		{
			pulse_type_now = 'V';
			tl = time_prev;
			if (pulse_type_prev == 'S')
			{
				sync_to_laser = tl - ts;
				flag = V_HIT;
			}
		}
		else
		{
			pulse_type_now = '?';
			
			// reset stuff
			ts = 0;
			tl = 0;
			sync_to_laser = 0;
			flag = 0;
		}
	}

	state_prev = state_now;
	time_prev = time_now;
	pulse_type_prev = pulse_type_now;
}

void setup() {
	pinMode(SIG_B, INPUT);
	Serial.begin(115200);
	attachInterrupt(digitalPinToInterrupt(SIG_B), isr_lighthouse, CHANGE);
}

void loop() {
	if (flag)
	{
		float angle = sync_to_laser * deg_per_us;
		
		if (flag == H_HIT)
		{
			Serial.print("H angle: "); Serial.println(angle);
		}
		else if (flag == V_HIT)
		{
			Serial.print("V angle: "); Serial.println(angle);
		}
		else {Serial.println("ERROR!");}
		
		flag = 0;
	}
}
