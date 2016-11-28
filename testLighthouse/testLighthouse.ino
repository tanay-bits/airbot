#define NUM_SENSORS 3
#define BUFLEN 256
#define PRINTAFTER 200000
#define SIG 4

const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)
unsigned long ctr = 0;	// counter for printing

// struct DataPacket
// {
// 	unsigned long timeStamp;
// 	bool state;

// 	DataPacket()
// 	{
// 		state = HIGH;
// 		timeStamp = 0;
// 	}
// };

class RingBuf
{
	private:
		volatile short readPos;						// index to read at
		volatile short writePos;					// index to write at
		volatile unsigned long timeStamps[BUFLEN];	// array to store time-stamps	
		volatile bool states[BUFLEN];				// array to store states
		float h_angle;
		float v_angle;

	public:
		bool buffer_empty(void) volatile;
		bool buffer_full(void) volatile;
		unsigned long read_time(void) volatile;
		bool read_state(void) volatile;
		void write_time(unsigned long time_val) volatile;
		void write_state(bool state_val) volatile;
		void incrementWritePos(void) volatile;
		void incrementReadPos(void) volatile;
		unsigned long get_storedTime(unsigned short ndx) volatile;
		void set_h_angle(float ang) volatile;
		float get_h_angle(void) volatile;
		void set_v_angle(float ang) volatile;
		float get_v_angle(void) volatile;

	RingBuf()
	{
		readPos = 0;
		writePos = 0;
		// saw_sync = false;
		// sync_time = 0;
		// laser_time = 0;
		h_angle = 0;
		v_angle = 0;
		// initialRead = false;
	}
};

// return true if buffer is empty
bool RingBuf::buffer_empty(void) volatile
{
	return (readPos == writePos);
}

// return true if buffer is full
bool RingBuf::buffer_full(void) volatile
{
	return ((writePos + 1) % BUFLEN == readPos);
}

// reads from current buffer location; assumes buffer is not empty
unsigned long RingBuf::read_time(void) volatile
{
	unsigned long time_val = timeStamps[readPos];
	
	// increment read index and wrap around if necessary
	// ++readPos;		
	// if (readPos >= BUFLEN)
	// {
	// 	readPos = 0;
	// }
 
	return time_val;
}

// reads from current buffer location; assumes buffer is not empty
bool RingBuf::read_state(void) volatile
{
	bool state_val = states[readPos];
	
	// increment read index and wrap around if necessary
	// ++readPos;		
	// if (readPos >= BUFLEN)
	// {
	// 	readPos = 0;
	// }
 
	return state_val;
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_time(unsigned long time_val) volatile
{
	timeStamps[writePos] = time_val;

		// // increment write index and wrap around if necessary
		// ++writePos;
		// if (writePos >= BUFLEN)
		// {
		// 	writePos = 0;
		// }
	
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_state(bool state_val) volatile
{
	states[writePos] = state_val;

		// // increment write index and wrap around if necessary
		// ++writePos;
		// if (writePos >= BUFLEN)
		// {
		// 	writePos = 0;
		// }

}

// increment writePos and wrap around if necessary
void RingBuf::incrementWritePos(void) volatile
{
	writePos = writePos + 1;
	if (writePos >= BUFLEN)
	{
		writePos = 0;
	}
}

// increment readPos and wrap around if necessary
void RingBuf::incrementReadPos(void) volatile
{
	readPos = readPos + 1;
	if (readPos >= BUFLEN)
	{
		readPos = 0;
	}
}

unsigned long RingBuf::get_storedTime(unsigned short ndx) volatile
{
	return timeStamps[ndx];
}

void RingBuf::set_h_angle(float ang) volatile
{
	h_angle = ang;
}

float RingBuf::get_h_angle(void) volatile
{
	return h_angle;
}

void RingBuf::set_v_angle(float ang) volatile
{
	v_angle = ang;
}

float RingBuf::get_v_angle(void) volatile
{
	return v_angle;
}





volatile RingBuf buf;







void isr_lighthouse() {
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG);
	if (!buf.buffer_full())
	{
		buf.write_time(time_val);
		buf.write_state(state_val);
	}
	buf.incrementWritePos();
}

void setup() {
	pinMode(SIG, INPUT);
	Serial.begin(115200);
	attachInterrupt(digitalPinToInterrupt(SIG), isr_lighthouse, CHANGE);
}


unsigned long time_now = 0, time_prev = 0, ts = 0, tl = 0;
bool state_now = HIGH, state_prev = HIGH;
char pulse_type_now = '?', pulse_type_prev = '?';


void loop() {
	// wait until at least two data packets are in the queue
	while (buf.buffer_empty() || (buf.get_storedTime(1) == 0)) {;}

	time_now = buf.read_time();
	state_now = buf.read_state();
	buf.incrementReadPos();

	if ((state_now == HIGH) && (state_prev == LOW))
	{
		unsigned long delta = time_now - time_prev;
		if (delta > 50)
		{
			pulse_type_now = 'S';
			ts = time_prev;

			// reset other stuff
			tl = 0;
		}
		else if (delta > 15)
		{
			pulse_type_now = 'H';
			tl = time_prev;
			if (pulse_type_prev == 'S')
			{
				float ang = (tl - ts) * deg_per_us;
				buf.set_h_angle(ang);
			}
		}
		else if (delta > 7)
		{
			pulse_type_now = 'V';
			tl = time_prev;
			if (pulse_type_prev == 'S')
			{
				float ang = (tl - ts) * deg_per_us;
				buf.set_v_angle(ang);
			}
		}
		else
		{
			pulse_type_now = '?';
			
			// reset stuff
			ts = 0;
			tl = 0;
		}
	}

	state_prev = state_now;
	time_prev = time_now;
	pulse_type_prev = pulse_type_now;

	// print for debugging
	// if (++ctr % PRINTAFTER == 0)
	// {
	Serial.print("H angle: ");
	// for (int i = 0; i < NUM_SENSORS; i++)
	// {
	Serial.print(buf.get_h_angle());
	// Serial.print(" ");
	// }
	Serial.print("\nV angle: ");
	// for (int i = 0; i < NUM_SENSORS; i++)
	// {
	Serial.print(buf.get_v_angle());
	// Serial.print(" ");
	// }
	Serial.print("\n");
	// }
}