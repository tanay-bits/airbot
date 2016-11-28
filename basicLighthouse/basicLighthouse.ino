#define NUM_SENSORS 3
#define BUFLEN 256
#define PRINTAFTER 200000

const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)
const byte SIG[] = {4, 5, 6};
unsigned long ctr = 0;	// counter for printing

struct DataPacket
{
	bool state;
	unsigned long timeStamp;

	DataPacket()
	{
		state = HIGH;
		timeStamp = 0;
	}
};

DataPacket dpOld, dpNew;

class RingBuf
{
	public:
		short readPos;					// index to read at
		short writePos;					// index to write at
		DataPacket data_buf[BUFLEN];	// array of data packets
		bool saw_sync;					// true only whe
		unsigned long sync_time;		// time of falling edge of sync pulse
		unsigned long laser_time;		// time of falling edge of laser pulse
		float h_angle;					// last valid horizontal angle
		float v_angle;					// last valid vertical angle
		bool initialRead;				// should be true after initial reading  

		bool buffer_empty(void);
		bool buffer_full(void);
		DataPacket buffer_read(void);
		void buffer_write(DataPacket dp);

		RingBuf()
		{
			readPos = 0;
			writePos = 0;
			saw_sync = false;
			sync_time = 0;
			laser_time = 0;
			h_angle = 0;
			v_angle = 0;
			initialRead = false;
		}
};

// return true if buffer is empty
bool RingBuf::buffer_empty(void)
{
	return readPos == writePos;
}

// return true if buffer is full
bool RingBuf::buffer_full(void)
{
	return (writePos + 1) % BUFLEN == readPos;
}

// reads from current buffer location; assumes buffer is not empty
DataPacket RingBuf::buffer_read(void)
{
	DataPacket dp = data_buf[readPos];
	
	// increment read index and wrap around if necessary
	++readPos;		
	if (readPos >= BUFLEN)
	{
		readPos = 0;
	}
 
	return dp;
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::buffer_write(DataPacket dp)
{
	if (!buffer_full())
	{
		data_buf[writePos] = dp;

		// increment write index and wrap around if necessary
		++writePos;
		if (writePos >= BUFLEN)
		{
			writePos = 0;
		}
	}
}

static RingBuf bufA;


void setup() {
	pinMode(SIG[0], INPUT);
	// pinMode(SIG[1], INPUT);
	// pinMode(SIG[2], INPUT);
	Serial.begin(115200);
	attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
}

void loop() {
	// wait until at least two data packets are in the queue
	while (bufA.buffer_empty() || bufA.data_buf[1].timeStamp == 0) {;}		

	dpNew.timeStamp = bufA.buffer_read().timeStamp;	// read the last unread packet
	dpNew.state = bufA.buffer_read().state;			// read the last unread packet
	
	if (bufA.initialRead)		// skip if initial reading
	{
		if ((dpNew.state == HIGH) && (dpOld.state == LOW))	// check for valid pulse
		{
			unsigned long delta = dpNew.timeStamp - dpOld.timeStamp;
			if (delta > 50)			// sync pulse
			{
				bufA.saw_sync = true;
				bufA.sync_time = dpOld.timeStamp;

				// reset other stuff
				bufA.laser_time = 0;
			}
			else if (delta > 15)	// horizontal laser pulse
			{
				bufA.laser_time = dpOld.timeStamp;
				
				if (bufA.saw_sync)
				{
					bufA.h_angle = (bufA.laser_time - bufA.sync_time) * deg_per_us;
				}

				// reset other stuff
				bufA.laser_time = 0;
				bufA.sync_time = 0;
				bufA.saw_sync = false;
			}
			else if (delta > 7)		// vertical laser pulse
			{
				bufA.laser_time = dpOld.timeStamp;
				
				if (bufA.saw_sync)
				{
					bufA.v_angle = (bufA.laser_time - bufA.sync_time) * deg_per_us;
				}
				
				// reset other stuff
				bufA.laser_time = 0;
				bufA.sync_time = 0;
				bufA.saw_sync = false;
			}
			else					// meaningless reading; reset stuff
			{	
				bufA.laser_time = 0;
				bufA.sync_time = 0;
				bufA.saw_sync = false;
			}	
		}

	}

	// update the member variables of dpOld with those of last reading
	dpOld.timeStamp = dpNew.timeStamp;
	dpOld.state = dpNew.state;

	if (!bufA.initialRead)
	{
		bufA.initialRead = true;
	}

	// print for debugging
	// if (++ctr % PRINTAFTER == 0)
	// {
	Serial.print("H angle: ");
	// for (int i = 0; i < NUM_SENSORS; i++)
	// {
	Serial.print(bufA.h_angle);
	// Serial.print(" ");
	// }
	Serial.print("\nV angle: ");
	// for (int i = 0; i < NUM_SENSORS; i++)
	// {
	Serial.print(bufA.v_angle);
	// Serial.print(" ");
	// }
	Serial.print("\n");
	// }
}

void isrA_lighthouse() {
	byte ndx = 0;
	DataPacket dp;
	dp.timeStamp = micros();
	dp.state = digitalReadFast(SIG[ndx]);
	bufA.buffer_write(dp);
}



// void isrA_lighthouse() {
// 	byte ndx = 0;

// 	time_now[ndx] = micros();
// 	state_now[ndx] = digitalReadFast(SIG[ndx]);

// 	if ((state_now[ndx] == HIGH) && (state_prev[ndx] == LOW))
// 	{
// 		unsigned long delta = time_now[ndx] - time_prev[ndx];
// 		if (delta > 50)
// 		{
// 			pulse_type_now[ndx] = 'S';
// 			ts[ndx] = time_prev[ndx];

// 			// reset other stuff
// 			tl[ndx] = 0;
// 		}
// 		else if (delta > 15)
// 		{
// 			pulse_type_now[ndx] = 'H';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				h_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else if (delta > 7)
// 		{
// 			pulse_type_now[ndx] = 'V';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				v_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else
// 		{
// 			pulse_type_now[ndx] = '?';
			
// 			// reset stuff
// 			ts[ndx] = 0;
// 			tl[ndx] = 0;
// 		}
// 	}

// 	state_prev[ndx] = state_now[ndx];
// 	time_prev[ndx] = time_now[ndx];
// 	pulse_type_prev[ndx] = pulse_type_now[ndx];
// }

// void isrB_lighthouse() {
// 	byte ndx = 1;

// 	time_now[ndx] = micros();
// 	state_now[ndx] = digitalReadFast(SIG[ndx]);

// 	if ((state_now[ndx] == HIGH) && (state_prev[ndx] == LOW))
// 	{
// 		unsigned long delta = time_now[ndx] - time_prev[ndx];
// 		if (delta > 50)
// 		{
// 			pulse_type_now[ndx] = 'S';
// 			ts[ndx] = time_prev[ndx];

// 			// reset other stuff
// 			tl[ndx] = 0;
// 		}
// 		else if (delta > 15)
// 		{
// 			pulse_type_now[ndx] = 'H';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				h_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else if (delta > 7)
// 		{
// 			pulse_type_now[ndx] = 'V';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				v_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else
// 		{
// 			pulse_type_now[ndx] = '?';
			
// 			// reset stuff
// 			ts[ndx] = 0;
// 			tl[ndx] = 0;
// 		}
// 	}

// 	state_prev[ndx] = state_now[ndx];
// 	time_prev[ndx] = time_now[ndx];
// 	pulse_type_prev[ndx] = pulse_type_now[ndx];
// }

// void isrC_lighthouse() {
// 	byte ndx = 2;

// 	time_now[ndx] = micros();
// 	state_now[ndx] = digitalReadFast(SIG[ndx]);

// 	if ((state_now[ndx] == HIGH) && (state_prev[ndx] == LOW))
// 	{
// 		unsigned long delta = time_now[ndx] - time_prev[ndx];
// 		if (delta > 50)
// 		{
// 			pulse_type_now[ndx] = 'S';
// 			ts[ndx] = time_prev[ndx];

// 			// reset other stuff
// 			tl[ndx] = 0;
// 		}
// 		else if (delta > 15)
// 		{
// 			pulse_type_now[ndx] = 'H';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				h_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else if (delta > 7)
// 		{
// 			pulse_type_now[ndx] = 'V';
// 			tl[ndx] = time_prev[ndx];
// 			if (pulse_type_prev[ndx] == 'S')
// 			{
// 				v_angles[ndx] = (tl[ndx] - ts[ndx]) * deg_per_us;
// 			}
// 		}
// 		else
// 		{
// 			pulse_type_now[ndx] = '?';
			
// 			// reset stuff
// 			ts[ndx] = 0;
// 			tl[ndx] = 0;
// 		}
// 	}

// 	state_prev[ndx] = state_now[ndx];
// 	time_prev[ndx] = time_now[ndx];
// 	pulse_type_prev[ndx] = pulse_type_now[ndx];
// }
