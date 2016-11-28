#define NUM_SENSORS 3
#define BUFLEN 256
#define PRINTAFTER 500

const byte SIG[] = {4, 5, 6};
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)
unsigned int ctr = 0;				// counter for printing

// Could use this struct instead of separate time and state arrays
struct DataPacket
{
	unsigned long timeStamp;
	bool state;

	DataPacket()
	{
		timeStamp = 0;
		state = HIGH;
	}
};

class RingBuf
{
	private:
		int id;										// object identifier (unique for each sensor)
		volatile short readPos;						// index to read at
		volatile short writePos;					// index to write at
		volatile DataPacket dataBuf[BUFLEN];		// array to store data packets	
		volatile float h_angle;						// current horizontal angle
		volatile float v_angle;						// current vertical angle

	public:
		void setID(int i) volatile { id = i; } 
		int getID() volatile { return id; }
		bool buffer_empty(void) volatile;
		bool buffer_full(void) volatile;
		DataPacket read_data(void) volatile;
		void write_data(DataPacket dp) volatile;
		unsigned long get_storedTime(unsigned short ndx) volatile;
		void set_h_angle(float ang) volatile { h_angle = ang; }
		float get_h_angle(void) volatile { return h_angle; }
		void set_v_angle(float ang) volatile { v_angle = ang; }
		float get_v_angle(void) volatile { return v_angle; }

	RingBuf()
	{
		readPos = 0;
		writePos = 0;
		h_angle = 0;
		v_angle = 0;
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

// reads data packet from current buffer location; assumes buffer is not empty
DataPacket RingBuf::read_data(void) volatile
{
	DataPacket dp;
	dp.timeStamp = dataBuf[readPos].timeStamp;
	dp.state = dataBuf[readPos].state;
	
	// increment read index and wrap around if necessary
	++readPos;		
	if (readPos >= BUFLEN)
	{
		readPos = 0;
	}
 
	return dp;
}

// add a data packet to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_data(DataPacket dp) volatile
{
	dataBuf[writePos].timeStamp = dp.timeStamp;
	dataBuf[writePos].state = dp.state;

	// increment write index and wrap around if necessary
	++writePos;
	if (writePos >= BUFLEN)
	{
		writePos = 0;
	}
	
}

unsigned long RingBuf::get_storedTime(unsigned short ndx) volatile
{
	return dataBuf[ndx].timeStamp;
}

volatile RingBuf bufs[NUM_SENSORS];


void isrA_lighthouse() {
	int i = 0;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		DataPacket dp;
		dp.timeStamp = time_val;
		dp.state = state_val;
		bufs[i].write_data(dp);
	}
}

void isrB_lighthouse() {
	int i = 1;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		DataPacket dp;
		dp.timeStamp = time_val;
		dp.state = state_val;
		bufs[i].write_data(dp);
	}
}

void isrC_lighthouse() {
	int i = 2;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		DataPacket dp;
		dp.timeStamp = time_val;
		dp.state = state_val;
		bufs[i].write_data(dp);
	}
}


void setup() {
	pinMode(SIG[0], INPUT);
	pinMode(SIG[1], INPUT);
	pinMode(SIG[2], INPUT);

	Serial.begin(115200);
	 
	for(int i = 0; i < NUM_SENSORS; i++) 
	{
		bufs[i].setID(i);
	}

	attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
}


unsigned long time_now[] = {0, 0, 0};
unsigned long time_prev[] = {0, 0, 0};
unsigned long ts[] = {0, 0, 0};
unsigned long tl[] = {0, 0, 0};
bool state_now[] = {HIGH, HIGH, HIGH};
bool state_prev[] = {HIGH, HIGH, HIGH};
char pulse_type_now[] = {'?', '?', '?'};
char pulse_type_prev[] = {'?', '?', '?'}; 

void loop() {
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		// wait until at least two data packets are in the queue
		while (bufs[i].buffer_empty() || (bufs[i].get_storedTime(1) == 0)) {;}

		DataPacket dp;
		dp = bufs[i].read_data();
		time_now[i] = dp.timeStamp;
		state_now[i] = dp.state;

		if ((state_now[i] == HIGH) && (state_prev[i] == LOW))	// valid pulse
		{
			unsigned long delta = time_now[i] - time_prev[i];
			if (delta > 50)			// SYNC pulse
			{
				pulse_type_now[i] = 'S';
				ts[i] = time_prev[i];

				// reset other stuff
				tl[i] = 0;
			}
			else if (delta > 18)	// HORIZONTAL LASER pulse
			{
				pulse_type_now[i] = 'H';
				tl[i] = time_prev[i];
				if (pulse_type_prev[i] == 'S')
				{
					float ang = (tl[i] - ts[i]) * deg_per_us;
					bufs[i].set_h_angle(ang);
				}

				// reset stuff
				ts[i] = 0;
				tl[i] = 0;
			}
			else if (delta > 8)		// VERTICAL LASER pulse
			{
				pulse_type_now[i] = 'V';
				tl[i] = time_prev[i];
				if (pulse_type_prev[i] == 'S')
				{
					float ang = (tl[i] - ts[i]) * deg_per_us;
					bufs[i].set_v_angle(ang);
				}

				// reset stuff
				ts[i] = 0;
				tl[i] = 0;
			}
			else					// meaningless pulse
			{
				pulse_type_now[i] = '?';
				
				// reset stuff
				ts[i] = 0;
				tl[i] = 0;
			}
		}

		// update previous values
		state_prev[i] = state_now[i];
		time_prev[i] = time_now[i];
		pulse_type_prev[i] = pulse_type_now[i];
	}

	// print for debugging
	if (++ctr % PRINTAFTER == 0)
	{
		Serial.print("H angles: ");
		for (byte j = 0; j < NUM_SENSORS; j++)
		{
			Serial.print(bufs[j].get_h_angle());
			Serial.print(" ");
		}
		Serial.print("\nV angles: ");
		for (int j = 0; j < NUM_SENSORS; j++)
		{
			Serial.print(bufs[j].get_v_angle());
			Serial.print(" ");
		}
		Serial.print("\n");
		Serial.print("\n");
	}
}