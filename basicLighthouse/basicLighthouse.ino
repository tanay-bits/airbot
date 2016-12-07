#define NUM_SENSORS 3
#define BUFLEN 8
#define PRINTAFTER 500

const byte SIG[] = {4, 5, 6};
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)
unsigned int ctr = 0;				// counter for printing

// // Could use this struct instead of separate time and state arrays
// struct DataPacket
// {
// 	uint32_t timeStamp;
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
		uint8_t id;						// object identifier (unique for each sensor)
		short readPos;					// index to read at
		short writePos;					// index to write at
		uint32_t timeStamps[BUFLEN];	// array to store time-stamps	
		bool states[BUFLEN];			// array to store states
		float h_angle;					// current horizontal angle
		float v_angle;					// current vertical angle

	public:
		void setID(uint8_t i) volatile; 
		uint8_t getID(void) volatile;
		bool buffer_empty(void) volatile;
		bool buffer_full(void) volatile;
		uint32_t read_time(void) volatile;
		bool read_state(void) volatile;
		void write_time(uint32_t time_val) volatile;
		void write_state(bool state_val) volatile;
		void incrementWritePos(void) volatile;
		void incrementReadPos(void) volatile;
		uint32_t get_storedTime(unsigned short ndx) volatile;
		void set_h_angle(float ang) volatile;
		float get_h_angle(void) volatile;
		void set_v_angle(float ang) volatile;
		float get_v_angle(void) volatile;

	RingBuf()
	{
		readPos = 0;
		writePos = 0;
		h_angle = 0;
		v_angle = 0;
	}
};

void RingBuf::setID(uint8_t i) volatile
{
	id = i;
}

uint8_t RingBuf::getID(void) volatile
{ 
	return id;
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
uint32_t RingBuf::read_time(void) volatile
{ 
	return timeStamps[readPos];
}

// reads from current buffer location; assumes buffer is not empty
bool RingBuf::read_state(void) volatile
{
	return states[readPos];
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_time(uint32_t time_val) volatile
{
	timeStamps[writePos] = time_val;	
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_state(bool state_val) volatile
{
	states[writePos] = state_val;
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

// query a time-stamp from the buffer
uint32_t RingBuf::get_storedTime(unsigned short ndx) volatile
{
	return timeStamps[ndx];
}

volatile RingBuf bufs[NUM_SENSORS];


void isrA_lighthouse() {
	uint32_t time_val = micros();
	uint8_t i = 0;
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}

void isrB_lighthouse() {
  uint32_t time_val = micros();
  uint8_t i = 1;
  bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}

void isrC_lighthouse() {
  uint32_t time_val = micros();
  uint8_t i = 2;
  bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}


void setup() {
	pinMode(SIG[0], INPUT);
	pinMode(SIG[1], INPUT);
	pinMode(SIG[2], INPUT);

	Serial.begin(115200);
	 
	for(uint8_t i = 0; i < NUM_SENSORS; i++) 
	{
		bufs[i].setID(i);
	}

	attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
}


uint32_t time_now[] = {0, 0, 0};
uint32_t time_prev[] = {0, 0, 0};
uint32_t ts[] = {0, 0, 0};
uint32_t tl[] = {0, 0, 0};
bool state_now[] = {HIGH, HIGH, HIGH};
bool state_prev[] = {HIGH, HIGH, HIGH};
char pulse_type_now[] = {'?', '?', '?'};
char pulse_type_prev[] = {'?', '?', '?'}; 

void loop() {
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		// wait until at least two data packets are in the queue
		while (bufs[i].buffer_empty() || (bufs[i].get_storedTime(1) == 0)) {;}

		time_now[i] = bufs[i].read_time();
		state_now[i] = bufs[i].read_state();
		bufs[i].incrementReadPos();

		if ((state_now[i] == HIGH) && (state_prev[i] == LOW))	// valid pulse
		{
			uint32_t delta = time_now[i] - time_prev[i];
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
		for (uint8_t j = 0; j < NUM_SENSORS; j++)
		{
			Serial.print(bufs[j].get_v_angle());
			Serial.print(" ");
		}
		Serial.print("\n");
		Serial.print("\n");
	}
}
