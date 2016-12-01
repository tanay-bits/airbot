#define NUM_SENSORS 3
#define BUFLEN 16
#define BTSERIAL Serial2

#define ANGSLEN 50

const uint8_t SIG[] = {4, 5, 6};
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)


class RingBuf
{
	private:
		uint8_t id;						// object identifier (unique for each sensor)
		uint8_t readPos;				// index to read at
		uint8_t writePos;				// index to write at
		uint32_t timeStamps[BUFLEN];	// array to store time-stamps	
		bool states[BUFLEN];			// array to store states
		float h_angles[ANGSLEN];		// current horizontal angle
		float v_angles[ANGSLEN];		// current vertical angle
		uint8_t h_count;				// no. of h angles collected
		uint8_t v_count;				// no. of v angles collected

	public:
		void setID(uint8_t i) volatile; 
		uint8_t getID(void) volatile;
		void setHcount(uint8_t i) volatile; 
		uint8_t getHcount(void) volatile;
		void setVcount(uint8_t i) volatile; 
		uint8_t getVcount(void) volatile;
		bool buffer_empty(void) volatile;
		bool buffer_full(void) volatile;
		uint32_t read_time(void) volatile;
		bool read_state(void) volatile;
		void write_time(uint32_t time_val) volatile;
		void write_state(bool state_val) volatile;
		void incrementWritePos(void) volatile;
		void incrementReadPos(void) volatile;
		uint32_t get_storedTime(uint8_t ndx) volatile;
		void insert_h_angle(float ang, uint8_t ndx) volatile;
		float get_h_angle(uint8_t ndx) volatile;
		void insert_v_angle(float ang, uint8_t ndx) volatile;
		float get_v_angle(uint8_t ndx) volatile;

	RingBuf()
	{
		readPos = 0;
		writePos = 0;
		h_count = 0;
		v_count = 0;
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

void RingBuf::setHcount(uint8_t i) volatile
{
	h_count = i;
}

uint8_t RingBuf::getHcount(void) volatile
{ 
	return h_count;
}

void RingBuf::setVcount(uint8_t i) volatile
{
	v_count = i;
}

uint8_t RingBuf::getVcount(void) volatile
{ 
	return v_count;
}

void RingBuf::insert_h_angle(float ang, uint8_t ndx) volatile 
{
	h_angles[ndx] = ang;
}

float RingBuf::get_h_angle(uint8_t ndx) volatile
{
	return h_angles[ndx];
}

void RingBuf::insert_v_angle(float ang, uint8_t ndx) volatile
{
	v_angles[ndx] = ang;
}

float RingBuf::get_v_angle(uint8_t ndx) volatile
{
	return v_angles[ndx];
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
uint32_t RingBuf::get_storedTime(uint8_t ndx) volatile
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

	BTSERIAL.begin(9600);
	 
	for(uint8_t i = 0; i < NUM_SENSORS; i++) 
	{
		bufs[i].setID(i);
	}
}


uint32_t time_now[] = {0, 0, 0};
uint32_t time_prev[] = {0, 0, 0};
uint32_t ts[] = {0, 0, 0};
uint32_t tl[] = {0, 0, 0};
bool state_now[] = {HIGH, HIGH, HIGH};
bool state_prev[] = {HIGH, HIGH, HIGH};
char pulse_type_now[] = {'?', '?', '?'};
char pulse_type_prev[] = {'?', '?', '?'};
bool flag[] = {false, false, false};		// flags go true when angles are collected

void loop() {
	// if MATLAB requests angles
	if (BTSERIAL.available())
	{
		char input = BTSERIAL.read();
		switch (input)
		{
			case 'l':
			{
				noInterrupts();
				attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
				attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
				attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
				interrupts();

				// compute a bunch of angles until enough are collected
				while (!(flag[0] && flag[1] && flag[2]))
				{
					for (uint8_t i = 0; i < NUM_SENSORS; i++)
					{
						// update flag for this sensor
						if ((bufs[i].getHcount() < ANGSLEN) || (bufs[i].getVcount() < ANGSLEN))
						{
							flag[i] = false;
						}
						else
						{
							flag[i] = true;
						}

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
									bufs[i].insert_h_angle(ang, bufs[i].getHcount());
									bufs[i].setHcount(bufs[i].getHcount() + 1);
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
									bufs[i].insert_v_angle(ang, bufs[i].getVcount());
									bufs[i].setVcount(bufs[i].getVcount() + 1);
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
				}

				detachInterrupt(digitalPinToInterrupt(SIG[0]));
				detachInterrupt(digitalPinToInterrupt(SIG[1]));
				detachInterrupt(digitalPinToInterrupt(SIG[2]));

				// calculate averages
				float Hsum = 0, Vsum = 0;
				float h_av[NUM_SENSORS], v_av[NUM_SENSORS];

				for (uint8_t sen = 0; sen < NUM_SENSORS; sen++)
				{
					for (uint8_t i = 0; i < ANGSLEN; i++)
					{
						Hsum = Hsum + bufs[sen].get_h_angle(i);
						Vsum = Vsum + bufs[sen].get_v_angle(i);
					}
					h_av[sen] = (float) (Hsum / ANGSLEN);
					v_av[sen] = (float) (Vsum / ANGSLEN);
					Hsum = 0;
					Vsum = 0;
				}
				
				// send to MATLAB
				char str[10];
				for (uint8_t sen = 0; sen < NUM_SENSORS; sen++)
				{
					// send horizontal angle of sen 
					dtostrf(h_av[sen], 4, 2, str);
					BTSERIAL.write(str);
					BTSERIAL.write("\r\n");

					// send vertical angle of sen
					dtostrf(v_av[sen], 4, 2, str);
					BTSERIAL.write(str);
					BTSERIAL.write("\r\n");
				}
				
				break;
			}

			default:
			{
				break;
			}
		}
	}
}
