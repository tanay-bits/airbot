#ifndef LIGHTHOUSE__H__
#define LIGHTHOUSE__H__

#include <Arduino.h>

#define NUM_SENSORS 3
#define BUFLEN 256

const byte SIG[] = {4, 5, 6};		// pins to which the sensors are hooked
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)

unsigned long time_now[] = {0, 0, 0};
unsigned long time_prev[] = {0, 0, 0};
unsigned long ts[] = {0, 0, 0};
unsigned long tl[] = {0, 0, 0};
bool state_now[] = {HIGH, HIGH, HIGH};
bool state_prev[] = {HIGH, HIGH, HIGH};
char pulse_type_now[] = {'?', '?', '?'};
char pulse_type_prev[] = {'?', '?', '?'}; 

class RingBuf
{
	private:
		int id;										// object identifier (unique for each sensor)
		volatile short readPos;						// index to read at
		volatile short writePos;					// index to write at
		volatile unsigned long timeStamps[BUFLEN];	// array to store time-stamps	
		volatile bool states[BUFLEN];				// array to store states
		volatile float h_angle;						// current horizontal angle
		volatile float v_angle;						// current vertical angle

	public:
		void setID(int i) volatile; 
		int getID(void) volatile;
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
		h_angle = 0;
		v_angle = 0;
	}
};

// Create array of ring buffers - one for each sensor
volatile RingBuf bufs[NUM_SENSORS];

void isrA_lighthouse(void);
void isrB_lighthouse(void);
void isrC_lighthouse(void);

#endif