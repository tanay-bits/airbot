/* This file is part of the AirBot firmware */

#include "lighthouse.h"


//////////////////////////////////////////////
// RING BUFFER CLASS FOR LIGHTHOUSE SENSING //
//////////////////////////////////////////////

void RingBuf::setID(int i) volatile
{
	id = i;
}

int RingBuf::getID(void) volatile
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
unsigned long RingBuf::read_time(void) volatile
{ 
	return timeStamps[readPos];
}

// reads from current buffer location; assumes buffer is not empty
bool RingBuf::read_state(void) volatile
{
	return states[readPos];
}

// add an element to the buffer, if it's not full (otherwise data would be lost)
void RingBuf::write_time(unsigned long time_val) volatile
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
unsigned long RingBuf::get_storedTime(unsigned short ndx) volatile
{
	return timeStamps[ndx];
}


////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINES FOR EACH SENSOR //
////////////////////////////////////////////////

void isrA_lighthouse() {
	int i = 0;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}

void isrB_lighthouse() {
	int i = 1;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}

void isrC_lighthouse() {
	int i = 2;
	unsigned long time_val = micros();
	bool state_val = digitalReadFast(SIG[i]);
	if (!bufs[i].buffer_full())
	{
		bufs[i].write_time(time_val);
		bufs[i].write_state(state_val);
	}
	bufs[i].incrementWritePos();
}