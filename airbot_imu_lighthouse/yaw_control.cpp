/* This file is part of the AirBot firmware */

#include "yaw_control.h"


/////////
// ISR //
/////////

void controller(void)
{  
  int e_yaw;

  switch (get_mode())
  {
		case IDLE:
		{
		  break;
		}

		case READ:
		{
		  yawNow = read_yaw();
		  if (BTSERIAL.available())
		  {
				set_mode(IDLE);
		  }
		  break;
		}
		
		case HOLD:
		{
		  // calculate control error and integral of error
		  yawNow = read_yaw();
		  e_yaw = calc_error(yawTarget - yawNow);
		  Eint = Eint + e_yaw;
		  	  
		  // calculate control signal and send to actuator, if error outside deadband
		  calc_send_control(e_yaw);

		  // update previous error for derivative calculation
		  e_yaw_prev = e_yaw;

		  if (BTSERIAL.available())
		  {
				set_mode(IDLE);
		  }
		  break;
		}

		case TRACK:		// TODO
		{
		  break;
		}

		case VIVE:		// TODO
		{
		  read_lighthouse();
		  if (BTSERIAL.available())
		  {
				set_mode(IDLE);
		  }
		  break;
		}

		default:
		{
		  digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
		  break;
		}
  }
}

//////////////////////
// HELPER FUNCTIONS //
//////////////////////

void read_lighthouse(void)
{
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		// wait until at least two data packets are in the queue
		while (bufs[i].buffer_empty() || (bufs[i].get_storedTime(1) == 0)) {;}

		time_now[i] = bufs[i].read_time();
		state_now[i] = bufs[i].read_state();
		bufs[i].incrementReadPos();

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
}

// Maps yaw to (0:360) system
int correct_yaw(float x) {
  if (x < 0)
  {
	x = x + 360;
  }
  else if (x > 360)
  {
	x = x - 360;
  }
  return (int)x;
}

int read_yaw(void) {
  IMUserial_clear();  // Clear input buffer
  IMUSERIAL.write("#f");  // Request one output frame
  while (IMUSERIAL.available() < 4) {;}  // Block until 4 bytes are received
  for (int i = 0; i < 4; i++)  // Read yaw angle
  {
	u.b_angle[i] = IMUSERIAL.read();    
  }
  return correct_yaw(u.f_angle[0]);
}

int calc_error(int diff) {
  if (diff > 180)
  {
	diff = diff - 360;
  }
  else if (diff < -180)
  {
	diff = 360 + diff;
  }
  return diff;
}

// Limits motor speeds b/w 0 to MAX_SPEED
int saturateSpeed(float s) {
  if (s > MAX_SPEED) {return MAX_SPEED;}
  else if (s < 0) {return 0;}
  else {return s;}
}

// Check if slow retraction is required (when actuators are maxed out and Eint > EINT_CAP)
bool need_to_retract(void) {
  if ( (abs(Eint) > EINT_CAP) && (abs(vals[0] - vals[1]) == MAX_SPEED) )
  {
	return true;
  }
  return false;
}

// calculate control signal and send to actuator, if error outside deadband
void calc_send_control(int e_yaw) {
  if (!need_to_retract())  // NOT saturation
  {
	if (abs(e_yaw) > yawTol)
	{
	  control_sig = Kp*e_yaw + Ki*Eint + Kd*(e_yaw - e_yaw_prev);  // ~ motor speed change
	  vals[0] = saturateSpeed(vals[0] - control_sig);  // new speed for motor 0
	  vals[1] = saturateSpeed(vals[1] + control_sig);  // new speed for motor 1
	  esc0.write(vals[0]);
	  esc1.write(vals[1]);
	}
  }
  else  // saturation => need to retract
  {
	noInterrupts();
	int delta = 1;
	int middle = MAX_SPEED/2;
	if (vals[0] > vals[1])
	{
	  while ((vals[0] != middle) || (vals[1] != middle))
	  {
		if (vals[0] > middle)
		{
		  vals[0] = vals[0] - delta;
		  esc0.write(vals[0]);
		}
		if (vals[1] < middle)
		{
		  vals[1] = vals[1] + delta;
		  esc1.write(vals[1]);
		}       
		delay(10);
	  }
	}
	else
	{
	  while ((vals[0] != middle) || (vals[1] != middle))
	  {
		if (vals[1] > middle)
		{
		  vals[1] = vals[1] - delta;
		  esc1.write(vals[1]);
		}
		if (vals[0] < middle)
		{
		  vals[0] = vals[0] + delta;
		  esc0.write(vals[0]);
		}       
		delay(50);
	  }      
	}
	Eint = 0;
	interrupts();
  }
}

void BTserial_clear(void) {
  while (BTSERIAL.available()) {BTSERIAL.read();}
}

void BTserial_block(void) {
  while (!BTSERIAL.available()) {;}
}

void IMUserial_clear(void) {
  while (IMUSERIAL.available()) {IMUSERIAL.read();}
}

void IMUserial_block(void) {
  while (!IMUSERIAL.available()) {;}
}

void set_mode(Mode_datatype m) {
  mode = m;
}

Mode_datatype get_mode(void) {
  return mode;
}

// Only needed if continuous output streaming is on
bool readToken(char* token) {
  IMUserial_clear(); // Clear input buffer
  
  IMUSERIAL.write("#s00"); // Request synch token
  delay(500);
  // Check if incoming bytes match token
  for (unsigned int i = 0; i < (sizeof(token) - (unsigned)1); i++)
	{
	  if (IMUSERIAL.read() != token[i])
		return false;
	}
  return true;
}