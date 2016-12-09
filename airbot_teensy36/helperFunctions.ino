/************************************************************
 This file is part of the AirBot firmware.
 Project repository: https://github.com/tanay-bits/airbot
 Written by Tanay Choudhary	(https://tanay-bits.github.io/).
 ************************************************************/

/*
HELPER FUNCTIONS
*/

// updates lighthouse angles
void read_lighthouse(void) {
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
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
}

// triangulate XYZ position of each sensor
void triangulate(float sA[3],float sB[3],float sC[3],float h1,float v1,float h2,float v2,float h3,float v3) {
	h1 = h1 * M_PI/180.0;
	v1 = v1 * M_PI/180.0;

	h2 = h2 * M_PI/180.0;
	v2 = v2 * M_PI/180.0;

	h3 = h3 * M_PI/180.0;
	v3 = v3 * M_PI/180.0;

	float eqsVec[3];

	float **jacMat=(float**)malloc(3*(sizeof(float*)));
	int i;
	for(i=0;i<3;i++)
		*(jacMat+i)=(float*)malloc(sizeof(float)*3);

	cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
	cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
	cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);

	float x0[3] = {50, 51, 52};		// initial guess
	int* iter;
	int ival = 50000;
	iter = &ival;

	newtonOpt(x0, iter, eqsVec, jacMat);

	sA[0] = x0[0] * sin(v1) * cos(h1);
	sA[1] = x0[0] * sin(v1) * sin(h1);
	sA[2] = x0[0] * cos(v1);

	sB[0] = x0[1] * sin(v2) * cos(h2);
	sB[1] = x0[1] * sin(v2) * sin(h2);
	sB[2] = x0[1] * cos(v2);

	sC[0] = x0[2] * sin(v3) * cos(h3);
	sC[1] = x0[2] * sin(v3) * sin(h3);
	sC[2] = x0[2] * cos(v3);
}

// maps yaw to (0:360) system
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

// limits motor speeds b/w 0 to MAX_SPEED
int saturateSpeed(float s) {
  if (s > MAX_SPEED) {return MAX_SPEED;}
  else if (s < 0) {return 0;}
  else {return s;}
}

// check if slow retraction is required (when actuators are maxed out and Eint > EINT_CAP)
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

// // Only needed if continuous output streaming is on
// bool readToken(char* token) {
//   IMUserial_clear(); // Clear input buffer
  
//   IMUSERIAL.write("#s00"); // Request synch token
//   delay(500);
//   // Check if incoming bytes match token
//   for (unsigned int i = 0; i < (sizeof(token) - (unsigned)1); i++)
// 	{
// 	  if (IMUSERIAL.read() != token[i])
// 		return false;
// 	}
//   return true;
// }

// // Generate trajectory array to follow
// void genRef(volatile int *ref, int *times, int *angs) {
//   int sample_list[3];

//   for (int i = 0; i < 3; i++)
//   {
// 	sample_list[i] = (int)(times[i] * CONTROL_FREQ);
//   }

//   refSize = sample_list[2]; // last element of sample_list (global)
//   int j = 1;
//   for (int i = 0; i < refSize; i++)
//   {
//    if (i == sample_list[j] - 1) {j++;}
//    ref[i] = angs[j-1];  
//   }
// }
