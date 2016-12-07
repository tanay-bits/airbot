
//////////////////////
// HELPER FUNCTIONS //
//////////////////////

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
	for (uint8_t i = 0; i < 4; i++)  // Read yaw angle
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
				delay(10);
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