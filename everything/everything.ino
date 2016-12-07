/////////////////////////////////
// IMPORTS, CONSTANTS, GLOBALS //
/////////////////////////////////

#include <Servo.h>

// #define PRINTAFTER 500
#define ANGSLEN 50
#define NUM_SENSORS 3
#define BUFLEN 16
#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define LEDPIN 13
// #define MAXSAMPS 50      				// max number of samples in ref trajectory

const int MAX_SPEED = 160;    		// max ESC write value
const int CONTROL_PERIOD_MS = 22;	// controller fires up every these many milliseconds
const uint8_t SIG[] = {4, 5, 6};
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)

// Create Servo objects: esc0 - CW wheel, esc1 - CCW wheel
Servo esc0, esc1;

// Create an IntervalTimer object for the controller
IntervalTimer myTimer;
// myTimer.priority(number);  // 0 highest and 255 lowest

// Union data structure to save incoming byte array as floats representing an angle (deg):
union u_tag {
  byte b_angle[4];
  float f_angle[1];
} u; 

// Enumerated type to represent different modes of operation
typedef enum {IDLE=0, HOLD=1, TRACK=2} Mode_datatype;                                  

volatile Mode_datatype mode;                       // declare global var which is the current mode 
volatile bool startup = false;
volatile bool synched = false;
volatile int yawTol = 0;                           // yaw error tolerance
volatile int ang_target = 0;                       // target yaw angle
volatile float Kp = 0.1, Ki = 0, Kd = 2;           // yawdot = Awy * (wu - wnom)
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_yaw_prev = 0;                       // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int EINT_CAP = 50000;                     // execute slow retraction if Eint >= EINT_CAP
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)
// volatile uint16_t num_samples = 0;                 // # of samples in ref trajectory
// volatile int16_t REFtraj[MAXSAMPS];                // stores reference trajectory
// volatile int16_t SENtraj[MAXSAMPS];                // stores measured trajectory


//////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
// RING BUFFER CLASS, LIGHTHOUSE ISR //
/////////////////////////////////////// 

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

// Create array of ring buffers - one for each sensor
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


//////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////
// CONTROLLER ISR //
////////////////////

void controller(void)
{
	// static int ctr = 0;  // initialize counter once  
  int e_yaw, yawNow, yawTarget;

  switch (get_mode())
  {
		case IDLE:
		{
		  break;
		}
		
		case HOLD:
		{
		  // calculate control error and integral of error
		  yawNow = read_yaw();
		  yawTarget = ang_target;
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
			// // calculate control error and integral of error
		 //  yawNow = read_yaw();
		 //  yawTarget = REFtraj[ctr];
		 //  e_yaw = calc_error(yawTarget - yawNow);
		 //  Eint = Eint + e_yaw;

		 //  // calculate control signal and send to actuator, if error outside deadband
		 //  calc_send_control(e_yaw);

		 //  // update previous error for derivative calculation
		 //  e_yaw_prev = e_yaw;

		 //  // // store data for MATLAB:
   //  //   SENtraj[ctr] = yawNow;

   //    ctr++;
   //    if (ctr == num_samples)
   //    {
   //      ang_target = REFtraj[num_samples-1];  // set last ref as target for HOLD
   //      ctr = 0;                              // reset counter
   //      set_mode(HOLD);
   //    }

		  break;
		}

		default:
		{
		  digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
		  break;
		}
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

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


//////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////
// SETUP AND LOOP //
////////////////////

void setup() {
  delay(3000);  	// give enough time for Razor to auto-reset
  noInterrupts();	// disable interrupts
  
  // make sensor pins inputs
  pinMode(SIG[0], INPUT);
  pinMode(SIG[1], INPUT);
  pinMode(SIG[2], INPUT);


  // initialize serial channels
  BTSERIAL.begin(9600);
//  IMUSERIAL.begin(58824);
  IMUSERIAL.begin(57600);
  
  // arm the ESC's
  esc0.attach(16);
  esc1.attach(17);
  esc0.write(vals[0]);
  esc1.write(vals[1]);
  
  // set Razor output parameters
  IMUSERIAL.write("#ob");  // Turn on binary output
  IMUSERIAL.write("#o0");  // Turn OFF continuous streaming output
  IMUSERIAL.write("#oe0"); // Disable error message output 

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);  // turn on LED

  for(int i = 0; i < NUM_SENSORS; i++) 
	{
		bufs[i].setID(i);
	}

  interrupts();	//enable interrupts
  
  // controller to run every 22ms (IMU sensing is at 20ms)
  myTimer.begin(controller, CONTROL_PERIOD_MS * 1000);
}

void loop() {
	// Keep polling for user input
	if (BTSERIAL.available())
	{
	  char input = BTSERIAL.read();
	  digitalWrite(LEDPIN, LOW);
	  
	  switch (input)
	  {
	  	// unpower motors
	  	case 'p':                      
	  	{
	  	  noInterrupts();
	  	  
	  	  esc0.write(0);
	  	  esc1.write(0);
	  	  digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate reset
	  	  set_mode(IDLE);
	  	  BTSERIAL.write("Motors are powered off.\r\n");
	  	  delay(1000);
	  	  
	  	  interrupts();
	  	  break;
	  	}

			// Equilibrium mode (both motors ramp up to MAX_SPEED/2)
			case 'e':
			{         
			  for (int i=10; i <= MAX_SPEED/2; i = i+1)
			  {
					vals[0] = i;
					vals[1] = i;
					esc0.write(vals[0]);
					esc1.write(vals[1]);
			  }
			  BTSERIAL.write("Ramp-up to equilibrium complete.\r\n");  
			  break;
			}
			
			// get parameters - PID gains, yawTol, EINT_CAP, current mode
			case '?':                      
			{
			  noInterrupts();
			  
			  char str[10];
			  dtostrf(Kp, 4, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  dtostrf(Ki, 4, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  dtostrf(Kd, 4, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");
			  
			  dtostrf(yawTol, 4, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  dtostrf(EINT_CAP, 4, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  switch (get_mode())
			  {
					case IDLE:
					{
					  BTSERIAL.write("IDLE mode\r\n");
					  break;
					}
					case HOLD:
					{
					  BTSERIAL.println("HOLD mode\r\n");
					  break;
					}
					case TRACK:
					{
					  BTSERIAL.println("TRACK mode\r\n");
					  break;
					}
					default:
					{
					  BTSERIAL.println("no valid mode!??\r\n");
					  break; 
					}
			  }
			  
			  interrupts();
			  break;
			}

			// set PID gains
			case 's':                      
			{
			  noInterrupts();
			  delay(1000);

			  BTserial_block();
			  Kp = BTSERIAL.parseFloat();

			  BTserial_block();
			  Ki = BTSERIAL.parseFloat();

			  BTserial_block();
			  Kd = BTSERIAL.parseFloat();

			  char str[10];
			  dtostrf(Kp, 1, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  dtostrf(Ki, 1, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");

			  dtostrf(Kd, 1, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");
			  
			  interrupts();
			  break;
			}

			// read and print current yaw (deg)
			case 'r':                      
			{
			  noInterrupts();
			  delay(1000);
			  BTserial_clear();
			  char str[10];
			  int yaw = read_yaw();
			  dtostrf(yaw, 1, 2, str);
			  BTSERIAL.write(str);
			  BTSERIAL.write("\r\n");
			  interrupts();
			  break;
			}

			// manually change motor speeds
			case 'm':
			{
			  noInterrupts();
			  delay(1000);
			  BTserial_block();
			  vals[0] = BTSERIAL.parseInt();
			  vals[1] = BTSERIAL.parseInt();
			  esc0.write(vals[0]);
			  esc1.write(vals[1]);
			  set_mode(IDLE);
			  delay(1000);
			  interrupts();
			  break;
			}

			// go to target yaw and hold
			case 'h':
			{
			  noInterrupts();
			  delay(1000);
			  e_yaw_prev = 0;
			  Eint = 0;
			  control_sig = 0;
			  BTserial_block();
			  int yawChange = BTSERIAL.parseInt();
			  int yaw = read_yaw();                   
			  ang_target = correct_yaw(yaw + yawChange);          
			  set_mode(HOLD);
			  interrupts();
			  while (get_mode() == HOLD)
			  {
			  	;
			  }
			  delay(1000);
			  break;
			}

			// // load step trajectory
			// case 'x':                      
			// {
			// 	noInterrupts();
			//   int i, ref_deg;
			//   BTserial_block();
			//   num_samples = BTSERIAL.parseInt();
			//   digitalWrite(LEDPIN, HIGH);		  
			//   for (i = 0; i < num_samples; i++)
			//   {
			//   	ref_deg = BTSERIAL.parseInt();
			//     REFtraj[i] = ref_deg;
			//   }
			//   digitalWrite(LEDPIN, LOW);
			//   interrupts();        
			//   break;
			// }

			// // load cubic trajectory
			// case 'y':                      
			// {
			// 	noInterrupts();
			//   int i, ref_deg;
			//   BTserial_block();
			//   num_samples = BTSERIAL.parseInt();
			//   digitalWrite(LEDPIN, HIGH);			  
			//   for (i = 0; i < num_samples; i++)
			//   {
			//   	ref_deg = BTSERIAL.parseInt();
			//     REFtraj[i] = ref_deg;
			//   }
			//   digitalWrite(LEDPIN, LOW);
			//   interrupts();        
			//   break;
			// }

			// // execute trajectory
			// case 'o':                      
			// {
			//   noInterrupts();
			//   delay(1000);
			//   e_yaw_prev = 0;
			//   Eint = 0;
			//   control_sig = 0;

			//   // Track, then hold:
			//   set_mode(TRACK);
			//   interrupts();
			//   while (get_mode() == TRACK) {;}
			//   BTSERIAL.write("Now in HOLD mode.\r\n");
			//   while (get_mode() == HOLD) {;}

			//   // // Send plot data to MATLAB:
			//   // noInterrupts();
			//   // char outstr[15];       
			//   // sprintf(outstr, "%d\r\n", num_samples);
			//   // BTSERIAL.write(outstr);
			//   // uint8_t i = 0;
			//   // for (i = 0; i < num_samples; i++)
			//   // {
			//   // 	sprintf(outstr, "%d %d\r\n", REFtraj[i], SENtraj[i]);
			//   //   BTSERIAL.write(outstr);
			//   // }
			//   // interrupts();
			  
			//   break;
			// }

	  	// read and print sensor angles from Lighthouse
	  	case 'l':
	  	{
	  		noInterrupts();

	  		uint32_t time_now[] = {0, 0, 0};
	  		uint32_t time_prev[] = {0, 0, 0};
	  		uint32_t ts[] = {0, 0, 0};
	  		uint32_t tl[] = {0, 0, 0};
	  		bool state_now[] = {HIGH, HIGH, HIGH};
	  		bool state_prev[] = {HIGH, HIGH, HIGH};
	  		char pulse_type_now[] = {'?', '?', '?'};
	  		char pulse_type_prev[] = {'?', '?', '?'};
	  		bool flag[] = {false, false, false};		// flags go true when angles are collected

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

			case 't':                      // Tune gains from disturbance response
			{
			  noInterrupts();
			  delay(1000);
			  e_yaw_prev = 0;
			  Eint = 0;
			  control_sig = 0;
			  ang_target = read_yaw();                   
			  set_mode(HOLD);
			  delay(1000);
			  interrupts();
			  while (get_mode() == HOLD)
			  {
					;
			  }
			  delay(1000);
			  break;
			}

			default:
			{
			  digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate an error
			  break;
			}
	  }
	}
}
