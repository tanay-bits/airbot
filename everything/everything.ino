#include <Servo.h>

#define PRINTAFTER 500

///////////////////////////////////////////////////////////////////////////////////////

#define IMUSERIAL Serial1
#define BTSERIAL Serial2
#define LEDPIN 13

const int MAX_SPEED = 160;    		// max ESC write value
const int CONTROL_PERIOD_MS = 22;	// controller fires up every these many milliseconds
//const int CONTROL_FREQ = 45;  		// approximate control frequency (Hz)

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
typedef enum {IDLE=0, READ=1, HOLD=2, TRACK=3, VIVE=4} Mode_datatype;                                  

volatile Mode_datatype mode;                       // declare global var which is the current mode 
volatile bool startup = false;
volatile bool synched = false;
volatile int yawTol = 0;                           // yaw error tolerance
volatile int yawNow = 0;                           // current (sensed) yaw
volatile int yawTarget = 0;                        // target yaw position
volatile float Kp = 0.1, Ki = 0, Kd = 2;           // yawdot = Awy * (wu - wnom)
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_yaw_prev = 0;                       // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int EINT_CAP = 50000;                     // execute slow retraction if Eint >= EINT_CAP
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)
volatile int REFtraj[500];                         // stores reference trajectory
volatile int ctr = 0;                              // counter to step through REFtraj array
volatile int refSize;                              // useful length of REFtraj array

///////////////////////////////////////////////////////

#define NUM_SENSORS 3
#define BUFLEN 8

const byte SIG[] = {4, 5, 6};		// pins to which the sensors are hooked
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)
 

class RingBuf
{
	private:
		int id;										// object identifier (unique for each sensor)
		short readPos;						// index to read at
		short writePos;					// index to write at
		uint32_t timeStamps[BUFLEN];	// array to store time-stamps	
		bool states[BUFLEN];				// array to store states
		float h_angle;						// current horizontal angle
		float v_angle;						// current vertical angle

	public:
		void setID(int i) volatile; 
		int getID(void) volatile;
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



////////////////////////////////////////////////////////////////////////

////////////////////
// CONTROLLER ISR //
////////////////////

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
//		  read_lighthouse();
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

/////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////

void setup() {
  delay(3000);  // Give enough time for Razor to auto-reset
  noInterrupts();
  
  pinMode(SIG[0], INPUT);
  pinMode(SIG[1], INPUT);
  pinMode(SIG[2], INPUT);


  // Initialize serial channels:
  BTSERIAL.begin(9600);
//  IMUSERIAL.begin(58824);
  IMUSERIAL.begin(57600);
//  delay(1000);
  
  // Arm the ESC's:
  esc0.attach(16);
  esc1.attach(17);
  esc0.write(vals[0]);
  esc1.write(vals[1]);
  
  // Set Razor output parameters:
  IMUSERIAL.write("#ob");  // Turn on binary output
  IMUSERIAL.write("#o0");  // Turn OFF continuous streaming output
  IMUSERIAL.write("#oe0"); // Disable error message output 

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);  // turn on LED

  interrupts();
  
  // controller to run every 22ms (IMU sensing is at 20ms)
  myTimer.begin(controller, CONTROL_PERIOD_MS * 1000);
}

void loop() {
  // Wait for user to send something to indicate the connection is ready
  if (!startup)
  {
		if (BTSERIAL.available())
		{
		  startup = true;
		  BTSERIAL.println("Start up successful");
		  BTSERIAL.println();
		  digitalWrite(LEDPIN, LOW);  // turn off LED to indicate startup
		  BTserial_clear();  // Empty the input buffer
		}
	}

  if (startup)
  {
		// Keep polling for user input
		if (BTSERIAL.available())
		{
		  char input = BTSERIAL.read();
		  
		  switch (input)
		  {

		  	// read and print sensor angles from Lighthouse
				case 'l':                      
				{
				  noInterrupts();
				  delay(1000);

          
          for(uint8_t i = 0; i < NUM_SENSORS; i++) 
          {
            bufs[i].setID(i);
          }
          
          uint32_t time_now[] = {0, 0, 0};
          uint32_t time_prev[] = {0, 0, 0};
          uint32_t tS[] = {0, 0, 0};
          uint32_t tLH[] = {0, 0, 0};
          uint32_t tLV[] = {0, 0, 0};
          bool state_now[] = {HIGH, HIGH, HIGH};
          bool state_prev[] = {HIGH, HIGH, HIGH};
          char pulse_type_now[] = {'?', '?', '?'};
          char pulse_type_prev[] = {'?', '?', '?'};
          
				  unsigned short out_counter = 1;  // counter for when to print
				  set_mode(VIVE);
				  BTSERIAL.println("You're in VIVE mode");
				  delay(1000);
				  BTserial_clear();
          attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
          attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
          attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
				  interrupts();       
//          delay(5);
				  while (get_mode() == VIVE)
				  {
            for (uint8_t i = 0; i < NUM_SENSORS; i++)
            {
              // wait until at least two data packets are in the queue
              while (bufs[i].buffer_empty() || (bufs[i].get_storedTime(1) == 0)) {;}
          
              time_now[i] = bufs[i].read_time();
              state_now[i] = bufs[i].read_state();
              bufs[i].incrementReadPos();
          
              if ((state_now[i] == HIGH) && (state_prev[i] == LOW)) // valid pulse
              {
                uint32_t delta = time_now[i] - time_prev[i];
                if (delta > 50)     // SYNC pulse
                {
                  pulse_type_now[i] = 'S';
                  tS[i] = time_prev[i];
          
                  // reset other stuff
                  tLH[i] = 0;
                  tLV[i] = 0;
                }
                else if ((delta > 15) && (delta < 25))  // HORIZONTAL LASER pulse
                {
                  pulse_type_now[i] = 'H';
                  tLH[i] = time_prev[i];
                  if (pulse_type_prev[i] == 'S')
                  {
                    float angH = (tLH[i] - tS[i]) * deg_per_us;
                    bufs[i].set_h_angle(angH);
                  }
          
                  // reset stuff
                  tS[i] = 0;
                  tLH[i] = 0;
                  tLV[i] = 0;
                }
                else if ((delta > 5) && (delta < 15))   // VERTICAL LASER pulse
                {
                  pulse_type_now[i] = 'V';
                  tLV[i] = time_prev[i];
                  if (pulse_type_prev[i] == 'S')
                  {
                    float angV = (tLV[i] - tS[i]) * deg_per_us;
                    bufs[i].set_v_angle(angV);
                  }
          
                  // reset stuff
                  tS[i] = 0;
                  tLH[i] = 0;
                  tLV[i] = 0;
                }
                else          // meaningless pulse
                {
                  pulse_type_now[i] = '?';
                  
                  // reset stuff
                  tS[i] = 0;
                  tLH[i] = 0;
                  tLV[i] = 0;
                }
              }
          
              // update previous values
              state_prev[i] = state_now[i];
              time_prev[i] = time_now[i];
              pulse_type_prev[i] = pulse_type_now[i];
            }

            
						out_counter = out_counter + 1;
						if (out_counter % PRINTAFTER == 0)
						{
							BTSERIAL.println("H angles: ");
							for (uint8_t j = 0; j < NUM_SENSORS; j++)
							{
								BTSERIAL.println(bufs[j].get_h_angle());
							}
							BTSERIAL.println("V angles: ");
							for (uint8_t k = 0; k < NUM_SENSORS; k++)
							{
								BTSERIAL.println(bufs[k].get_v_angle());
							}
							BTSERIAL.println();
							BTSERIAL.println();
						}
				  }
         
				  BTSERIAL.println("You're in IDLE mode");
				  BTSERIAL.println();
				  delay(1000);
				  break;
				}

				// stop motors and reset
				case 'q':                      
				{
				  noInterrupts();
				  
				  esc0.write(0);
				  esc1.write(0);
				  startup = false;
				  digitalWrite(LEDPIN, HIGH);  // turn on LED to indicate reset
				  set_mode(IDLE);
				  BTSERIAL.println("You've quit. Press any key to start up again.");
				  BTSERIAL.println();
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
			//            delay(50);  // to slow down the ramp-up
				  }
				  BTSERIAL.println("Ramp-up to equilibrium complete.");
				  BTSERIAL.println();   
				  break;
				}
				
				// get parameters - PID gains, yawTol, EINT_CAP, current mode
				case '?':                      
				{
				  noInterrupts();
				  
				  BTSERIAL.println("PID gains are:");
				  BTSERIAL.println(Kp);
				  BTSERIAL.println(Ki);
				  BTSERIAL.println(Kd);
				  BTSERIAL.println();

				  BTSERIAL.println("yawTol is:");
				  BTSERIAL.println(yawTol);
				  BTSERIAL.println();

				  BTSERIAL.println("EINT_CAP is:");
				  BTSERIAL.println(EINT_CAP);
				  BTSERIAL.println();

				  BTSERIAL.print("You are in ");
				  switch (get_mode())
				  {
						case IDLE:
						{
						  BTSERIAL.println("IDLE mode");
						  break;
						}
						case READ:
						{
						  BTSERIAL.println("READ mode");
						  break;
						}
						case HOLD:
						{
						  BTSERIAL.println("HOLD mode");
						  break;
						}
						case TRACK:
						{
						  BTSERIAL.println("TRACK mode");
						  break;
						}
						default:
						{
						  BTSERIAL.println("no valid mode!??");
						  break; 
						}
				  }
				  BTSERIAL.println();
				  
				  interrupts();
				  break;
				}

				// set PID gains
				case 's':                      
				{
				  noInterrupts();
				  delay(1000);

				  BTSERIAL.println("Enter P gain:");
				  BTserial_block();
				  Kp = BTSERIAL.parseFloat();

				  BTSERIAL.println("Enter I gain:");
				  BTserial_block();
				  Ki = BTSERIAL.parseFloat();

				  BTSERIAL.println("Enter D gain:");
				  BTserial_block();
				  Kd = BTSERIAL.parseFloat();

				  BTSERIAL.println("New gains are:");
				  BTSERIAL.println(Kp);
				  BTSERIAL.println(Ki);
				  BTSERIAL.println(Kd);
				  BTSERIAL.println();
				  
				  interrupts();
				  break;
				}

				// set yawTol
				case 'b':                      
				{
				  noInterrupts();
				  delay(1000);

				  BTSERIAL.println("Enter desired yawTol:");
				  BTserial_block();
				  yawTol = BTSERIAL.parseInt();
				  BTSERIAL.println("New yawTol is:");
				  BTSERIAL.println(yawTol);
				  BTSERIAL.println();
				  
				  interrupts();
				  break;
				}

				// set EINT_CAP for triggering retraction
				case 'c':                      
				{
				  noInterrupts();
				  delay(1000);

				  BTSERIAL.println("Enter desired EINT_CAP:");
				  BTserial_block();
				  EINT_CAP = BTSERIAL.parseInt();
				  BTSERIAL.println("New EINT_CAP is:");
				  BTSERIAL.println(EINT_CAP);
				  BTSERIAL.println();
				  
				  interrupts();
				  break;
				}

				// read and print current yaw (deg)
				case 'r':                      
				{
				  noInterrupts();
				  delay(1000);
				  unsigned short out_counter = 1;  // counter for when to print
				  set_mode(READ);
				  BTSERIAL.println("You're in READ mode");
				  delay(1000);
				  BTserial_clear();
				  interrupts();
				  while (get_mode() == READ)
				  {
						out_counter = out_counter + 1;
						if (out_counter % PRINTAFTER == 0)
						{
						  BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
						}
				  }
				  BTSERIAL.println("You're in IDLE mode");
				  BTSERIAL.println();
				  delay(1000);
				  break;
				}

				// manually change motor speeds
				case 'm':
				{
				  noInterrupts();
				  delay(1000);
				  BTSERIAL.println("Enter motor speeds separated by whitespace:");
				  BTserial_block();
				  vals[0] = BTSERIAL.parseInt();
				  vals[1] = BTSERIAL.parseInt();
				  BTSERIAL.println("Speeds:");
				  BTSERIAL.print(vals[0]);
				  BTSERIAL.print(" ");
				  BTSERIAL.println(vals[1]);
				  esc0.write(vals[0]);
				  esc1.write(vals[1]);
				  set_mode(IDLE);
				  BTSERIAL.println("You're in IDLE mode");
				  BTSERIAL.println();
				  delay(1000);
				  interrupts();
				  break;
				}

				// go to target yaw and hold
				case 'h':
				{
				  noInterrupts();
				  delay(1000);
				  unsigned short out_counter = 1;  // counter for when to print
				  e_yaw_prev = 0;
				  Eint = 0;
				  control_sig = 0;
				  BTSERIAL.println("Enter desired yaw change (deg):");
				  BTserial_block();
				  int yawChange = BTSERIAL.parseInt();
				  yawNow = read_yaw();                   
				  yawTarget = correct_yaw(yawNow + yawChange);          
				  BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
				  BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
				  set_mode(HOLD);
				  BTSERIAL.println("You're in HOLD mode");
				  delay(1000);
				  interrupts();
				  while (get_mode() == HOLD)
				  {
						out_counter = out_counter + 1;
						if (out_counter % PRINTAFTER == 0)
						{
						  // BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
						  // BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
						  BTSERIAL.println(Eint);
						}
				  }
				  BTSERIAL.println("You're in IDLE mode");
				  BTSERIAL.println();
				  delay(1000);
				  break;
				}

				case 't':                      // Tune gains from disturbance response
				{
				  noInterrupts();
				  delay(1000);
				  unsigned short out_counter = 1;  // counter for when to print
				  e_yaw_prev = 0;
				  Eint = 0;
				  control_sig = 0;
				  yawNow = read_yaw();
				  yawTarget = yawNow;                   
				  set_mode(HOLD);
				  BTSERIAL.println("You're in HOLD (tune) mode. Disturb and observe.");
				  delay(1000);
				  interrupts();
				  while (get_mode() == HOLD)
				  {
						out_counter = out_counter + 1;
						if (out_counter % PRINTAFTER == 0)
						{
						  // BTSERIAL.print("Current yaw: "); BTSERIAL.println(yawNow);
						  // BTSERIAL.print("Target yaw: "); BTSERIAL.println(yawTarget);
						  BTSERIAL.println(Eint);
						}
				  }
				  BTSERIAL.println("You're in IDLE mode");
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
}
