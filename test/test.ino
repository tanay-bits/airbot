#include <Servo.h>

#define LEDPIN 13
#define BTSERIAL Serial2
#define IMUSERIAL Serial1
#define NUM_SENSORS 3
#define BUFLEN 256

const int PRINTAFTER = 50000;
const int MAX_SPEED = 160;    // max ESC write value
const int CONTROL_PERIOD_MS = 22;
const uint8_t SIG[] = {4, 5, 6};
const float deg_per_us = 0.0216;	// (180 deg) / (8333 us)

bool startup = false;
union u_tag {
  byte b_angle[4];
  float f_angle[1];
} u;
typedef enum {IDLE=0, HOLD=1, READ=2, TRACK=3, VIVE=4} Mode_datatype;
Servo esc0, esc1;
IntervalTimer myTimer;

volatile Mode_datatype mode;                       // declare global var which is the current mode 
volatile int yawTol = 0;                           // yaw error tolerance
volatile int yawNow = 0;                           // current (sensed) yaw
volatile int ang_target = 0;                       // target yaw angle
volatile float Kp = 0.1, Ki = 0, Kd = 2;           // PID gains
volatile float control_sig = 0;                    // yaw control signal (~ motor speed change)
volatile int e_yaw_prev = 0;                       // previous yaw error (for D control)
volatile int Eint = 0;                             // integral (sum) of control error
volatile int EINT_CAP = 50000;                     // execute slow retraction if Eint >= EINT_CAP
volatile int vals[2] = {0, 0};                     // w0 and w1 (motor speeds)

uint32_t time_now[] = {0, 0, 0};
uint32_t time_prev[] = {0, 0, 0};
uint32_t ts[] = {0, 0, 0};
uint32_t tl[] = {0, 0, 0};
bool state_now[] = {HIGH, HIGH, HIGH};
bool state_prev[] = {HIGH, HIGH, HIGH};
char pulse_type_now[] = {'?', '?', '?'};
char pulse_type_prev[] = {'?', '?', '?'}; 

//////////////////////////////////////////////////////////////////////////////////////////////////////

class RingBuf
{
  private:
    uint8_t id;           // object identifier (unique for each sensor)
    short readPos;          // index to read at
    short writePos;         // index to write at
    uint32_t timeStamps[BUFLEN];  // array to store time-stamps 
    bool states[BUFLEN];      // array to store states
    float h_angle;          // current horizontal angle
    float v_angle;          // current vertical angle

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

//////////////////////////////////////////////////////////////////////////////////////////////////////

void controller(void)
{
	int e_yaw, yawTarget;

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
		  break;
		}

		case VIVE:
		{
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

//////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
	delay(3000);  	// give enough time for Razor to auto-reset
	noInterrupts();	// disable interrupts

	pinMode(SIG[0], INPUT);
	pinMode(SIG[1], INPUT);
	pinMode(SIG[2], INPUT);

	for(uint8_t i = 0; i < NUM_SENSORS; i++) 
	{
		bufs[i].setID(i);
	}

	// initialize serial channels
	BTSERIAL.begin(9600);
	IMUSERIAL.begin(58824);
	// IMUSERIAL.begin(57600);

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

	interrupts();	//enable interrupts

	// controller to run every 22ms (IMU sensing is at 20ms)
	myTimer.begin(controller, CONTROL_PERIOD_MS * 1000);

	attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
}

// unsigned int ctr = 0;       // counter for printing
void loop() {
	// Keep updating Lighthouse angles
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		// wait until at least two data packets are in the queue
		while (bufs[i].buffer_empty()) {;}
		// BTSERIAL.println("Hi!");
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

	// // print for debugging
	// if (++ctr % PRINTAFTER == 0)
	// {
	//   Serial2.print("H angles: ");
	//   for (byte j = 0; j < NUM_SENSORS; j++)
	//   {
	//     Serial2.print(bufs[j].get_h_angle());
	//     Serial2.print(" ");
	//   }
	//   Serial2.println();
	//   Serial2.print("V angles: ");
	//   for (uint8_t j = 0; j < NUM_SENSORS; j++)
	//   {
	//     Serial2.print(bufs[j].get_v_angle());
	//     Serial2.print(" ");
	//   }
	//   Serial2.println();
	//   Serial2.println();
	// }

	// Wait for user to send something to indicate the connection is ready
	if (!startup)
	{
		if (BTSERIAL.available())
		{
			startup = true;
			BTSERIAL.println("Start up successful");
			BTSERIAL.println();
			BTserial_clear();  // Empty the input buffer
		}
	  }

	// Keep polling for user input
	if (startup)
	{
		if (BTSERIAL.available())
		{
			char input = BTSERIAL.read();
			digitalWrite(LEDPIN, LOW);
		  
			switch (input)
			{
				// stop motors and reset
				case 'q':                      
				{
					noInterrupts();

					esc0.write(0);
					esc1.write(0);
					startup = false;
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
					}
					BTSERIAL.println("Ramp-up to equilibrium complete.");  
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
						case VIVE:
						{
							BTSERIAL.println("VIVE mode");
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
					ang_target = correct_yaw(yawNow + yawChange);          
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
						// ;
					}
					BTSERIAL.println("You're in IDLE mode");
					BTSERIAL.println();
					delay(1000);
					break;
				}

				// read and print sensor angles from Lighthouse
				case 'l':
				{
					noInterrupts();
					delay(1000);
					unsigned short out_counter = 1;  // counter for when to print
					set_mode(VIVE);
					BTSERIAL.println("You're in VIVE mode");
					delay(1000);
					BTserial_clear();
					interrupts();
					while (get_mode() == VIVE)
					{
						if (++out_counter % PRINTAFTER == 0)
						{
							BTSERIAL.print("H angles: ");
							for (uint8_t j = 0; j < NUM_SENSORS; j++)
							{
								BTSERIAL.print(bufs[j].get_h_angle());
								BTSERIAL.print(" ");
							}
							BTSERIAL.println();
							BTSERIAL.print("V angles: ");
							for (uint8_t j = 0; j < NUM_SENSORS; j++)
							{
								BTSERIAL.print(bufs[j].get_v_angle());
								BTSERIAL.print(" ");
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
					break;
				}
			}
			digitalWrite(LEDPIN, HIGH);
		}						
	}
}