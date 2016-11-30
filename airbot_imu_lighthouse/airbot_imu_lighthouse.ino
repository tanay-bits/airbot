/* This file is part of the AirBot firmware */

#include "lighthouse.h"
#include "yaw_control.h"

#define PRINTAFTER 500


void setup() {
  delay(3000);  // Give enough time for Razor to auto-reset
  noInterrupts();
  
  pinMode(SIG[0], INPUT);
  pinMode(SIG[1], INPUT);
  pinMode(SIG[2], INPUT);

  for(int i = 0; i < NUM_SENSORS; i++) 
  {
  	bufs[i].setID(i);
  }

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

  attachInterrupt(digitalPinToInterrupt(SIG[0]), isrA_lighthouse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIG[1]), isrB_lighthouse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIG[2]), isrC_lighthouse, CHANGE);
  
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
				  unsigned short out_counter = 1;  // counter for when to print
				  set_mode(VIVE);
				  BTSERIAL.println("You're in VIVE mode");
				  delay(1000);
				  BTserial_clear();
				  interrupts();
				  while (get_mode() == VIVE)
				  {
						out_counter = out_counter + 1;
						if (out_counter % PRINTAFTER == 0)
						{
							BTSERIAL.print("H angles: ");
							for (byte j = 0; j < NUM_SENSORS; j++)
							{
								BTSERIAL.print(bufs[j].get_h_angle());
								BTSERIAL.print(" ");
							}
							BTSERIAL.print("\nV angles: ");
							for (int j = 0; j < NUM_SENSORS; j++)
							{
								BTSERIAL.print(bufs[j].get_v_angle());
								BTSERIAL.print(" ");
							}
							BTSERIAL.print("\n");
							BTSERIAL.print("\n");
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