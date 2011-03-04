#include <pic.h>
	__CONFIG(11111110110010);

/***********************************************************
Solar Tracker Code Revised


PORTB 1-4 are the LED's that display the mode of operation
		LED 3 - manual operation (red LED)
		LED 2 - auto operation (amber LED)
		LED 1 - moving left (green LED)
		LED 0 - moving right (green LED)
		LED 1 & 2 blinking - centered, not moving 
		LED 3 blinking - low light/cloudy

PORTC 4-6 are control buttons
		bit 4 - toggles between auto and manual modes
		bit 5 - move left (manual mode only)	
		bit 6 - move right (manual mode only)	

PORTC 1,7 are inputs from saftey sensors to prevent 
		concentrator from rotating too far; disconnect and
		ground bits 1 and 7 to exclude this feature 

PORTA 0-4 are sensor inputs S1 to S5

Note: This version uses S2 to track, S4 for sensing ambient light

***********************************************************/

#define PORTBIT(adr, bit)			((unsigned)(&adr)*8+(bit))

// defining motor control bits
	static bit	brake @				PORTBIT(PORTC,0);
//	static bit	PWM @				PORTBIT(PORTC,2);
	static bit	direction @			PORTBIT(PORTC,3);

// defining button input bits
	static bit	startButton @			PORTBIT(PORTC,4);
	static bit	leftButton @			PORTBIT(PORTC,5);
	static bit	rightButton @			PORTBIT(PORTC,6);

// defining saftey sensor inputs
	static bit 	sunrise @				PORTBIT(PORTC,1);
	static bit 	sunset @				PORTBIT(PORTC,7);
	
// Variable Declarations
	unsigned char S1 = 0;			// holds sensor 1 avg reading 
	unsigned char S2 = 0;			// holds sensor 2 avg reading
	unsigned char S3 = 0;			// holds sensor 3 avg reading
	unsigned char S4 = 0;			// holds sensor 4 avg reading
	unsigned char S5 = 0;			// holds sensor 5 avg reading
	
	unsigned char n = 64;			// number of readings to average
	unsigned long sum = 0;			// accumulator for sensor reading
	unsigned char count = 0;		// number of times sky was scanned	
//	char error = 0;					// error between S2 and S4
	unsigned char speed = 220;		// set default motor speed
	
	unsigned char dark = 170;		// night/dark conditions
	unsigned char lowLight = 210;	// low light/cloudy conditions
	unsigned char threshold = 245;	// threshold for centering
		
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
// function prototypes

void initPorts();
void delay (unsigned long k);
void lightCenter ();
void lightRight ();
void lightLeft();
void moveMotor (char D);
void manual ();
void automatic ();
void initAtoD();
void averageSensor ();
void stop (char R);
void flashState ();
void reset ();

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF



//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void main ()					// main function
{
	initPorts();				// initialize ports
	TMR2ON = 1;
	brake = 1;					// make sure brake is applied

	while (1==1)				// main loop
	{
		manual ();				// default starting mode is manual

		automatic ();			// go to auto after exiting manual
	}

}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Port initialization

void initPorts()
{
	PORTC = 0b00000000;		// clear used ports
	PORTB = 0b00000000;
	PORTA = 0b00000000;
	
	TRISA = 0b11111111;		// make all the Port A as input for sensors
	TRISB = 0b00000000;		// bits 1-4 are state LEDs, rest are unused
	TRISC = 0b11110010;		// bits 0,2,3 are movement buttons 
							// bits 4,5,6 are motor controls
							
	ADCON0 = 0b00001001;	// set up for fast operation (fosc/2)
	ADCON1 = 0b00000010;	// set up A/D for Port A and Port E is digital


	CCP1CON = 0b00001100;	//sets CCP module for PWM operation
	T2CON = 0b00000100;		//sets timer 2 on with a 1 prescale
	PR2 = 0xFF;				//sets max period
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


// Resets all tracking variables
//
void reset ()
{
	dark = 170;		
	lowLight = 210;
	threshold = 245;
	count = 0
}


// variable delay function
//
void delay (unsigned long k)
{
	unsigned long i;
	for (i=0; i<k && startButton==1; i++) {}
}


// Slowly flashes both green LEDs, signaling that concentrator
// is centered
void lightCenter ()
{
	delay(40000);
	PORTB = PORTB | 0b00000011;		// turn on bit 0,1
	delay(40000);
	PORTB = PORTB & 0b11111100;		// turn off bit 0,1
}


// Slowly flashes amber LED, signaling low light conditions
//
void flashState ()
{
	delay(40000);
	PORTB = 0b00000000;
	delay(40000);
	PORTB = 0b00000100;
}	


// Flashes one green LED, signaling that concentrator is
// moving toward right
void lightRight ()
{
	delay(15000);
	PORTB = PORTB | 0b00000001;		// turn on bit 0
	delay(15000);
	PORTB = PORTB & 0b11111110;		// turn off bit 0
}


// Flashes other green LED, signaling that concentrator is
// moving toward left
void lightLeft ()		// flashes other green LED	
{
	delay(15000);
	PORTB = PORTB | 0b00000010;		// turn on bit 1
	delay(15000);
	PORTB = PORTB & 0b11111101;		// turn off bit 1
}


// Flashes all LEDs, signaling that concentrator has reached
// boundaries of allowed movement
void lightAll ()
{
	char state = PORTB & 0b00001100;	// storing state
	
	delay(15000);
	PORTB = 0b00011111;				// turn on bit 1,2
	delay(15000);
	PORTB = state;					// turn off bit 1,2
}


// Releases brake and runs motor; D determines direction;
// 1 is forward, 0 is reverse
void moveMotor (char D)	
{
						
	direction = D;			// set direction
	
// check if moving beyond set boundaries
	if ((sunset && D==1) || (sunrise && D==0))
	{
		brake = 1;
		lightAll ();
	}
	else
	{
		brake = 0;
	}

	CCPR1L = speed;			// set motor speed
}



// Manual operation of concentrator; use buttons to control
//
void manual ()		
{
	PORTB = 0b00001000;		// light first LED to indicate manual

	while (startButton==1)	// manual control mode until button pressed
	{
// state M1 - moving left
		while (leftButton==0 && startButton==1)	// left button pressed, move left
		{
			moveMotor(1);
			lightLeft();
		}

// state M2 - moving right
		while (rightButton==0 && startButton==1) // right button pressed, move right
		{
			moveMotor(0);
			lightRight();
		}

		brake=1;			// if not moving make sure brake is applied
	}
	
	while (startButton==0) {}	// pause until button release
	delay (1000);	
}


// Concentrator is automatically tracking
//
void automatic ()
{
	PORTB = 0b00000100;		// light second LED to indicate auto		
	count = 0;

	initAtoD ();			// get initial readings

	while (startButton==1)
	{
// night condition, reset values, don't move
		while (S4 <= dark && startButton==1)
		{
			stop (0);
			initAtoD ();
		}

// low light or cloudy condition, don't move
		while (S4 <= lowLight && S4 > dark && startButton==1)
		{
			stop (1);
			initAtoD ();
		}

// not low light condition, but cannot track
		if (count >=3 && threshold > 200)
		{
			count = 0;
			threshold -= 15;	// decrease threshold, try again
			delay (200000);
		}
		
// scan toward sunset position		
		if (!sunset && count < 3)
		{
			count++;
			while (!sunset && S4 > lowLight && startButton==1)
			{
				if (S2 > threshold)
				{
					stop (2);
				}
				else
				{
					lightLeft ();
					moveMotor (1);
				}
				initAtoD ();
			}
		}
		brake = 1;

// scan toward sunrise position, exits on center
		if (!sunrise && count < 3)
		{
			count++;
			while (!sunrise && S4 > lowLight && S2 <= threshold && startButton==1)
			{
				lightRight ();
				moveMotor (0);
				initAtoD ();
				if (S2 > threshold)
				{
					count = 0;	// centered, reset count
				}
			}
		}
		brake = 1;

		initAtoD ();
	}

	brake = 1;					// make sure motor stops
	while (startButton==0) {}	// pause until button release
	delay (1000);	
}


// Stops the motor, flashes light fors a while 
// R = 0; night or dark conditions
// R = 1; low light or cloudy conditions
// R = 2; tracker is centered (probably not cloudy)
void stop (char R)
{
	int i = 0;
	brake = 1;
	
	if (R == 0)
	{
		reset ();
		for (i=0; i<100 && startButton==1; i++)
		{
			flashState ();
		}
	}
	else if (R == 1)
	{
		count = 0;
		for (i=0; i<10 && startButton==1; i++)
		{
			flashState ();
		}
	}			
	else 
	{
//		lowLight =  S4 - 5;			// set low light condition
		lowLight = S4 - S1/10;		// set low light condition
		count = 0;
		for (i=0; i<10 && startButton==1; i++)
		{
			lightCenter ();
		}
	}

}


// Reads voltages from sensors and converts them to 8-bit binary;
// sensors 3,5 are not used in this version
// sensor 1 is a pot for setting low light conditions
void initAtoD()
{

	ADCON0 = 0b00000001;			// do A/D for sensor 1
	averageSensor ();
	S1 = sum/n;
	
	ADCON0 = 0b00001001;			// do A/D for sensor 2
	averageSensor ();
	S2 = sum/n;
/*	  
	ADCON0 = 0b00010001;			// do A/D for sensor 3	
	averageSensor ();
	S3 = sum/n;
*/	  
	ADCON0 = 0b00011001;			// do A/D for sensor 4	
	averageSensor ();
	S4 = sum/n;
/*	  
	ADCON0 = 0b00100001;			// do A/D for sensor 5
	averageSensor ();
	S5 = sum/n;

	error = S2 - S4;
*/
}


// Averages n readings from a particular sensor
//
void averageSensor ()
{
	unsigned char i = 0;

	sum = 0;
	ADGO = 1;
	while (i < n)
	{							
		if (ADGO==0)
		{
			sum += ADRES;
			i++;
			ADGO = 1;
		}
	}
}

