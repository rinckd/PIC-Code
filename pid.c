#include <pic.h>

__CONFIG(11111110110010);
/*************************************************************
* Case Study 6. DC Motor
*  
* Port A	Pin 1			Analog Tachometer
* Port A	Pin 4			Black Button
* Port B	Pin 0-7			LEDs
* Port C	Pin 0			Motor Direction
* Port C 	Pin 1			Brake
* Port C	Pin 2			PWM
* Port C 	Pin 3			Eddy Sensor
* Port C	Pin 4,5			Octal Switch
* Port C	Pin 6,7			Encoders
*
* state Register
* bit 0-1 	Mode Bits
*************************************************************/
#define CLOCKWISE			1
#define COUNTERCLOCKWISE		0
#define VREF				160 // starting velocity reference
#define BIAS				160  // bias
#define KP				12 	// proportional constant
#define KD				4 	// derivative controller gains
#define KI				2	// integral controller gain

#define GetBit(var, bit) ((var & (1 << bit)) != 0) // Returns true/false if bit is set
#define SetBit(var, bit) (var |= (1 << bit))
#define FlipBit(var, bit) (var ^= (1 << bit))
#define ClearBit(var, bit) (var &= ~(1 << bit))

#define PORTBIT(adr, bit)		((unsigned)(&adr)*8+(bit))
static bit 	tachometer @ 	PORTBIT(PORTA,1);
static bit	blackButton @	PORTBIT(PORTA,4);
static bit	direction @	PORTBIT(PORTC,0);
static bit	brake @		PORTBIT(PORTC,1);
static bit	pwm @		PORTBIT(PORTC,2);
static bit 	eddy @		PORTBIT(PORTC,3);
static bit 	octal1 @		PORTBIT(PORTC,4);
static bit 	octal2 @		PORTBIT(PORTC,5);
static bit	encoderB @	PORTBIT(PORTC,6);
static bit 	encoderA @	PORTBIT(PORTC,7);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
// Function Declarations

void InitPorts(void);
void InitAtoD(void);
void MoveMotor (char D, char speed);	
void Increase (void);
void Constant(void);
void Decrease(void);
void Control(void);
void EddyStart (void);
void ModeChange(void);
void Delay (unsigned long timer);
void SetupDelay(void);
void FlashLED (void);
void EncoderTest (void);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

char ADcounter = 0;		// Sample Counter
char state, speed, i, eddyCount;

unsigned long sum, avg, encoderCount, rotationCount;
long error, previousError, feedback;
bit encoderToggle, eddyToggle;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void main ()			// main function
{
	InitPorts();		// initialize ports
	EddyStart();		// run start routine (4 rotations @ constant speed)
	while(1)
	{
		if (blackButton)	// start routine if black button is pressed
		{
			while(blackButton){}	// do nothing while black is pressed
			ModeChange();			// find the control mode
			Increase();				// ramp up until V=190
			Constant();				// hold constant for 60 rotations
			Decrease();				// ramp down to initial velocity
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/* Port initialization */

void InitPorts()
{
	PORTA=0B00000000;		// clear ports
	PORTB=0B00000000;
	PORTC=0B00000000;
	
	TRISA=0B00010010;		// Port A inputs at RA1, RA4
	TRISB=0B00000000;		// Port B LEDs all output
	TRISC=0B11111000;		// Port C motor outputs at pins 0-2, inputs at pins 3-7
	
	ADCON0 = 0b00001001;	// set up for fast operation (fosc/2)
	ADCON1 = 0B00000000;	// RA1 is analog?	
	
	PR2 = 0b11111111;		// max PWM motor frequency is 20khz
	T2CON =0B0000100;		// Timer 
	TMR2ON = 1;			// Start Timer
	
	CCP1CON = 0b00111100;	// Sets PWM on, holds LSb of duty cycle
	CCPR1L=0B00000000;		// duty cycle - sets speed (initiate at zero, adjust this later to adjust speed)

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void InitAtoD(void)
{
	SetupDelay();		// AD setup delay
	PEIE=1;				// enable peripheral interrupts
	GIE=1;				// enable global interrupts
	ADIE=1;				// enable AD interrupts
	ADGO=1;				// start AD conversion
}
void MoveMotor (char D, char speed)	
{						
	direction = D;		// set direction
	brake = 0;			// make sure brake is off
	CCPR1L = speed;		// set motor speed by adjusting duty cycle
}
void Increase()
{
	avg=0;					// reset avg
	error=0;				// reset error
	speed = VREF;			// start at VREF
	rotationCount=0;		// reset rotation counter at beginning of routine
	previousError=0;
	while (speed<190)		// ramp up until speed is at 190
	{
		InitAtoD();					// get Tachometer feedback
		Control();					// use feedback to control speed
		while (encoderCount <=100)	
		{
		//	InitAtoD();
			MoveMotor(CLOCKWISE, feedback);
			if ((encoderA==1) && (encoderToggle==0))
			{
				encoderCount++;		// increment encoderCount for each encoder signal
				encoderToggle=1;	// make sure it only increments once for each encoder signal
			}
			if (encoderA==0)
			{
				encoderToggle=0;
			}
		}
		encoderCount = 0;			// reset encoder count after 100 counts(=one revolution)
		speed++;					// increment speed every revolution
		rotationCount++;			// count revolutions
	}
}

void Constant ()
{
	rotationCount=0;				// reset rotation counter at beginning of routine
	while (rotationCount <=60)		// hold constant speed for 60 rotations
	{
		InitAtoD();					// get Tachometer feedback
		Control();					// use feedback to control speed
		MoveMotor(CLOCKWISE, feedback);
		while (encoderCount <=100)
		{
			if ((encoderA==1) && (encoderToggle==0))
			{
				encoderCount++;		// increment encoderCount for each encoder signal
				encoderToggle=1;	// make sure it only increments once for each encoder signal
			}
			if (encoderA==0)
			{
				encoderToggle=0;
			}
		}
		encoderCount = 0;			// reset encoder count after 100 counts (=one revolution)
		rotationCount++;			// count revolutions
	}
}
void Decrease()
{
	rotationCount=0;				// reset rotation counter at beginning of routine
	while (avg>150)				// ramp down until speed is back to VREF2
	{
		InitAtoD();					// get Tachometer feedback
		Control();					// use feedback to control speed
		while (encoderCount <=100)
		{
			MoveMotor(CLOCKWISE, feedback);
			if ((encoderA==1) && (encoderToggle==0))
			{
				encoderCount++;		// increment encoderCount for each encoder signal
				encoderToggle=1;	// make sure it only increments once for each encoder signal
			}
			if (encoderA==0)
			{
				encoderToggle=0;
			}
		}
		encoderCount = 0;			// reset encoder count after 100 counts (=one revolution)
		speed--;					// decrement speed every revolution
		rotationCount++;			// count revolutions
	}	
	brake=1;			// apply brake after trapazoid is done
	MoveMotor(CLOCKWISE, 0); // turn off motor
	FlashLED();			// flash LEDs when routine is done
}
void Control()
{
	switch(state)
	{
		case 0:
			feedback=KP*error;		// proportional control: error*k
			break;
		case 1:
			feedback=((KP*error)+BIAS);	// bias control: (error*k)-bias
			break;
		case 2:
			feedback=(KP*error)+(KD*(previousError-error));
			MoveMotor(CLOCKWISE, 255); //test
			break;
		case 3:
			feedback = (KP*error)+(KD*(previousError-error))+(KI*(previousError+error));
			break;
	}
	if (feedback>255) 
	{	
		feedback=255;		// max speed is 255
	}
	if (feedback<0)
	{
		feedback=0;
	}
	previousError=error;
}

void EddyStart()		// Start routine - rotate disk slowly four times
{
	char eddyCount=0;		// Set eddyCount
	eddyToggle=0; 			// "bit eddyToggle" generated an error that bit variables needed to be global --?
	brake = 0;		 	// make sure brake is not applied
	while(eddyCount<=4) 
	{
		MoveMotor(CLOCKWISE, VREF); // move motor slowly
		PORTB=eddyCount;
		if ((eddy==1) && (eddyToggle==0))
		{
			eddyCount++;			// increment eddyCount for each Eddy signal
			eddyToggle=1;			// make sure it only increments once for each encoder signal
		}
		if (eddy==0) 
		{
			eddyToggle=0;
		}
	}
	brake = 1; 						// apply brake at end of start routine
}

void ModeChange()
{
	state = (octal1+2*octal2);		// read state from octal ports
}
void SetupDelay()
{
	for (i=1; i>0; i--) {}
}
void Delay (unsigned long timer)
{
	unsigned long i;
	for (i=0; i<timer; i++) {}
}
void FlashLED()
{
	for (i=0; i<10; i++)
	{
		PORTB=255;
		Delay(10000);
		PORTB=0;	
		Delay(10000);
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/* Interrupt Routine */

void interrupt ISR()	// AD interrupt routine
{
	if (ADIF)			// each time AD conversion finishes,
	{
		ADcounter++;				// increment ADcounter
		sum+=ADRES;					// Add up all AD results
		if (ADcounter == 64)		// once we've run the AD conversion n times,
		{
			avg = sum / 64;			// average the results			
			error = speed-avg;
			PORTB=error;			// output error to LEDs
			ADcounter = 0;			// reset AD counter
			sum = 0;				// reset sum
			ADIE=0;					// disable AD interrupts	
		}
		ADIF=0;						// reset AD interrupt flag
		ADGO=1;						// restart AD conversion
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void EncoderTest()
{
	rotationCount=0;				// reset rotation counter at beginning of routine
	while (rotationCount <=60)
	{	
		MoveMotor(CLOCKWISE, 255); // move motor slowly
		PORTB=rotationCount;
		while (encoderCount <=100)
			{
				if ((encoderA==1) && (encoderToggle==0))
				{
					encoderCount++;		// increment encoderCount for each encoder signal
					encoderToggle=1;	// make sure it only increments once for each encoder signal
				}
				if (encoderA==0)
				{
					encoderToggle=0;
				}
			}
		encoderCount = 0;			// reset encoder count after 100 counts (=one revolution)
		rotationCount++;
	}
	brake=1;
	MoveMotor(CLOCKWISE, 0); // move motor slowly
}			