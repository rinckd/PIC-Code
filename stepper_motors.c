#include <pic.h> 
__CONFIG(11111110110010); 
/************************************************************* 
* Case Study 5. Stepper Motor 
*
* Port E    Pin 0,1,2       Octal Switch
* Port B    Pin 0,1,2,3     4 LED Lights
* Port B    Pin 4           Optical Interrupter. Unipolar. Horizontal
* Port B    Pin 5           Optical Interrupter. Unipolar. Vertical
* Port B    Pin 6           Optical Interrupter. Bipolar. Vertical
* Port B    Pin 7           Optical Interrupter. Bipolar. Horizontal
* Port C    Pin 0,1         Red / Green Button
* Port D    Pin 0,1,2,3     Unipolar Stepper Motor
* Port D    Pin 4           Bipolar. Phase A
* Port D    Pin 5           Bipolar. Ia
* Port D    Pin 6           Bipolar. Phase B
* Port D    Pin 7           Bipolar. Ib
* Port D    Pin 8           Phase Input: Direction of current through motor winding
*
* State Register
* bit 0-2   Mode Bits
* bit 3     Error Bit
/************************************************************* 
/* Variable Declarations */
#define _40ms               1538
//#define _40ms             1 // Uncomment to debug
#define _1second            38461
#define UNIWAVEDRIVE        0B00010001
#define UNIFULLSTEP         0B10011001
#define BIFULLSTEP          0B10011001
#define BIWAVEDRIVE         0B11100001
#define BIWAVEIO            0B00100010
#define CLOCKWISE           1
#define COUNTERCLOCKWISE    -1


#define GetBit(var, bit) ((var & (1 << bit)) != 0) // Returns true/false if bit is set
#define SetBit(var, bit) (var |= (1 << bit))
#define FlipBit(var, bit) (var ^= (1 << bit))
#define PORTBIT(adr, bit) ((unsigned) (&adr)*8+(bit))

static bit greenButton @ PORTBIT(PORTC,0);
static bit redButton @ PORTBIT(PORTC,1);
static bit uPosVertical @ PORTBIT(PORTB,5);
static bit bPosVertical @ PORTBIT(PORTB,6);
static bit uPosHorizontal @ PORTBIT(PORTB,4);
static bit bPosHorizontal @ PORTBIT(PORTB,7);
char state, bipolarState, unipolarState, mode1Toggle, bstep, ustep;

void Init(void);
void StepperInit(void);
void ModeOne(void);
void ModeTwoAndFour(void);
void ModeThree(void);
void UniStep(char method, signed char direction);
void BiStep(char method, signed char direction);
void Delay(unsigned long timer);
void SetLED(void);
void ClearLED(void);
void Error(void);
void Realign(void); // Realign bipolar stepper after wavedrive

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void main()
{
	Init();
	StepperInit();
	while(1) {
		if (greenButton) {
			while(greenButton){}
			mode1Toggle=0;
			if (state == 4) {
				Realign();
			}
			state = (~PORTE & 0B00000111);
			state=4;
			SetLED();
			if (state == 3) {
//   			If we are in state 3, the motors need to be set accordingly
				if (!bPosHorizontal) {
					while (!bPosHorizontal) {
						BiStep(BIFULLSTEP, COUNTERCLOCKWISE);
					}
				}
				if (!uPosVertical) {
					while (!uPosHorizontal) {
						UniStep(UNIFULLSTEP, COUNTERCLOCKWISE);
					}
				}
			}
			else {
//   			Otherwise they should go to normal home positions
				if (!uPosHorizontal) {
					while (!uPosHorizontal) {
						UniStep(UNIFULLSTEP, CLOCKWISE);
					}
				}
				if (!bPosHorizontal) {
					while (!bPosHorizontal) {
						BiStep(BIFULLSTEP, COUNTERCLOCKWISE);
					}
				}
			}
		}
		else if (redButton) {
//  		If red button is pushed, what mode are we in?
			while(redButton){}
			switch(state) {
				case 1:
					ModeOne();
					break;
				case 2:
					ModeTwoAndFour();
					break;
				case 3:
					ModeThree();
					break;
				case 4:
					ModeTwoAndFour();
					break;
				default:
					Error();
					break;
			}
		}
	}
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/* Port Initialization */
void Init()
{
	PORTD = 0B00000000; // clear portd
	PORTB = 0B00000000;
	TRISB = 0B11110000;
	TRISD = 0B00000000; // Port D. all output
	TRISC = 0B11111111; // Port C. all input
	TRISE = 0B00000111; // Port E. Pins 0-2 input
	ADCON1 = 0B00000110; // Need to set ADCON1 for Octal Switch
	state = 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/* Stepper Motor Initialization */
void StepperInit()
{
	//Motors Init
	PORTD=0B01001100;
	Delay(_40ms);
	PORTD=0B00000110;
	Delay(_40ms);
	PORTD=0B00010011;
	Delay(_40ms);
	PORTD=0B01011001;
	Delay(_40ms);
	bipolarState=0;
//	Move bipolar to Home position
	while(!bPosHorizontal) {
		BiStep(BIFULLSTEP, CLOCKWISE);
	}
// 	Move unipolar to Home position
	unipolarState=0;
	while(!uPosHorizontal) {
		UniStep(UNIFULLSTEP, CLOCKWISE);
	}
}
void ModeOne()
{
// 	mode1 toggle remembers what sequence to do next
	mode1Toggle++;
	switch(mode1Toggle)
	{
		case 1:
			while (!uPosVertical) {
				UniStep(UNIFULLSTEP, COUNTERCLOCKWISE);
			}
			break;
		case 2:
			while (!bPosVertical) {
				BiStep(BIFULLSTEP, CLOCKWISE);
			}
			break;
		case 3:
			while (!uPosHorizontal) {
				UniStep(UNIFULLSTEP, CLOCKWISE);
			}
			break;
		case 4:
// 			Last step. Set mode1Toggle to 0 reset next round
			mode1Toggle=0;
			while (!bPosHorizontal) {
				BiStep(BIFULLSTEP, COUNTERCLOCKWISE);
			}
			break;
	}
}
void ModeTwoAndFour()
{
	if (state==2) {
		bstep=BIFULLSTEP;
		ustep=UNIFULLSTEP;
	}
	else {
// 		This routine realigns steps in terms of wavedrive
		if (bipolarState == 1) {
			bipolarState=2;
		}
		if (bipolarState == 2) {
			bipolarState=4;
		}
		if (bipolarState == 3) {
			bipolarState=6;
		}
		bstep=BIWAVEDRIVE;
		ustep=UNIWAVEDRIVE;
	}
	while (!redButton) {
		if (uPosHorizontal) {
			while (!bPosVertical) {
				BiStep(bstep, CLOCKWISE);
				if (!uPosVertical) {
					UniStep(ustep, COUNTERCLOCKWISE);
				}
			}
		}
		if (uPosVertical) {
			while (!bPosHorizontal) {
				BiStep(bstep, COUNTERCLOCKWISE);
				if (!uPosHorizontal) {
					UniStep(ustep, CLOCKWISE);
				}
			}
		}
	}
	while (redButton) {}
}
void ModeThree()
{
	while (!redButton) {
		if (uPosVertical) {
			while (uPosVertical || bPosHorizontal) {
				BiStep(BIFULLSTEP, COUNTERCLOCKWISE);
				UniStep(UNIFULLSTEP, COUNTERCLOCKWISE);
			}
			while (!uPosHorizontal) {
				BiStep(BIFULLSTEP, COUNTERCLOCKWISE);
				if (!uPosHorizontal) {
					UniStep(UNIFULLSTEP, COUNTERCLOCKWISE);
				}
			}
		}
		if (uPosHorizontal) {
			while (uPosHorizontal || bPosVertical) {
				BiStep(BIFULLSTEP, CLOCKWISE);
				UniStep(UNIFULLSTEP, CLOCKWISE);
			}
			while (!bPosHorizontal) {
				BiStep(BIFULLSTEP, CLOCKWISE);
				if (!uPosVertical) {
					UniStep(UNIFULLSTEP, CLOCKWISE);
				}
			}
		}
	}
	while (redButton) {}
}
void UniStep(char method, signed char direction)
{
/* Block Insertion */
// We want to insert a block of four bits into PORTD without changing the other bits. We use this logic:
// PORTD = ( ~(bitmask << offsetamount) & PORTD) |
// ( ((shifted insertblock) & bitmask)) << offsetamount)
// For unipolar: offsetamount = 0
// For bipolar: offsetamount = 4 and 6
//
// insertblock is the sequence, fed through PORTD like a piano roll
// for example UNIWAVEDRIVE would be 0B00010001 shifted by unipolarState
	unipolarState = (unipolarState + direction)%4;
	PORTD= (~(0B00001111)&PORTD) | (method>>((3-unipolarState))&0B00001111);
	Delay(_40ms);
}

void BiStep(char method, signed char direction)
{
// Same approach as above, though the patterns don't move synchronously across the pins
// Here pins 4 and 6 move through the same pattern fullstep 11001100 asynchronously with each other
	if (method == BIWAVEDRIVE) {
		bipolarState = (bipolarState + direction)%8;
		PORTD= (~(0B0000001<<4)&PORTD) | (( (method>>bipolarState) & 0B0000001)<<4);
		PORTD= (~(0B0000001<<6)&PORTD) | (( (method>>((bipolarState+5)%8)) & 0B0000001)<<6);
		PORTD= (~(0B0000001<<5)&PORTD) | (( (method>>(bipolarState)) & 0B0000001)<<5);
		PORTD= (~(0B0000001<<7)&PORTD) | (( (method>>(bipolarState+2)) & 0B0000001)<<7);
		Delay(_40ms);
	}
	else {
		bipolarState = (bipolarState + direction)%4;
		PORTD= (~(0B0000001<<4)&PORTD) | (( (method>>bipolarState) & 0B0000001)<<4);
		PORTD= (~(0B0000001<<6)&PORTD) | (( (method>>(bipolarState+3)) & 0B0000001)<<6);
		Delay(_40ms);
	}
}

void Delay (unsigned long timer)
{
	unsigned long i;
	for (i=0; i<timer; i++) {}
}
void SetLED()
{
	PORTB = state;
}
void ClearLED()
{
	PORTB = 0B00000000;
}
void Error()
{
	SetBit(state, 3);
	while (1) {
		SetLED();
		Delay(_1second);
		ClearLED();
		Delay(_1second);
	}
}
void Realign()
{
	SetLED();
}
