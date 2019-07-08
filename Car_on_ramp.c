//Assignment: Lab 5
//Names: Alvin Yu, Maria Cardenas, Max Griffon
//Section: 2
//Side: B

/*
This code is for a car oriented facing up a simple ramp.
The code will begin by reversing the car until the measured roll of the body is near-zero.
The car will then drive forward, accelerating based on the deviation of the body from the x-y plane, or the flatness of the ground.
The car will turn to face down the ramp, and will continue to drive until an instance of sufficient flatness is detected.

Please lay the car on a flat surface during the initial stages of the code.
This is to ensure that the calibrations for tilt will be correct to the area.
*/

#include <c8051_SDCC.h>
#include <stdlib.h> 
#include <stdio.h>
#include <i2c.h>

//************************
//Initialization Functions 
void Port_Init(void); 
void PCA_Init(void); 
void ADC_Init(void); 
void PCA_ISR(void) __interrupt 9; 
void XBR0_Init(void);
void SMB0_Init(void);
void Accel_Init_C(void);

//Setting Values
void Set_Pitch_Gain(void);
void Set_Servo_PW(void); 

//Drive Motor
void Forward(void);
void Reverse(void);

//Read
unsigned char Read_AD_Input(unsigned char n);
void Read_Accel(void);

//Global Variables 
unsigned int Servo_PW;										//Servo Pulsewidth
unsigned int PW_D = 2765;

signed int avg_Roll = 0;
signed int avg_Pitch = 0;
signed int Roll = 0;
signed int Pitch = 0;
signed int Pitch_Gain = 0;	// y-axis drive feedback gain
signed int Roll_Gain = 0;	// x-axis drive feedback gain
signed int Steer_Gain = 0;	// steering feedback gain

signed int Roll_calib = 0;	// offsets after calibrating accelerometer
signed int Pitch_calib = 0;

signed int Roll_sum=0;
signed int Pitch_sum=0;

unsigned int i;
signed int j=0;
unsigned char input;
unsigned char print_flag;
int stage_flag = 0;
unsigned char new_accel = 0;	// flag for count of accel timing

__sbit __at 0xB7 SS;			//SS associated with P3.7
__sbit __at 0xA1 BILED0;		//Green BILED associated with P2.1
__sbit __at 0xA3 BILED1;		//Red BILED associated with P2.3

//Accelerometer Variables
unsigned char a_count;			// overflow count for Accelerometer 
unsigned char print_count;
unsigned int counts;
unsigned char addr_A = 0x3A;	// adddress of accelerometer
unsigned char Data[4];

//Main Function
void main(void) 
{ 
	Sys_Init(); // initialize board 
	putchar('  '); 
	Port_Init(); 
	PCA_Init(); 
	XBR0_Init();
	SMB0_Init();
	ADC_Init();
	Accel_Init_C();

	a_count = 0;
	counts = 0;
	print_count = 0;
	BILED0 = 1;
	BILED1 = 1;

	//Drive Motor Initialization
	PCA0CPL2 = 0xFFFF - 2765;
	PCA0CPH2 = (0xFFFF - 2765)>> 8;
	//Steering Servo Initialization
	PCA0CPL0 = 0xFFFF - 2765;
	PCA0CPH0 = (0xFFFF - 2765)>> 8;
	
	printf("\n\r Calibrating Flat State");
	for(j=0;j<64;j++)
	{
		while(!new_accel);
	
		Read_Accel();
		Roll_sum+=Roll;
		Pitch_sum+=Pitch;
		new_accel = 0;
		a_count=0;
	}		
	Roll_calib= Roll_sum/64;
	Pitch_calib= Pitch_sum/64;
	printf("\n\r Flatestate Calibration Complete");

	//Drive Motor Stablestate Initialization
	while(counts < 50);
	printf("\n\r Calibrating Steady State.");
	counts = 0;
	while(counts < 50);
	printf("\r Calibrating Steady State..");
	counts = 0;
	while(counts < 50);
	printf("\r Calibrating Steady State...");
	counts = 0;
	while(counts < 50);
	printf("\n\r Steady State Reached \n\r");
	counts = 0; 
	
	//LCD screen initialization
	while (counts < 50);
	counts = 0;
	lcd_clear();

	//Set gain values
	
	//Front-Back Gain Using Potentiometer
	Set_Pitch_Gain();
	printf("\n\r Pitch Gain at %d out of 50  ", Pitch_Gain);
	printf("\n\r Please Enter Gains on Keypad");

	//Left-Right Gain Using Keypad
	lcd_clear();
	lcd_print("Enter Roll Gain:\n");
	Roll_Gain = kpd_input(1);
	
	//Steering Gain Using Keypad
	lcd_clear();
	lcd_print("Enter Steering Gain:");	
	Steer_Gain = kpd_input(1);

	while (counts < 150);
	counts = 0;
	while (1) 
	{
		if(SS)
		{
			lcd_clear();
			printf("Paused, Car Neutral  \r");
			PCA0CP2 = 0xFFFF - 2765;
			lcd_clear();
			lcd_print("\n\r Recalibrating Drive Gain");
			while(SS)
			{
				Set_Pitch_Gain();
				printf("\r Pitch Gain at %d out of 50  ", Pitch_Gain);
			}
			lcd_clear();
		}
		else
		{
			if (stage_flag == 0)
			{
				printf("\n\r Please enter whether to turn (l)eft or (r)ight while reversing");
				input = getchar();
				if (input == 'l' || input == 'L')
				{
					Servo_PW = 2505;
					PCA0CPL0 = 0xFFFF - Servo_PW;
					PCA0CPH0 = (0xFFFF - Servo_PW) >> 8;	
				}
				if (input == 'r' || input == 'R')
				{
					Servo_PW = 3265;
					PCA0CPL0 = 0xFFFF - Servo_PW;
					PCA0CPH0 = (0xFFFF - Servo_PW) >> 8;
				}
				
				printf("\n\r Data will be recorded and printed as follows.");
				printf("\n\r Pitch, Roll, PW_D, Servo_PW, Pitch_Gain, Roll_Gain, Steer_Gain");
				while (stage_flag == 0)
				{
					int old_pitch = Pitch;
					Read_Accel();
					Reverse();
					if (print_flag)
					{
						printf("\n\r %d,	%d,	%d,	%d,	%d,	%d,	%d", Pitch, Roll, PW_D, Servo_PW, Pitch_Gain, Roll_Gain, Steer_Gain);
						print_flag = 0;
					}
					BILED0 = 1;
					BILED1 = 0;
					if(SS)
					{
						lcd_clear();
						printf("Paused, Car Neutral  \r");
						PCA0CP2 = 0xFFFF - 2765;
						lcd_clear();
						lcd_print("\n\r Recalibrating Drive Gain");
						while(SS)
						{
							Set_Pitch_Gain();
							printf("\r Pitch Gain at %d out of 50  ", Pitch_Gain);
						}
						lcd_clear();
					}
					if(abs(Pitch) <= 5 && abs(old_pitch) <= 5)
					{
						stage_flag=1;
					}
				}

			}
			while (stage_flag == 1) 
			{
				if (print_flag)
				{
					printf("\n\r %d,	%d,	%d,	%d,	%d,	%d,	%d", Pitch, Roll, PW_D, Servo_PW, Pitch_Gain, Roll_Gain, Steer_Gain);
					print_flag = 0;
				}
				if (new_accel)			// enough overflows to read accelerometer
				{
					BILED0 = 0;
					BILED1 = 1;
					
					Read_Accel();		// read accelerometer
					Set_Servo_PW();		// set the servo PWM
					Forward();			// set drive PWM
					
					new_accel = 0;		// reset new_accel flag
					a_count = 0;
				}
				if(SS)
				{
					lcd_clear();
					printf("Paused, Car Neutral  \r");
					PCA0CP2 = 0xFFFF - 2765;
					lcd_clear();
					lcd_print("\n\r Recalibrating Drive Gain");
					while(SS)
					{
						Set_Pitch_Gain();
						printf("\r Pitch Gain at %d out of 50  ", Pitch_Gain);
					}
					lcd_clear();
				}
				if (abs(Roll) <= 10 && abs(Pitch) <= 50)
				{
					PCA0CP0 = 0xFFFF - 2765;
					PCA0CP2 = 0xFFFF - 2765;
					stage_flag = 2;
				}
			}
			if (stage_flag == 2)
			{
				printf("\n\r End of Code, End of Data Collection");
				while (stage_flag == 2)
				{
					PCA0CP0 = 0xFFFF - 2765;
					PCA0CP2 = 0xFFFF - 2765;
				}
			}
		}
	}
}

//*******************
void Read_Accel(void)
{
	avg_Roll = 0;	// clear the averages
	avg_Pitch = 0;
	for (i = 0; i < 4; i++)	// for 4 iterations
	{
		do
		{
			i2c_read_data(addr_A, 0x27, Data, 1);	// read 1 byte of data from status_reg_a
		}
		while ((Data[0] & 0x03) != 0x03);	// continue if 2 LS bits are high, otherwise reread status
		i2c_read_data(addr_A, 0x28|0x80, Data, 4);	// read 4 registers starting with 0x28
		avg_Roll += ((Data[1] << 8) >> 4);	// discard low byte
		avg_Pitch += ((Data[3] << 8) >> 4);
	}

	avg_Roll = avg_Roll >> 2;				// finish calculating averages
	avg_Pitch = avg_Pitch >> 2;
	Roll = avg_Roll - Roll_calib;			// set global variables, subtract offset determined
	Pitch = avg_Pitch - Pitch_calib;		// during calibration
}
//***********************
void Set_Servo_PW(void)
{
	Servo_PW = 2765 + (Steer_Gain * Roll);
	//Safety Limit Check and Rollover
	if(Servo_PW > 3365)
	{
		Servo_PW = 3365;
	}
	if(Servo_PW < 2405)
	{
		Servo_PW = 2405;
	}

	PCA0CPL0 = 0xFFFF - Servo_PW;
	PCA0CPH0 = (0xFFFF - Servo_PW) >> 8;
}

//***********************
void Set_Pitch_Gain(void)
{
	Pitch_Gain = Read_AD_Input(4) * 10 / 51;
}
//****************
void Forward(void)	//Neutral to full forwards
{
	PW_D = 2765 - (Pitch_Gain / 10 * Pitch);	// calculate drive motor PW
	PW_D += (Roll_Gain * abs(Roll));

	if (PW_D < 2765)
	{
		PW_D = 2765;
	}
	if (PW_D > 3500)
	{
		PW_D = 3500;
	}

	PCA0CP2 = 0xFFFF - PW_D;
}
//********************************
void Reverse(void) 	//Neutral to full backwards
{
	PW_D = 2300;
	PCA0CP2 = 0xFFFF - PW_D;
}

//****************************************************************
//Initialization Functions
//*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_
//********************************
//Port Ininitialization
void Port_Init()
{
	P0MDOUT &= ~0xC0; 		//Set inputs to open drain
	P0 |= 0xC0; 			//Set inputs to high impedance

	P1MDOUT |= 0x05;		//CEX0, CEX2 PP
	P1MDIN &= ~0x90;		//set P1.4, 1.7 for analog input
	P1 |= 0x20;

	P2MDOUT |= 0x0A;		//Set Output BILED0, BILED1 to PP
	
	P3MDOUT &= ~0x80;		//Set Input SS P3.7 to OD
	P3 |= 0x80;
}
//********************************
//CrossBar Initialization
void XBR0_Init()
{
	XBR0 = 0x27;
}
//********************************
//Programmable Counter Array Initialization
void PCA_Init(void)
{
	PCA0MD = 0x81;					//SYS/12, CF interrupts enabled
	PCA0CPM0 = PCA0CPM2 = 0xC2;		//16 bit, enable CCM0 and CCM2
	PCA0CN |= 0x40;					//Enable PCA counter
	EIE1 |= 0x08;					//PCA interupt
	EA = 1;							//Global variables
	IE |= 0x02;						//Includes Interrupt Initializations
}		
//********************************
//SMBus Initialization
void SMB0_Init(void)
{
	SMB0CR=0x93;		//set SCL to 100KHz (actual freq ~ 94,594Hz)
	ENSMB=1;			//bit 6 of SMB0CN, enable the SMBus
}
//********************************
//Analog / Digital Conversion Initialization
void ADC_Init(void)
{
	REF0CN = 0x03;		//Set Vref = to internal reference voltage (2.4v)
	ADC1CN = 0x80;		//Enables ADC converter
	ADC1CF |= 0x01;		//Gain = 1
}
//********************************
//Read Analog / Digital Conversion Pin
unsigned char Read_AD_Input (unsigned char n)
{
	AMX1SL = n; 							//Set P1.n as the analog input for ADC1
	ADC1CN &= ~0x20; 						//Clear the Conversion Completed flag
	ADC1CN |= 0x10;							//Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00); 		//Wait for conversion to complete
	return ADC1; 							//Return digital value in ADC1 register
}
//********************************
//Interrupt Service Routine for Programmable Counter Array Overflow Interrupt 
void PCA_ISR(void) __interrupt 9 
{ 
	if (CF)
	{
		CF = 0; 				//Clear overflow indicator 
		a_count++;				//Heading Counter
		counts++;				//Timer0 Proxy
		print_count++;			//Print Counter
		if(a_count>=2)
		{
			new_accel = 1;
			a_count = 0;
		}
		if(print_count>=5)
 		{
			print_count = 0;
			print_flag = 1; 
		} 
		PCA0 = 28666; //PCA_start
	} 
	PCA0CN &= 0xC0; 
}