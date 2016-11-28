// (c) 2008-2014, Jesus Calvino-Fraga
// Modified by  D && A
// ~C51~ 

#include <stdio.h>
#include <stdlib.h>
#include <c8051f38x.h>

#define MHZ 1000000L
#define SYSCLK (48*MHZ)
#define BAUDRATE 115200L

#define frequency 4000	//freq = 1/(250x10^-6)

// Some ANSI escape sequences
#define CLEAR_SCREEN "\x1b[2J"


/*----------------------------------------------------------------------------*/
/*                         H bridge for motors                                */
/*----------------------------------------------------------------------------*/
#define OUT_Left_0  P2_5
#define OUT_Left_1  P2_4
#define OUT_Right_0 P2_3
#define OUT_Right_1 P2_2
/*----------------------------------------------------------------------------*/
/*                             Buzzer & LED                                   */
/*----------------------------------------------------------------------------*/
#define Buzzer  		P1_1
#define LED_front 		P1_2
#define LED_back		P1_3			
#define stage_indicator P0_6
/*----------------------------------------------------------------------------*/
/*                       voltages using in tracking                           */
/*----------------------------------------------------------------------------*/
#define Left_v_large_gain P2_7
#define Right_v_large_gain P2_1
#define Left_v_small_gain P2_6
#define Right_v_small_gain P2_0
/*----------------------------------------------------------------------------*/
/*                             auto detect                                    */
/*----------------------------------------------------------------------------*/
#define back_danger  P1_5	//the other terminal is connecting to logic low
#define front_hit_wall P1_7	//the other terminal is connecting to logic low
#define dark		P0_7 //if dark=1,turn on bright_LED
#define bright_LED  P1_0
/*----------------------------------------------------------------------------*/
/*                     digital signals from the inductors                     */
/*----------------------------------------------------------------------------*/
#define command_L_inductor P1_4

/*----------------------------------------------------------------------------*/
/*                                commands                                    */
/*----------------------------------------------------------------------------*/

#define track_mode    0b_11011100//220  -50...
#define buzzer        0b_10101010//170
#define move_forward  0b_01111000//120
#define move_backward 0b_01000110//70
#define rotate_180    0b_00010100//20


#define parallel_park 0b_11100010//226   -30...
#define front_led     0b_11000100//196
#define back_led      0b_10100110//166
#define control_mode  0b_10001000//130
#define repeat		  0b_01100100//100


#define brake_command  0b_11111111//255
#define rotate_CW	   0b_10001001//137
#define rotate_CCW     0b_11101100//236

#define command_NULL   0b_00000000
/*----------------------------------------------------------------------------*/
/*                             For ADC                                        */
/*----------------------------------------------------------------------------*/
#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  2
/*----------------------------------------------------------------------------*/
/*              Predefined distance in tracking mode                          */
/*----------------------------------------------------------------------------*/ 
#define desired_volt_0  2.0 //15 is the closest
#define tolerance_0     0.2

#define desired_volt_1  0.5//30
#define tolerance_1     0.2

#define desired_volt_2  2.2
#define tolerance_2     0.2

#define desired_volt_3  1.3
#define tolerance_3     0.2

#define desired_volt_4  0.2
#define tolerance_4     0.05

/*----------------------------------------------------------------------------*/
/*                             LED_stage                                      */
/*----------------------------------------------------------------------------*/
volatile int LED_f_stage=0, LED_b_stage=0;
/*----------------------------------------------------------------------------*/
/*                             Peak V                                         */
/*----------------------------------------------------------------------------*/
volatile float v_left=0,v_right=0,v;
/*----------------------------------------------------------------------------*/
/*                             Motor Control                                  */
/*----------------------------------------------------------------------------*/
volatile unsigned char pwm_count=0;
volatile unsigned int left_forward=0,left_backward=0,right_forward=0,right_backward=0;
volatile unsigned int temp_left_forward=0,temp_left_backward=0,temp_right_forward=0,temp_right_backward=0;
/*----------------------------------------------------------------------------*/
/*                            Receiver                                        */
/*----------------------------------------------------------------------------*/
volatile int robot_control=0,positive=0;
volatile int track_distance_count=0;
volatile unsigned char command=command_NULL;
volatile float desired_distance,expected_tolerance;
/*----------------------------------------------------------------------------*/
/*                             Auto Detect                                    */
/*----------------------------------------------------------------------------*/
volatile int hit_wall_flag=0;
//zvolatile int count_dark=0;
/*----------------------------------------------------------------------------*/
/*                             Repeat                                         */
/*----------------------------------------------------------------------------*/
volatile unsigned char recorder[3];
volatile unsigned int recorder_counter=0;
/*----------------------------------------------------------------------------*/
/*                             Joystick                                       */
/*----------------------------------------------------------------------------*/
volatile unsigned int brake_counter=0,speed_counter=0; 
char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == (12*MHZ))
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == (24*MHZ))
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == (48*MHZ))
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12MHZ, 24MHZ, or 48MHZ
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency
	
    // Configure P2.7 P2.6 P2.1 and P2.0 as analog inputs
	P2MDIN = 0b_0011_1100; // P2.7 P2.6 P2.1 and P2.0
	P2SKIP |= 0b_1100_0011; // Skip Crossbar decoding for P2.7 P2.6 P2.1 and P2.0
	P2MDOUT|=0b_0000_0000;
	P1MDIN = 0b_1111_1111;
	P1MDOUT|= 0b_0000_1111;
	
	P0MDIN = 0b_0111_1111;
	P0SKIP |= 0b_1000_0000;
	P0MDOUT|=0b_0100_0000;
	

	// Init ADC multiplexer to read the voltage between P2.0 and ground.
	// These values will be changed whe	n measuring to get the voltages from
	// other pins.
	// IMPORTANT: check section 6.5 in datasheet.  The constants for
	// each pin are available in "c8051f38x.h" both for the 32 and 48
	// pin packages.
	AMX0P = LQFP32_MUX_P2_7; // Select positive input from P2.7
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output

	
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
	#if (SYSCLK/BAUDRATE/2L/256L < 1)
		TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
		CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
		CKCON |=  0x08;
	#elif (SYSCLK/BAUDRATE/2L/256L < 4)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
		CKCON |=  0x01;
	#elif (SYSCLK/BAUDRATE/2L/256L < 12)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
	#else
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
		CKCON |=  0x02;
	#endif

	TR0=0; // Stop timer 0
	TMOD|=0B_0000_0001; // Set timer 0 as 16-bit timer


	TL1 = TH1;     // Init timer 1
	TMOD &= 0x0f;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |= 0x20;                       
	TR1 = 1;       // Start timer1
	SCON = 0x52;      // Start timer1                     

	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*48))/(100L)); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2
	EA=1; // Enable interrupts
	return 0;
}

void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}


void Timer2_PWM (void) interrupt 5
{
	TF2H = 0; // Clear Timer2 interrupt flag
	if(back_danger==1 && command!=(unsigned int)brake_command)
	{
		left_backward=100;
		right_backward=100;
	}
    else if(front_hit_wall==0&&hit_wall_flag==0)
    {
    	hit_wall_flag=1;
    }
	
	pwm_count++;
	if(pwm_count>100) 
		pwm_count=0;
		
	OUT_Left_0=pwm_count>left_backward?0:1;
	OUT_Left_1=pwm_count>left_forward?0:1;
	OUT_Right_0=pwm_count>right_forward?0:1;
	OUT_Right_1=pwm_count>right_backward?0:1;
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

unsigned char rx_byte ()
{
	int i=0,j=0,v=0;
	unsigned char val = 0,stop_bits=0,start_bits=0;
	// Figure out when to sync the reading time w/ data receiving time
	for(i = 0; i<20; i++){
		if(command_L_inductor)
			return 0;
		waitms(1);
	}
	// Wait for a rising edge to SYNC
	while(!command_L_inductor);
	waitms(15);
	// SYNCED - Date Reading sequence
	
	for(i=0; i<3; i++)
	{
		//P1_0=1; //debugging pin
		for(j=0;j<5;j++)
		{
			if(command_L_inductor)
				v++;
			waitms(1);
		}
		//P1_0=0; //debugging pin
		start_bits|=(v >= 3)?(0b_10000000>>i):0x00;
	//	val|=(v >=3)?(0x01<<j):0x00;
		waitms(10);
		v=0;
	}
	for(i=0; i<8; i++)
	{
		//P1_0=1; //debugging pin
		for(j=0;j<5;j++)
		{
			if(command_L_inductor)
				v++;
			waitms(1);
		}
		//P1_0=0; //debugging pin
		val|=(v >= 3)?(0b_10000000>>i):0x00;
	//	val|=(v >=3)?(0x01<<j):0x00;
		waitms(10);

		v=0;
	}
	//Wait for stop bits
		for(i=0; i<3; i++)
	{
		//P1_0=1; //debugging pin
		for(j=0;j<5;j++)
		{
			if(command_L_inductor)
				v++;
			waitms(1);
		}
		//P1_0=0; //debugging pin
		stop_bits|=(v >= 3)?(0b_10000000>>i):0x00;
	//	val|=(v >=3)?(0x01<<j):0x00;
		waitms(10);

		v=0;
	}
	if(stop_bits==0b_01000000&&start_bits==0b_01000000)
		return val;
	else
		return 0;
}

void send_data()
{		
		printf( CLEAR_SCREEN );
	   	printf("%6.2fV\n",v_left );
	   	printf("%6.2fV\n",v_right );
}




void response()
{

	if(hit_wall_flag==1 && robot_control==1)
	{
		back_danger=0;
		left_forward=100;
		right_forward=100;
		left_backward=0;
		right_backward=0;
		Buzzer = 1;
		waitms (300);
		Buzzer = 0;
		waitms (300);
		Buzzer=1;

		left_forward=100;
		right_backward=100;
		left_backward=0;
		right_forward=0;
		waitms(300);
		Buzzer=0;
		waitms(300);
		Buzzer=1;
		waitms(300);
		Buzzer=0;
		waitms(100);
		
		left_forward=0;
		right_backward=0;
		if(positive==0)
			positive=1;
		else
			positive=0;
		back_danger=1;
		hit_wall_flag=0;
	}

	else if((unsigned int)command==move_forward && robot_control==1)//move forward ,further to the transmitter when positive=0
	{
		if(track_distance_count == 0)
		{
		desired_distance=desired_volt_1;
		expected_tolerance=tolerance_1;
		track_distance_count = 1;
		}
		else if (track_distance_count == 1)
		{
		desired_distance=desired_volt_2;
		expected_tolerance=tolerance_2;
		track_distance_count = 2;
		}
		else if (track_distance_count == 2)
		{
		desired_distance=desired_volt_3;
		expected_tolerance=tolerance_3;
		track_distance_count = 3;
		}
		else if (track_distance_count == 3)
		{
		desired_distance=desired_volt_4;
		expected_tolerance=tolerance_4;	
		track_distance_count = 4;
		}	   
		command=(unsigned char)command_NULL;

	}
	
	else if( (unsigned int)command==move_backward && robot_control==1)// move backward,close to the transmitter 
	{
		if(track_distance_count == 1)
		{
		desired_distance=desired_volt_0;
		expected_tolerance=tolerance_0;
		track_distance_count = 0;
		}
		else if (track_distance_count == 2)
		{
		desired_distance=desired_volt_1;
		expected_tolerance=tolerance_1;
		track_distance_count = 1;
		}
		else if (track_distance_count == 3)
		{
		desired_distance=desired_volt_2;
		expected_tolerance=tolerance_2;
		track_distance_count = 2;
		}
		else if (track_distance_count == 4)
		{
		desired_distance=desired_volt_3;
		expected_tolerance=tolerance_3;
		track_distance_count = 3;
		}	   
		command=(unsigned char)command_NULL;

	}
	else if((unsigned int)command==rotate_180 && robot_control==1)//rotating 180
	{
		left_forward=100;
		right_backward=100;
		left_backward=0;
		right_forward=0;
		
		waitms(1000);
		left_forward=0;
		right_backward=0;
		if(positive==0)
			positive=1;
		else
			positive=0;
		waitms(500);
		command=(unsigned char)command_NULL;
	}
	else if((unsigned int)command==buzzer)
	{	
		temp_left_forward   = left_forward;
		temp_right_forward  = right_forward;
		temp_left_backward  = left_backward;
		temp_right_backward = right_backward;
		Buzzer = 1;
		left_forward   = 0;
		right_forward  = 0;
		left_backward  = 0;
		right_backward = 0;
		waitms (1000);
		Buzzer = 0;
		left_forward   = temp_left_forward;
		right_forward  = temp_right_forward;
	    left_backward  = temp_left_backward;
		right_backward = temp_right_backward;
		command=(unsigned char)command_NULL;
	}
	else if((unsigned int)command==parallel_park && robot_control==1)
	{
		
		right_forward=0;
		right_backward=0;
		left_forward=0;
		
		left_backward=80;
		waitms(450);
		right_backward=80;
		waitms(950);
		left_backward=0;
		waitms(450);
		right_backward=0;
		command=1;
		while(command==1)
		{
			command=rx_byte ();
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
			if((unsigned int)command==parallel_park)
				command=(unsigned char)command_NULL;//clear the flag after done	
			else
				command=1;	
		}
	
	}
	else if((unsigned int)command==front_led)
	{
		left_forward   = 0;
		right_forward  = 0;
		left_backward  = 0;
		right_backward = 0;
		if(LED_f_stage == 0)
		{
		LED_front = 1;
		LED_f_stage = 1;}	
		else{
		LED_front = 0;
		LED_f_stage = 0;
		}
	
		command=(unsigned char)command_NULL;
	}
	else if((unsigned int)command==back_led)
	{
		left_forward   = 0;
		right_forward  = 0;
		left_backward  = 0;
		right_backward = 0;
		if(LED_b_stage == 0)
		{
		LED_back = 1;
		LED_b_stage = 1;
		}
		else{
		LED_back = 0;
		LED_b_stage = 0;
		}
		command=(unsigned char)command_NULL;
		}
	else if((unsigned int)command==control_mode && robot_control==2)
	{
		stage_indicator = 0;
		robot_control=1;
		left_forward   = 0;
		right_forward  = 0;
		left_backward  = 0;
		right_backward = 0;
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		waitms (400);
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		waitms (400);
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		command=(unsigned char)command_NULL;	
	}
	else if((unsigned int)command==control_mode && robot_control==1)
	{
		stage_indicator = 1;
		robot_control=2;
		left_forward   = 0;
		right_forward  = 0;
		left_backward  = 0;
		right_backward = 0;
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		waitms (400);
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		waitms (400);
		Buzzer = 1;
		waitms (400);
		Buzzer = 0;
		command=(unsigned char)command_NULL;	
	}
//Joy stick control mode, to be continued...
	else if((unsigned int)command==move_forward && robot_control==2)
	{
	// robot starts moving forward
	back_danger=0;
	left_forward=35;
	right_forward=35;
	while(1)
	{
		// robot is still moving forward but keeps checking if 
		// the transmitters have sent out the brake_command 
		if(speed_counter==20)//60*20ms
		{
			left_forward=50;
			right_forward=50;
		}
		else if(speed_counter==35)
		{
			left_forward=75;
			right_forward=75;
		}
		else if(speed_counter==50)
		{
			left_forward=90;
			right_forward=90;
		}
		
		ET2=0;
		command=rx_byte ();
		ET2=1;
		if((unsigned int)command==brake_command)
		{	
			// stops the robot once a brake_command is received
			command=(unsigned char)command_NULL;
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
			speed_counter=0;					
			break;	
		}
		waitms(80);
		speed_counter++;
	}
	back_danger=1;
}

	else if((unsigned int)command==move_backward && robot_control==2)
	{
	back_danger=0;
	// robot starts moving forward
	left_backward=35;
	right_backward=35;
	while(1)
	{
		// robot is still moving forward but keeps checking if 
		// the transmitters have sent out the brake_command 
		if(speed_counter==20)//80*20ms
		{
			left_backward=50;
			right_backward=50;
		}
		else if(speed_counter==35)
		{
			left_backward=75;
			right_backward=75;
		}
		else if(speed_counter==50)
		{
			left_backward=90;
			right_backward=90;
		}
		
		ET2=0;
		command=rx_byte ();
		ET2=1;
		if((unsigned int)command==brake_command)
		{	
			// stops the robot once a brake_command is received
			command=(unsigned char)command_NULL;
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
			speed_counter=0;					
			break;	
		}
		waitms(80);
		speed_counter++;
	}
	back_danger=1;
}

	else if((unsigned int)command==parallel_park && robot_control==2)//ccw
	{
		back_danger=0;
		left_forward=25;
		right_backward=25;
		waitms(1000);
		left_forward=0;
		right_backward=0;
		back_danger=1;
	}

	else if((unsigned int)command==rotate_180 && robot_control==2)//cw
	{
		back_danger=0;
		left_backward=25;
		right_forward=25;
		waitms(1000);
		left_backward=0;
		right_forward=0;
		back_danger=1;
	}
	
	else 
		command=(unsigned char)command_NULL;
}

void get_voltages()
{
	int i=0;
	unsigned char j;
	
	// Start the ADC in order to select the first channel.
	// Since we don't know how the input multiplexer was set up,
	// this initial conversion needs to be discarded.
	if(track_distance_count>=2)
		AMX0P=LQFP32_MUX_P2_7;//Left_v_large_gain
	else
		AMX0P=LQFP32_MUX_P2_6;
	AD0BUSY=1;
	while (AD0BUSY); // Wait for conversion to complete
	{
		for(j=0; j<NUM_INS; j++)
		{
			AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			// Select next channel while ADC0 is busy
			switch(j)
			{
				case 0:					
					if(track_distance_count>=2)
						AMX0P=LQFP32_MUX_P2_1;//Right_v_large_gain
					else
						AMX0P=LQFP32_MUX_P2_0;
				break;
				case 1:
					if(track_distance_count>=2)
						AMX0P=LQFP32_MUX_P2_7;//Left_v_large_gain
					else
						AMX0P=LQFP32_MUX_P2_6;
				break;
			}
			while (AD0BUSY); // Wait for conversion to complete
			v=((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			switch(j)
			{
				case 0:		   
		    		v_left=v;
				break;
				case 1:
			    	v_right=v;
				break;
			}
		} //end of one measurement
}

void adjust_robot(float temp_distance,float temp_tolerance)
{

	unsigned int adj_pwm=80;
	//when positive=0, backward is the direction towarding the motor side of the car
	if(positive==0)
	{
		if(v_left>temp_distance+temp_tolerance && v_right>temp_distance+temp_tolerance)// too close
		{
			if(v_left>(v_right+temp_tolerance*2))
			{
				left_backward=adj_pwm;
				left_forward   = 0;
				right_forward  = 0;
				right_backward = 0;
			}
			else if(v_right>(v_left+temp_tolerance*2))
			{
				right_backward=adj_pwm;
				left_forward   = 0;
				right_forward  = 0;
				left_backward  = 0;
			}
			else
			{
				left_forward=adj_pwm;
				right_forward=adj_pwm;
				left_backward  = 0;
				right_backward = 0;
			}
			
		}
		else if(v_left<temp_distance-temp_tolerance && v_right<temp_distance-temp_tolerance)
		{
			if(v_left<(v_right-temp_tolerance*2))
			{
				left_forward=adj_pwm;
				right_forward  = 0;
				left_backward  = 0;
				right_backward = 0;
			}
			else if(v_right<(v_left-temp_tolerance*2))
			{
				right_forward=adj_pwm;
				left_forward   = 0;
				left_backward  = 0;
				right_backward = 0;
			}
			else
			{
				left_backward=adj_pwm;
				right_backward=adj_pwm;
				left_forward   = 0;
				right_forward  = 0;
			}
		}
		else if(v_left>temp_distance+temp_tolerance)
		{
			if(v_right<=temp_distance+temp_tolerance &&v_right>=temp_distance-temp_tolerance)
			{
				left_backward=adj_pwm;
				left_forward   = 0;
				right_forward  = 0;
				right_backward = 0;
			}
			else
			{
				left_backward=adj_pwm;
				right_forward=adj_pwm;
				left_forward   = 0;
				right_backward = 0;
			}
		}
		else if(v_left<temp_distance-temp_tolerance)
		{
			if(v_right<=temp_distance+temp_tolerance &&v_right>=temp_distance-temp_tolerance)
			{
				left_forward=adj_pwm;				
				right_forward  = 0;
				left_backward  = 0;
				right_backward = 0;
			}
			else
			{
				left_forward=adj_pwm;
				right_backward=adj_pwm;				
				right_forward  = 0;
				left_backward  = 0;
			}
		}
		else if(v_right>temp_distance+temp_tolerance)
		{
			right_backward=adj_pwm;
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
		}
		else if(v_right<temp_distance-temp_tolerance)
		{
			right_forward=adj_pwm;
			left_forward   = 0;
			left_backward  = 0;
			right_backward = 0;
		}
		else
		{
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
		}
	}
	else if(positive==1)
	{
		if(v_left>temp_distance+temp_tolerance && v_right>temp_distance+temp_tolerance)// too close
		{
			if(v_left>(v_right+temp_tolerance*2))
			{
				left_forward=adj_pwm;
				left_backward   = 0;
				right_forward  = 0;
				right_backward = 0;
			}
			else if(v_right>(v_left+temp_tolerance*2))
			{
				right_forward=adj_pwm;
				left_forward   = 0;
				right_backward  = 0;
				left_backward  = 0;
			}
			else
			{
				left_backward=adj_pwm;
				right_backward=adj_pwm;		
				left_forward  = 0;
				right_forward = 0;
			}
		}
		else if(v_left<temp_distance-temp_tolerance && v_right<temp_distance-temp_tolerance)
		{
			if(v_left<(v_right-temp_tolerance*2))
			{
				left_backward=adj_pwm;
				right_forward  = 0;
				left_forward  = 0;
				right_backward = 0;
			}
			else if(v_right<(v_left-temp_tolerance*2))
			{
				right_backward=adj_pwm;
				left_forward   = 0;
				left_backward  = 0;
				right_forward = 0;
			}
			else
			{
				left_forward=adj_pwm;
				right_forward=adj_pwm;
				left_backward   = 0;
				right_backward  = 0;
			}
		}
		else if(v_left>temp_distance+temp_tolerance)
		{
			if(v_right<=temp_distance+temp_tolerance &&v_right>=temp_distance-temp_tolerance)
			{
				left_forward=adj_pwm;
				left_backward   = 0;
				right_forward  = 0;
				right_backward = 0;
			}
			else
			{
				left_forward=adj_pwm;
				right_backward=adj_pwm;
				left_backward   = 0;
				right_forward = 0;
			}
		}
		else if(v_left<temp_distance-temp_tolerance)
		{
			if(v_right<=temp_distance+temp_tolerance &&v_right>=temp_distance-temp_tolerance)
			{
				left_backward=adj_pwm;				
				right_forward  = 0;
				left_forward  = 0;
				right_backward = 0;
			}
			else
			{
				left_backward=adj_pwm;
				right_forward=adj_pwm;				
				right_backward  = 0;
				left_forward  = 0;
			}
		}
		else if(v_right>temp_distance+temp_tolerance)
		{
			right_forward=adj_pwm;
			left_forward   = 0;
			right_backward  = 0;
			left_backward  = 0;
		}
		else if(v_right<temp_distance-temp_tolerance)
		{
			right_backward=adj_pwm;
			left_forward   = 0;
			left_backward  = 0;
			right_forward = 0;

		}
		else
		{
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
		}
	}
}	

void initial_vars()
{
	back_danger=1;
	front_hit_wall=1;
	Buzzer = 0;
	LED_front = 0;
	LED_back = 0;
	bright_LED=0;
	stage_indicator=0;
	P2_7=0;
	P2_6=0;
	P2_0=0;
	P1_4=0;
	desired_distance=desired_volt_0;
	expected_tolerance=tolerance_0;
}

void main (void)
{
	//initialization
	int i=0;
	initial_vars();
	robot_control=1;



	while(1)
	{
		
		ET2 = 0;
		command=rx_byte();
		ET2 = 1;
								
		if(robot_control==1)//tracking
		{

			response();//rotation,parallel park,turn on/off leds or buzzer
			get_voltages();
//			send_data();
			adjust_robot(desired_distance,expected_tolerance);
			v_left=0;//clear v_left,v_right for next round of measurement
			v_right=0;

		}
		else if(robot_control==2)
		{
			left_forward   = 0;
			right_forward  = 0;
			left_backward  = 0;
			right_backward = 0;
			response();
		}
		


	}	//end of while(1)
}
