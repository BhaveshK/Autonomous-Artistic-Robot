/********************************************************************************
 Platform: SPARK V

 In this experiment for the simplicity PD4 and PD5 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:   L-1---->PB0;		L-2---->PB1;
   					   R-1---->PB2;		R-2---->PB3;
   					   PD4 (OC1B) ----> Logic 1; 	PD5 (OC1A) ----> Logic 1; 

 Serial Communication: PORTD 0 --> RXD UART receive for RS232 serial communication
					   PORTD 1 --> TXD UART transmit for RS232 serial communication

 
 Make sure that J5 us set towards the front side of the robot 
 i.e. USB to Serial converter is connected to the serial port of the microcontroller.

 Use baud rate as 115200bps.
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 
 *********************************************************************************/

 /********************************************************************************

   Copyright (c) 2012, IIT Bombay                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>

#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC

#define sbit(reg,bit)	reg |= (1<<bit)			// Macro defined for Setting a bit of any register.
#define cbit(reg,bit)	reg &= ~(1<<bit)		// Macro defined for Clearing a bit of any register.

#define Servo2port PORTD // servo 2 control pin connection 
#define Servo2bit 7

#define S1 OCR2
#define S2 OCR0
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned char receive_data='0';   // used to save Receiving data
unsigned int recv_data=0;
unsigned char flag;
unsigned char digit;
unsigned char color;
void init_ports();
void lcd_reset();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_line1();
void lcd_line2();
void lcd_string(char*);

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//Function to Reset LCD
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 to initialise LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	
}

//Function to Initialize LCD
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28);			//LCD 4-bit mode and 2 lines.
	lcd_wr_command(0x01);
	lcd_wr_command(0x06);
	lcd_wr_command(0x0E);
	lcd_wr_command(0x80);
		
}

	 
//Function to Write Command on LCD
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

//Function to Write Data on LCD
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}


//Function to bring cursor at home position
void lcd_home()
{
	lcd_wr_command(0x80);
}


//Function to Print String on LCD
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

//Position the LCD cursor at "row", "column".

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}
//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
 PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
 PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}

//Function to initialize ports
void port_init()
{
 lcd_port_config();
 motion_pin_config();          //robot motion pins config
 left_encoder_pin_config();    //left encoder pin config
 right_encoder_pin_config();   //right encoder pin config	
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
 GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
 sei(); // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
 GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
 sei(); // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT0_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

ISR(TIMER0_OVF_vect)
{
  // Timer 2 over flow 
  Servo2port |= (1 << Servo2bit); // set servo 2 control pin
  TCNT0 = 0x6F; // reload timer 0 initial value 
}

//-------------------------------------------------------------------------

ISR(TIMER0_COMP_vect)
{
  // Timer 0 compare match interrupt 
  Servo2port &= ~(1 << Servo2bit); //clear servo 2 control pin 
}


//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}

void servo_init(void) 
{ 
    
	DDRD|=(1<<PD7); //input defination 
	TCCR0|=(1<<CS02)|(1<<CS00);    // Timer 0 prescaler 1024 , normal mode 
	TCNT0=0x6F;                               // Timer 0 initial vlaue 
 	TIMSK|=(1<<OCIE0)|(1<<TOIE0); // timer 0 compare match and overflow interrupt enable  
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void back(void)        //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0x0A);
}

void stop (void)            //hard stop
{
  motion_set(0x00);
}
void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}
void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0x02);
}

void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 12.85; // division by resolution to get shaft count 
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0; 
 while(1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) & (ShaftCountLeft >= ReqdShaftCountInt))
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 //left(); //Turn left
 //angle_rotate(Degrees);
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 //right(); //Turn right
 //angle_rotate(Degrees);
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void init_devices (void)
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 uart0_init(); 
 servo_init();
 lcd_set_4bit();
 lcd_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();   // Enables the global interrupt 
}
void uart0_clr(void)
{
 UCSRB = 0x18; 
}
void uart0_rein(void)
{
 UCSRB = 0x98; 
}

SIGNAL(SIG_UART_RECV) 
{
    receive_data = UDR;			
    UDR = '.';          
}


//Main Function
int main()
{ 
   init_devices();
   while(1)
   {
     if(receive_data=='<')
     {
       lcd_cursor(1,1);
       lcd_string("Drawing Completed");
       lcd_cursor(2,1);
       lcd_string("NEX ROBOTICS IND");
     }
     if(receive_data=='[')
	 {
	    S2=0x78;
		_delay_ms(1000);
	 }

	 if(receive_data==']')
	 {
	    S2=0x7F;
		_delay_ms(1000);
	 }

     if(receive_data == 'r'||receive_data == 'g'||receive_data == 'b')      
     {
        //uart0_clr();
        color=receive_data;
		receive_data='0';
        //continue;
     }  	 
     if(receive_data == 'w'||receive_data == 'a'||receive_data == 'd'||receive_data == 's')      
     {
        //uart0_clr();
        flag=receive_data;
		receive_data='0';
        //continue;
     }
	 if(receive_data == 't'||receive_data == 'h')
	 {
	     digit=receive_data;
         receive_data='0';
	 }	 
	 if(color=='b'&&flag=='w'&&digit=='h')
	 {
	     if(receive_data=='1')
		 {
		    forward_mm(100);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    forward_mm(200);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    //S2=0x78; // 90 degree
            //_delay_ms(1000);
		    forward_mm(300);
			//_delay_ms(1000);
			//S2=0x7F; // 180 degree
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='4')
		 {
		    forward_mm(400);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='5')
		 {
		    forward_mm(500);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='6')
		 {
		    forward_mm(600);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='7')
		 {
		    forward_mm(700);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='8')
		 {
		    forward_mm(800);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='9')
		 {
		    forward_mm(900);
			receive_data='0';
			//UDR='x';
		 }

	 }
	 if(color=='b'&&flag=='w'&&digit=='t')
	 {
	     if(receive_data=='1')
		 {
		    forward_mm(11);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    forward_mm(20);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    forward_mm(30);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='4')
		 {
		    forward_mm(40);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='5')
		 {
		    forward_mm(50);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='6')
		 {
		    forward_mm(60);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='7')
		 {
		    forward_mm(70);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='8')
		 {
		    forward_mm(80);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='9')
		 {
		    forward_mm(90);
			receive_data='0';
			//UDR='x';
		 }

	 }
     if(color=='b'&&flag=='s'&&digit=='h')
	 {
	     if(receive_data=='1')
		 {
		    back_mm(170);
			receive_data='0';
			//UDR='x';
		 }
     }
     if(color=='b'&&flag=='a'&&digit=='t')
	 {
	     if(receive_data=='1')
		 {
		    left_degrees(10);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    left_degrees(20);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    left_degrees(30);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='4')
		 {
		    left_degrees(40);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='5')
		 {
		    left_degrees(50);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='6')
		 {
		    left_degrees(60);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='7')
		 {
		    left_degrees(70);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='8')
		 {
		    left_degrees(80);
			receive_data='0';
			//UDR='x';
		 }
         if(receive_data=='9')
		 {
		    left_degrees(90);
			receive_data='0';
			//UDR='x';
		 }
	 }
	 if(color=='b'&&flag=='a'&&digit=='h')
	 {
	     if(receive_data=='1')
		 {
		    left_degrees(100);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    left_degrees(200);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    left_degrees(300);
			receive_data='0';
			//UDR='x';
		 }
	 }
	 if(color=='b'&&flag=='d'&&digit=='t')
	 {
	     if(receive_data=='1')
		 {
		    right_degrees(10);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    right_degrees(20);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    right_degrees(30);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='4')
		 {
		    right_degrees(40);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='5')
		 {
		    right_degrees(50);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='6')
		 {
		    right_degrees(60);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='7')
		 {
		    right_degrees(70);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='8')
		 {
		    right_degrees(80);
			receive_data='0';
			//UDR='x';
		 }
         if(receive_data=='9')
		 {
		    right_degrees(90);
			receive_data='0';
			//UDR='x';
		 }
	 }
	 if(color=='b'&&flag=='d'&&digit=='h')
	 {
	     if(receive_data=='1')
		 {
		    right_degrees(100);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='2')
		 {
		    right_degrees(200);
			receive_data='0';
			//UDR='x';
		 }
		 if(receive_data=='3')
		 {
		    right_degrees(300);
			receive_data='0';
			//UDR='x';
		 }
	 }    
   }      
}
     


