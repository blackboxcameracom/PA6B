/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : MMA8652FC/MMA8652FC_funcs.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface Functions source code file for the MMA8652FC accelerometer
*
****************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include<wiringPi.h>
#include<wiringPiI2C.h>
#include<stdio.h>
#include<stdint.h>

#include"MMA8652FC_funcs.h"

/* Connection to the MMA8652 interrupt pins, buttons and the GPS 1PPS requires jumpers 
 * or wire links to be installed on the GPS-PIE board */
#define GPIO_INT1	3	// Wiring pin 3, physical pin 15, connection for MMA8652 int pin 1
#define GPIO_INT2	2	// Wiring pin 2, physical pin 13, connection for MMA8652 int pin 2

#define BUTTON1		21	// Wiring pin 21, physical pin 29, connection for button 1
#define BUTTON2		22	// Wiring pin 22, physical pin 31, connection for button 2

#define PPS			23	// Wiring pin 23, physical pin 33, connection for GPS receiver

#define STATUS_LED	25	// Wiring pin 25, physical pin 37, connected to status LED


/* This union consists of a char and bit field variable sharing the same memory space.
 * It us used to read a register byte from the MMA8652 as a character, then change 
 * individual bits in the register. */
union bytefield{
	unsigned char ctrlByte;
	
	
	struct {
		unsigned char ctrlBit0 : 1; 
		unsigned char ctrlBit1 : 1; 
		unsigned char ctrlBit2 : 1;
		unsigned char ctrlBit3 : 1;
		unsigned char ctrlBit4 : 1; 
		unsigned char ctrlBit5 : 1; 
		unsigned char ctrlBit6 : 1; 
		unsigned char ctrlBit7 : 1; 
	}controlBit;
		
			
}controlReg;


void int1_ISR( void );		// Interrupt service routine for MMA8652 pin 1
void int2_ISR( void );		// Interrupt service routine for MMA8652 pin 2
void setMMA8652StandByMode( void );	// Set the MMA8652 into standby mode
void setMMA8652ActiveMode( void );	// Set the MMA8652 into active mode

volatile char dataWaitingInt1;	// Flag set in the interrupt service rountines
volatile char dataWaitingInt2;	

int fd;						// i2c file descriptor

mma8652Data_t accel_data;

/* This routine opens the i2c connection to the MMA8562 and sets up the wiringPi
 * interface */
int mma8652SetUp( void )
{
	int id_reg;
	const char *i2cDevice;

//	i2cDevice = "/dev/i2c-3";	// Set the i2c device. 
//	fd = wiringPiI2CSetupInterface(i2cDevice,0x1D);	// Initialise the i2c interface.  Non-standard for RPi operating system i2c	

	fd = wiringPiI2CSetup(0x1D);					// Initialise the i2c interface.  Standard RPi hardware i2c, not used

	/* At the start of the program verify the MMA8562 is connected by reading the ID register */
	id_reg  = wiringPiI2CReadReg8(fd, 0x0D);			// Read the id register from address 0x0D
	if( id_reg == 0x4A )
	{
		printf("MMA8652FC detected. The ID register value is %2X\n", id_reg); 	// This should return 0x4A 'J'
	}
	else
	{
		printf("MMA8652FC not detected\n");
		
		return 0;
	}
		
	
	//--- GPIO pin set up with WiringPi library functions
	wiringPiSetup();

	// Interrupt pins setup in the individual interrupt setup functions

	pinMode( STATUS_LED, OUTPUT );		// Set up the status LED pin
	digitalWrite( STATUS_LED, LOW );	// Set initial state to low, LED off

 
	//=== MMA8652FC accelerometer set up. All page numbers refer to the MMA8652 datasheet ======================  

	setMMA8652StandByMode();	// All changes to the set up of the MMA8652 must be made in standby mode


	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x0E); // Read the XYZ_DATA_CFG register
 
	controlReg.controlBit.ctrlBit0 = 0;	// Set 2g range see pg 31
	controlReg.controlBit.ctrlBit1 = 0;	

	wiringPiI2CWriteReg8(fd, 0x0E, controlReg.ctrlByte );  // Write XYZ_DATA_CFG to the device with new settings

	//--- Unset all interrupt enable bits	
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2D); 	// Read CTRL_REG4 register
 
	controlReg.ctrlByte = 0;	// Clear all interrupt enable bit.  Return to known state at start up

	wiringPiI2CWriteReg8(fd, 0x2D, controlReg.ctrlByte );  	// Write CTRL_REG4 to the device with new settings
	
	//--- Interrupt pin and source configuration done in separate routines below


	//--- Set the control register
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2A); // Read the CTRL_REG1 register
 
	// Control register settings.  See pg 52 - 53.
	controlReg.controlBit.ctrlBit0 = 1;	// Set active mode
	controlReg.controlBit.ctrlBit1 = 0;	// Fast read mode off, data in 2 bytes
	controlReg.controlBit.ctrlBit3 = 1;	// Select output data rate of 12.5Hz
	controlReg.controlBit.ctrlBit4 = 0;
	controlReg.controlBit.ctrlBit5 = 1;
	controlReg.controlBit.ctrlBit6 = 1;	// ALSP rate to 12.5Hz.
	controlReg.controlBit.ctrlBit7 = 0;		

	wiringPiI2CWriteReg8(fd, 0x2A, controlReg.ctrlByte );  // Write CTRL_REG1 to the device with new settings
//===========================================================================

	return 1;
}
	


/* This routine is called to poll the MMA8652 to see if new data is waiting to be read.  
 * It returns true when data can be read */
int pollMMA8652( void )
{
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x00); // Read the MMA8652 STATUS register
   
	if( controlReg.controlBit.ctrlBit3 )	// Check the bit indicating data waiting to be read
	{
		return 1;
	}
	else
	{
		return 0;
	}		
	
}


/* This routine sets up the data ready interrupt on pin 1 of the MMA8652 and the corresponding 
 * interrupt service routine on the GPIO_INT1. To change the interrupt type see the documentation. */
void setUpMMA8652Int1( void )
{
	pinMode( GPIO_INT1, INPUT );			// Interrupt pins are set as inputs
	pullUpDnControl( GPIO_INT1, PUD_DOWN );	// with pull downs
	
	setMMA8652StandByMode();	// Set the device into standby mode so settings can be changed
		
	//--- Set up interrupt pin configuration refer to ACCL_AN4083 section 10.2 pg 29 -30
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2C);	// Read CTRL_REG3 register
 
	controlReg.controlBit.ctrlBit0 = 0;	// Select push/pull mode for interrupt pins
	controlReg.controlBit.ctrlBit1 = 1;	// Interrupt polarity active high	

	wiringPiI2CWriteReg8(fd, 0x2C, controlReg.ctrlByte );	// Write CTRL_REG3 to the device with new settings
	
	//---	
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2D); 	// Read CTRL_REG4 register
 
	controlReg.controlBit.ctrlBit0 = 1;	// Enable the data ready interrupt

	wiringPiI2CWriteReg8(fd, 0x2D, controlReg.ctrlByte );  	// Write CTRL_REG4 to the device with new settings
	
	//---	
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2E); 	// Read CTRL_REG5 register
 
	controlReg.controlBit.ctrlBit0 = 1;	// The data ready interrupt is routed to interrupt pin 1 = 1 or pin 2 = 0
	
	wiringPiI2CWriteReg8(fd, 0x2E, controlReg.ctrlByte );  	// Write CTRL_REG5 to the device with new settings

	//---
	setMMA8652ActiveMode();		// Set the device back toactive mode

	
	//--- Set up ISR for int 1
	wiringPiISR(GPIO_INT1, INT_EDGE_RISING, &int1_ISR);
	dataWaitingInt1 = 0;	// Clear the Int flag 

	read_MMA8652( &accel_data );	// Read to reset the MMA8652 interrupt pin

	return;
}


/* This routine sets up the data ready interrupt on pin 2 of the MMA8652 and the corresponding 
 * interrupt service routine on the GPIO_INT2. To change the interrupt type see the documentation. */
void setUpMMA8652Int2( void )
{
	pinMode( GPIO_INT2, INPUT );
	pullUpDnControl( GPIO_INT2, PUD_DOWN );

	setMMA8652StandByMode();

	//--- Set up interrupt pin configuration refer to ACCL_AN4083 section 10.2 pg 29 -30
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2C);	// Read CTRL_REG3 register
 
	controlReg.controlBit.ctrlBit0 = 0;	// Select push/pull mode for interrupt pins
	controlReg.controlBit.ctrlBit1 = 1;	// Interrupt polarity active high	

	wiringPiI2CWriteReg8(fd, 0x2C, controlReg.ctrlByte );	// Write CTRL_REG3 to the device with new settings
	
	//---	
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2D); 	// Read CTRL_REG4 register
 
	controlReg.controlBit.ctrlBit0 = 1;	// Enable the data ready interrupt

	wiringPiI2CWriteReg8(fd, 0x2D, controlReg.ctrlByte );  	// Write CTRL_REG4 to the device with new settings

	//---	
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2E);	// Read CTRL_REG5 register
 
	controlReg.controlBit.ctrlBit0 = 0;	// The data ready interrupt is routed to interrupt pin 1 = 1 or pin 2 = 0
	
	wiringPiI2CWriteReg8(fd, 0x2E, controlReg.ctrlByte );	// Write CTRL_REG5 to the device with new settings

	//---
	setMMA8652ActiveMode();
	
	//--- Set up isr for int 2
	wiringPiISR(GPIO_INT2, INT_EDGE_RISING, &int2_ISR);
	dataWaitingInt2 = 0; 
	
	read_MMA8652( &accel_data );	// Read to reset the MMA8652 interrupt pin
	
	return;
}	




/* Set the MMA8652 into standby mode.  All changes to the MMA8652 settings must be made when the 
 * device is in standby mode. */
void setMMA8652StandByMode( void )
{
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2A); // Read the CTRL_REG1 register
   
	controlReg.controlBit.ctrlBit0 = 0;	// Set standby mode.  All device setting should be made in standby mode
   
	wiringPiI2CWriteReg8(fd, 0x2A, controlReg.ctrlByte );
	
	return;
}	


/* Set the MMA8652 into active mode.  All changes to the MMA8652 settings must be made when the 
 * device is in standby mode. */
void setMMA8652ActiveMode( void )
{  
	controlReg.ctrlByte = wiringPiI2CReadReg8(fd, 0x2A); // Read the CTRL_REG1 register
   
	controlReg.controlBit.ctrlBit0 = 1;	// Set active mode.  All device setting should be made in standby mode
   
	wiringPiI2CWriteReg8(fd, 0x2A, controlReg.ctrlByte );
	
	return;
}	



/* This routine reads the accelerometer data from the MMA8652 as byte values and converts
 * the data to the floating point represention of the data. */
void read_MMA8652( mma8652Data_t *accel_data )
{
	unsigned char x_reg1, x_reg2;
	unsigned char y_reg1, y_reg2;
	unsigned char z_reg1, z_reg2;

	x_reg1 = wiringPiI2CReadReg8(fd, 0x01);	// MSB
	x_reg2 = wiringPiI2CReadReg8(fd, 0x02);	// LSB
			
	y_reg1 = wiringPiI2CReadReg8(fd, 0x03);
	y_reg2 = wiringPiI2CReadReg8(fd, 0x04);
			
	z_reg1 = wiringPiI2CReadReg8(fd, 0x05);
	z_reg2 = wiringPiI2CReadReg8(fd, 0x06);
		
	accel_data->x_axis =  ((float)((int16_t)(((x_reg1*256) + (x_reg2)))>> 4) * 0.0009765);
	accel_data->y_axis =  ((float)((int16_t)(((y_reg1*256) + (y_reg2)))>> 4) * 0.0009765);
	accel_data->z_axis =  ((float)((int16_t)(((z_reg1*256) + (z_reg2)))>> 4) * 0.0009765);
   
			
	return;

}


/* This routine displays the x, y, z accelerometer data. */
void display_data( mma8652Data_t *accel_data )
{
	printf("x %2.1fg y %2.1fg z %2.1fg\n", accel_data->x_axis, accel_data->y_axis, accel_data->z_axis);
	
	return;
}



// ISR for the int 1 interrupt
void int1_ISR( void )
{
	dataWaitingInt1 = 1;	// Set the flag toshow data waiting to be read
	
}	


// ISR for the int 2 interrupt
void int2_ISR( void )
{
	dataWaitingInt2 = 1;	// Set the flag toshow data waiting to be read
	
}	


// Routine to toggle the status LED
void toggleStatus( void )
{
	static char toggleState = 0;
	
	if( toggleState )
	{
		toggleState = 0;
		digitalWrite( STATUS_LED, LOW );	// Set LED off
	}
	else
	{
		toggleState = 1;
		digitalWrite( STATUS_LED, HIGH );	// Set LED on
	}
	
	
	return;
}


void statusOn( void )
{		
	digitalWrite( STATUS_LED, HIGH );	

	return;
}


void statusOff( void )
{ 
   	digitalWrite( STATUS_LED, LOW );	

	return;
}	
