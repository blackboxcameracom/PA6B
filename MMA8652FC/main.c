/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : mma8652_Example2/main.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: main.c file showing usage of the MMA8652FC accelerometer interface functions
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

#include<stdio.h>
#include<stdint.h>

#include "MMA8652FC_funcs.h"


int main()
{
	int num_tries;
	mma8652Data_t accel_data;
	
	if( !mma8652SetUp() )	// Set up the MMA8652 connection & configuration 
	{
			return 0;
	}
		

	statusOn();		// Set status LED state to high, LED on at the start.
	
	delay( 1000 );

	/* Accelerometer data can be read from the MMA8652 either by polling the device's status
	 * register or in response to an interrupt on one of the device's two interrupt pins.
	 * Here both methods are demonstrated.  The interrupts will not work unless the device 
	 * pins are connected to the GPIO by jumpers or wire links.  Although in this example 
	 * both interrupts are used to signal data waiting, other interrupt sources which can
	 * be set are freefall, motion, pulse and transient.  For details see the MMA8652 
	 * documentation. */
	 
	printf("Read data using polling\n");


	num_tries = 0;
	
	while( num_tries  < 100 )
	{
		if( pollMMA8652() )	// Check the bit indicating data waiting to be read
		{
			read_MMA8652( &accel_data );
			
			display_data( &accel_data);
			
			toggleStatus();
			
			num_tries++;			
		}
	}

	delay( 1000 );


	printf("Read data using INT 1\n");
	setUpMMA8652Int1();
	
	num_tries = 0;
	
	while( num_tries  < 100 )
	{
		if( dataWaitingInt1 )	// Check the bit indicating data waiting to be read
		{
			dataWaitingInt1 = 0;

			read_MMA8652( &accel_data );
			
			display_data( &accel_data );
			
			toggleStatus();
			
			num_tries++;
		}		
		
	}

	delay( 1000 );


	printf("Read data using INT 2\n");

	setUpMMA8652Int2();
		
	num_tries = 0;
	
	while( num_tries  < 100 )
	{
		
		if( dataWaitingInt2 )	// Check the bit indicating data waiting to be read
		{
			dataWaitingInt2 = 0;

			read_MMA8652( &accel_data );
			
			display_data( &accel_data );
			
			toggleStatus();
			
			num_tries++;
			
		}	
		
	}

	statusOff();	// Leave the program with the status LED off
	 

   
	return 0;
}	
	
