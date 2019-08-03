/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : MMA8652FC/MMA8652FC_funcs.h
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface Functions header file for the MMA8652FC accelerometer
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

extern volatile char dataWaitingInt1;	// Flag set in the interrupt service rountines
extern volatile char dataWaitingInt2;	


struct mma8652Data {
	float x_axis;	// variables for the converted MMA8652 accelerometer data
	float y_axis;
	float z_axis;	

};
typedef struct mma8652Data mma8652Data_t;

extern void statusOff( void );			// Turn off the Status LED
extern void statusOn( void );			// Turn on the Status LED
extern void toggleStatus( void );		// Toggle the state of the status LED
extern void read_MMA8652( mma8652Data_t * );	// Read and convert the MMA8652 accelerometer data
extern void display_data( mma8652Data_t * );	// Display the MMA8652 accelerometer data
extern void setUpMMA8652Int1( void );	// Set up the MMA8653 interrupt on pin 1, example is the data waiting interrupt
extern void setUpMMA8652Int2( void );	// Set up the MMA8653 interrupt on pin 2, example is the data waiting interrupt
extern int pollMMA8652( void );			// Poll the MMA8652 to see if there is data waiting
extern int mma8652SetUp( void );		// Set up the MMA8652 connection and configuration


