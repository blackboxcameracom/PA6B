/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : COMBINED_EXAMPLE/main.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Example program source code file for the MMA8652FC accelerometer
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
#include<math.h>
#include "../GPS_EXAMPLE/gps_functions.h"	// Defines data types and functions for accessing the GPS Receiver
#include "../MMA8652FC/MMA8652FC_funcs.h"	// Defines data types and functions for accessing the MMA8652 accelerometer


int main(int argc, char *argv[])
{

	mma8652Data_t accel_data;	// Data structure used to return data from the MMA8652FC

	const float alpha = 0.5;
	double fXg = 0;
	double fYg = 0;
	double fZg = 0;	
	double pitch, roll;
	
	
	if( !mma8652SetUp() )	// Set up the MMA8652 connection & configuration 
	{
			return 0;
	}
	
//---
	
	// Open the UART and connect to the GPS receiver.
	if( openGPSUART(PA6B, 9600, 9600, 1000 ) <0 )
	{	// Abort if UART open fails								
        return -1;
	}
 
	startSerialThread();	// 	Start the thread that reads the GPS NMEA data from the serial port
	
	printf( "Thread started\n");

//---	

	setTimerAlarmInterval( 3  );	// Set a timer interval of 3 minutes 
   
	// Loop waiting for characters to arrive from the GPS receiver
	do 
	{
		/* Accelerometer data can be read from the MMA8652 either by polling the device's status
		 * register or in response to an interrupt on one of the device's two interrupt pins.
		 * Here polling is used.  The interrupts will not work unless the device 
		 * pins are connected to the GPIO by jumpers or wire links.  */
		if( pollMMA8652() )	// Check the bit indicating data waiting to be read
		{
			read_MMA8652( &accel_data );
			
			// display_data( &accel_data);	// Display raw accelerometer data if reqd.

			// Low Pass Filter
			// from https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
		    fXg = accel_data.x_axis * alpha + (fXg * (1.0 - alpha));
		    fYg = accel_data.y_axis * alpha + (fYg * (1.0 - alpha));
		    fZg = accel_data.z_axis * alpha + (fZg * (1.0 - alpha));
		 
			//Roll & Pitch Equations
		    roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
		    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
		    
		    printf("roll %3.1f pitch %3.1f\n", roll, pitch);
			
			toggleStatus();
									
		}

		if( rmc_recvd && gprmc.dataValid )
		{
			rmc_recvd = FALSE;	
			
			printf("%s %s\n", gprmc.time, gprmc.date );
			printf("Lat %lf Lon %lf Speed %3.2lf knots heading %3.2lf degrees\n", gprmc.latitude, gprmc.longitude, gprmc.speed, gprmc.course);
	
		}		
		
		if( gga_recvd && gpgga.dataValid ) //Synchronise altitude display with GGA data
		{
			gga_recvd = FALSE;	
			
			printf("Altitude %5.2lfm\n", gpgga.altitude);
			
		}
		
	}
	while( !timesUp );	// Uncomment for timed loop set by period of alarm interrupt  
	// while( 1 );    // Uncomment for endless loop 
   
	closeGPSUART();
	
	return 0;
}
