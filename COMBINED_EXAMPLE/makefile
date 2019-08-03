CC=gcc

CFLAGS=

combined_example: main.o ../GPS_EXAMPLE/nmea.o ../GPS_EXAMPLE/gps_functions.o ../GPS_EXAMPLE/gps_functions.o ../MMA8652FC/MMA8652FC_funcs.o 
	gcc main.c ../GPS_EXAMPLE/nmea.o ../GPS_EXAMPLE/gps_functions.o ../MMA8652FC/MMA8652FC_funcs.o -o combined_example -lm -lwiringPi -lpthread