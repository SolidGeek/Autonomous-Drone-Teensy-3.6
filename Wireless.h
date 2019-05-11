#ifndef _WIRELESS_H_
#define _WIRELESS_H_

#include "Arduino.h"

#define WIFIPORT Serial6


typedef struct {

	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	float roll, pitch, yaw, altitude;

	float rError, rAccum, rDeriv;
	float pError, pAccum, pDeriv;
	float yError, yAccum, yDeriv;
	float aError, aAccum, aDeriv;

	float voltage;

	// Speeds
	uint16_t s1, s2, s3, s4;

} Telemetry;



class Wireless {


public: 

	void setup( void );

	// Listen for incoming data
	bool listen( void );

	// Send serial data
	void sendBuffer( void );

	// Send entire telemetry package
	void sendTelemetry( void  );

	bool check( char * name );

	bool value( char * name, double * var );
	bool value( char * name, int * var );

	bool addToBuffer( double value );
	bool addToBuffer( int value );
	bool addToBuffer( char * value );


private:

	bool isConnected = false;

	char buffer[200] = {'\0'};
	char params[200] = {'\0'};


};

#endif