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

	void begin( void );

	// Listen for incoming data
	void listen( void );

	// Send serial data
	void send( char * data );

	// Send entire telemetry package
	void sendTelemetry( void  );


private:

	bool isConnected = false;

	char buffer[200] = { '\0' };


};

#endif