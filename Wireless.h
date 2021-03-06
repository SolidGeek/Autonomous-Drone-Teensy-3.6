#ifndef _WIRELESS_H_
#define _WIRELESS_H_

#include "Arduino.h"

#define WIFIPORT Serial6

class Wireless {
public: 

	void setup( void );

	// Listen for incoming data
	bool listen( void );

	// Send serial data
	void sendBuffer( void );
  void send( char * data );

	// Send entire telemetry package
	void sendTelemetry( void  );

	bool check( const char * name );

	bool value( const char * name, float * var );
	bool value( const char * name, int * var );

  void clearBuffer( void );

	void addToBuffer( float value );
	void addToBuffer( int value );
	void addToBuffer( const char * value );

  char buffer[200] = {'\0'};
  char params[200] = {'\0'};

private:

	bool isConnected = false;

	
	


};

#endif
