#include "Wireless.h"

void Wireless::setup( void ){
	// The WiFi module is programmed seperately as a simple "Serial <--> Websocket" module
	// There is nothing to do, but begin UART communication to the module
	WIFIPORT.begin(115200);
}

// Listen for incoming data
bool Wireless::listen( void ){

	if( WIFIPORT.available() > 0 ){

		memset(buffer, 0, sizeof(buffer));
		WIFIPORT.readBytesUntil( '\n', buffer, sizeof(buffer) );

		return true;

	}

	return false;

}

// Send serial data
void Wireless::send( char * data ){
	WIFIPORT.println( data );
}

// Send entire telemetry package
void Wireless::sendTelemetry( void  ){


}

bool Wireless::check( char * name ){

	if( strstr( this->buffer, name ) != NULL ){

		// Find the end of the "command" and save this pointer in the params variable for variable extraction
		this->params = strpbrk(this->buffer, " ");

		return true;

	}

	return false;
}


void Wireless::clearBuffer( void ){
	memset(buffer, 0, sizeof(buffer));
}

bool Wireless::value( char * name, int * var ){

	double temp;

	if( this->value( name, &temp ) ){

		*var = (int)temp;
		return true; 

	}

	return false;

}

// Get value from received string
bool Wireless::value( char * name, double * var ){

	char * start;
	char * end;
	size_t len; 

	char buf[20] = {'\0'};

	// Find start of parameter value
	if( start = strstr( this->buffer, name )){

		start++; // Not interested in the param name

		// Find end of parameter value
		if( end = strpbrk(start, " \n") ){

			len = end - start;

			strncpy(buf, start, len);
			buf[len] = '\0';

			// Now convert the string in buf to a float
			*var = atof(buf);
     
			return true;

		}

	}

	return false;

}