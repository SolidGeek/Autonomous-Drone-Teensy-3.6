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
  WIFIPORT.write( data );
  WIFIPORT.write('\n');
}

void Wireless::sendBuffer( ){
  WIFIPORT.write( this->buffer );
  WIFIPORT.write('\n');
}


// Send entire telemetry package
void Wireless::sendTelemetry( void  ){


}

bool Wireless::check( const char * name ){
  
  char command[20] = {'\0'};

  // The first part is always the command
  char * start = &this->buffer[0];
  char * end = strpbrk(this->buffer, " ");

  uint8_t len = end - start;
  strncpy ( command, this->buffer, len );

  if ( strcmp(command, name) == 0 ) {
    strcpy( params, end );
    return true;

  }

  return false;

}


void Wireless::clearBuffer( void ){
	memset(buffer, 0, sizeof(buffer));
}

bool Wireless::value( const char * name, int * var ){

	float temp;

	if( this->value( name, &temp ) ){

		*var = (int)temp;
		return true; 

	}

	return false;

}

// Get value from received string
bool Wireless::value( const char * name, float * var ){

	char * start;
	char * end;
	size_t len; 

	char buf[20] = {'\0'};

	// Find start of parameter value
	if( start = strstr( this->params, name )){

		start++; // Not interested in the param name

		// Find end of parameter value
		if( end = strpbrk(start, " \n") ){

			len = end - start;

			strncpy(buf, start, len);
			buf[len] = '\0';

			// Now convert the string in buf to a float
			*var = (float)atof(buf);
     
			return true;

		}

	}

	return false;

}

void Wireless::addToBuffer( float value ){
	
	char tempBuffer[20] = {'\0'};

	sprintf(tempBuffer, "%.2f", value );
	strcat(buffer, tempBuffer);  
	strcat(buffer, " ");

}

void Wireless::addToBuffer( int value ){
	
	char tempBuffer[20] = {'\0'};

	sprintf(tempBuffer, "%d", value );
	strcat(buffer, tempBuffer);  
	strcat(buffer, " ");

}


void Wireless::addToBuffer( const char * value ){
	
	strcat(buffer, value);  
	strcat(buffer, " ");

}
