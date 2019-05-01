    
#ifndef _DRONE_SETTINGS_h
#define _DRONE_SETTINGS_h

#include <Arduino.h>
#include <EEPROM.h>

class Settings
{

public:

	/* Settings variables */

	float yawKp = 0.0;
	float yawKi = 0.0;
	float yawKd = 0.0;

	float pitchKp = 0.0;
	float pitchKi = 0.0;
	float pitchKd = 0.0;

	float rollKp = 0.0;
	float rollKi = 0.0;
	float rollKd = 0.0;


public:
	/* Settings functions */

	Settings( void );

	void setValue( uint8_t index, uint16_t value );

	uint16_t getValue( uint8_t index );

	void load( void );

	void save( void );

};

#endif
