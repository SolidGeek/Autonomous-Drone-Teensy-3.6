    
#ifndef _DRONE_SETTINGS_h
#define _DRONE_SETTINGS_h

#include <Arduino.h>
#include <EEPROM.h>

class Settings
{

public:

	/* Settings variables */

	double yawKp = 0.0;
	double yawKi = 0.0;
	double yawKd = 0.0;

	double pitchKp = 0.0;
	double pitchKi = 0.0;
	double pitchKd = 0.0;

	double rollKp = 0.0;
	double rollKi = 0.0;
	double rollKd = 0.0;

	double altKp = 0.0;
	double altKi = 0.0;
	double altKd = 0.0;

	// int16_t speedOff1 = 8, speedOff2 = 0, speedOff3 = 9, speedOff4 = 3;
	int offset1 = 0;
	int offset2 = 0;
	int offset3 = 0;
	int offset4 = 0;

	int hoverOffset = 0;

public:
	/* Settings functions */

	Settings( void );

	void setValue( uint8_t index, uint16_t value );

	uint16_t getValue( uint8_t index );

	void load( void );

	void save( void );

  float yawRef = 0.0;

};

#endif
