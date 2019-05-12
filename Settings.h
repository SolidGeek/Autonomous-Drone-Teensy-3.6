    
#ifndef _DRONE_SETTINGS_h
#define _DRONE_SETTINGS_h

#include <Arduino.h>
#include <EEPROM.h>

class Settings
{

public:

	/* Settings variables */

  float yawPID[3] = {0.0};
  float yawRatePID[3] = {0.0};

  float pitchPID[3] = {0.0};
  float pitchRatePID[3] = {0.0};

  float rollPID[3] = {0.0};
  float rollRatePID[3] = {0.0};

  float altPID[3] = {0.0};

	// int16_t speedOff1 = 8, speedOff2 = 0, speedOff3 = 9, speedOff4 = 3;
  int motorOffset[4] = {0};

	int hoverOffset = 0;

public:
	/* Settings functions */

	Settings( void );

	void load( void );

	void save( void );

  float yawRef = 0.0;

};

#endif
