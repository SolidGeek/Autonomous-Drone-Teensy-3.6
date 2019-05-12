#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <Arduino.h>

class Controller{

public:

	Controller( void );
	Controller( float Kp, float Ki, float Kd);

	void run(float setpoint, float input );
	float getOutput( void);

	void setConstants( float p, float i, float d );
  void setConstants( float * params );
	void setMaxIntegral( float value );
	void setMaxOutput( float value );

	void reset( void );

private: 

	float Kp = 0.0;
	float Ki = 0.0;
	float Kd = 0.0;

	float output = 0.0;
	float error = 0.0;
	float lastError = 0.0;
	float integral = 0.0;
	float derivative = 0.0;

	/* Used for integral windup */
	float maxIntegral = 0.0;
	float maxOutput = 0.0;

	uint32_t lastTime = 0;

};
#endif
