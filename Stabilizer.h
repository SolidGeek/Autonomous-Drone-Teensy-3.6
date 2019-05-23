#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include <Arduino.h>
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20  // Needed to pre-declare the DMP functions
#include "helper_3dmath.h"                // Needed to use the Quaternion and VectorFloat data types
#include <MPU6050.h> // IMU library
#include "DSHOT.h"
#include "Settings.h"
#include "Controller.h"
#include <VL53L1X.h> // Height sensor library

#define _DEBUG

#define DSHOT_TLM_INTERVAL 2500 // 2500 us = 400 Hz
#define ALT_OFFSET 50.0f

#define TLM1 Serial2
#define TLM2 Serial3
#define TLM3 Serial4
#define TLM4 Serial5

/* Stabilizer class */

class Stabilizer
{

public:

  uint32_t lastRpmControl = 0;

	Settings config;

	float angles[3] = {0}; // Yaw, roll, pitch
	int16_t gyro[3] = {0}; // X, Y, Z
	int16_t accel[3] = {0}; // X, Y, Z

  int16_t filtAccel[3] = {0}; // X, Y, Z
	float height = 0.0;
  bool newAlt = false;

	Stabilizer( void );

	void setup( void );

	void calibrateIMU( void );	
	bool readDMPAngles( void );

	void readAltitude( void );

	void motorMixing( void ); 

	// When ever telemetry is wanted from all motors, loop this for a while (main loop)
	void getTelemetry( void );

	void setMotorSpeeds( void );

  void setSameThrottle( uint16_t value );
  
	void stopMotors( void );

	void armMotors( void );

  void motorRPMControl( void );

	void setHome( void );

	float batteryVoltage( void );

	// Motor speeds 
	int16_t s1 = 0;
	int16_t s2 = 0;
	int16_t s3 = 0;
	int16_t s4 = 0;

  // Actual RPM
  float rpm[4] = {0.0};

  // Wanted RPM
  float rpmRef[4] = {0.0};
  
	bool motorsOn = false;

  // Inner loop controllers 
  Controller RollRate;
  Controller PitchRate; 
  Controller YawRate;

  // Outer loop controllers
  Controller Yaw;
  Controller Roll;
  Controller Pitch;

  Controller Altitude;

  Controller Motor1;
  Controller Motor2;
  Controller Motor3;
  Controller Motor4;

    // Motor objects
  DShot * ESC1;
  DShot * ESC2;
  DShot * ESC3;
  DShot * ESC4;
  
private:

	// Altitude sensor object (TOF)
	VL53L1X TOF;

	// Attitude sensor object (IMU)
	MPU6050 IMU;



	// Yaw home
	float yawRef = 0.0;

	// Yaw setpoint (for controlling front of drone)
	float yawSetpoint = 0.0;

	// Roll and pitch setpoints for controlling position movement
	float rollSetpoint = 0.0;
	float pitchSetpoint = 0.0;
	
	// Variables to calculate Eulor angles (Roll, Pitch & Yaw)
	Quaternion q; 
	VectorFloat gravity;   

	/* -- IMU variables for DMP extraction -- */

	bool dmpReady = false;  // Set true if DMP init was successful
	uint8_t imuStatus;   	// Holds actual status byte from IMU
	uint8_t dmpStatus;      // Status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // Count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// IMU offsets found by calibration
	int16_t ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;

	uint32_t lastTlmReq = 0;

	void initIMU( void );

	void initTOF( void );

	void setIMUOffsets( void );

  float EMA( float newSample, float oldSample, float alpha );

};

#endif
