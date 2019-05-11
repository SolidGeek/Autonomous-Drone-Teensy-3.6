#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include <Arduino.h>
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20  // Needed to pre-declare the DMP functions
#include "helper_3dmath.h"                // Needed to use the Quaternion and VectorFloat data types
#include <MPU6050.h> // IMU library
#include "Settings.h"
#include "Controller.h"
#include <VL53L1X.h> // Height sensor library

#define _DEBUG

/* Stabilizer class */

class Stabilizer
{

public:

	Settings config;

	float angles[3]; // Yaw, roll, pitch
	int16_t gyro[3]; // X, Y, Z
	int16_t accel[3]; // X, Y, Z

	Stabilizer( void );

	void setup( void );

	void calibrateIMU( void );	
	bool readDMPAngles( void );

	void motorMixing( void ); 

	void setHome( void );

	// Motor speeds 
	int16_t s1 = 0;
	int16_t s2 = 0;
	int16_t s3 = 0;
	int16_t s4 = 0;

	bool motorsOn = false;

private:

	// Altitude sensor object
	VL53L1X altitude;

	// Attitude sensor object
	MPU6050 IMU;

	// Yaw home
	float yawRef = 0.0;

	// Yaw setpoint (for controlling front of drone)
	float yawSetpoint = 0.0;
	

	// Inner loop controllers 
	Controller RollSpeed;
	Controller PitchSpeed; 
	Controller YawSpeed;

	// Outer loop controllers
	Controller Yaw;
	Controller Roll;
	Controller Pitch; 


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

	void setIMUOffsets( void );

};

#endif
