#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include <Arduino.h>
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20  // Needed to pre-declare the DMP functions
#include "helper_3dmath.h"                // Needed to use the Quaternion and VectorFloat data types
#include <MPU6050.h>
#include "Controller.h"

#define _DEBUG

/* Stabilizer class */

class Stabilizer
{

public:

	float angles[3]; // Yaw, roll, pitch
  int16_t gyro[3];

	Stabilizer( void );

	void setup( void );

	void calibrateIMU( void );	
	bool readDMPAngles( void );

	void motorMixing( int16_t &s1, int16_t &s2, int16_t &s3, int16_t &s4 ); 

  Controller Yaw;
  Controller Roll;
  Controller Pitch; 

private:

  // Not need to be public, as they only use a p-controller with a scale of 1
  Controller RollSpeed;
  Controller PitchSpeed; 

	MPU6050 IMU;

	Quaternion q; 
	VectorFloat gravity;   

	bool dmpReady = false;  // Set true if DMP init was successful
	uint8_t imuStatus;   	// Holds actual status byte from IMU
	uint8_t dmpStatus;      // Status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)

	uint16_t fifoCount;     // Count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// IMU offsets found by calibration
	int16_t ax_offset = 0, ay_offset = 0, az_offset = 0 ,gx_offset = 0, gy_offset = 0, gz_offset = 0;


	void setIMUOffsets( void );

};

#endif
