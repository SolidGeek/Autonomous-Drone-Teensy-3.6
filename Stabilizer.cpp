#include "Stabilizer.h"

Stabilizer::Stabilizer(){

  /* Inner angular speed controllers */
  PitchSpeed.setConstants(1, 0, 0);
  RollSpeed.setConstants(1, 0, 0);

}

void Stabilizer::setup() {

	// Join I2C bus at 400kHz

	Wire.begin();
	Wire.setClock(400000);

	// Initialize the IMU with stardard parameters for calibration

	IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	IMU.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

	// this->calibrateIMU();

	// Now initiate with needed settings for DMP
	IMU.initialize();

	this->dmpStatus = IMU.dmpInitialize();

	// Set found offsets again
	// this->setIMUOffsets();

  IMU.setXAccelOffset(939);
  IMU.setYAccelOffset(-1627);
  IMU.setZAccelOffset(980);
  IMU.setXGyroOffset(104);
  IMU.setYGyroOffset(1);
  IMU.setZGyroOffset(27);

	// Check if
	if ( this->dmpStatus == 0) {

		#ifdef _DEBUG
			Serial.println( "Enabling DMP" );
		#endif

		IMU.setDMPEnabled(true);
		this->imuStatus = IMU.getIntStatus();

		this->dmpReady = true;
		
		// get expected DMP packet size for later comparison
		this->packetSize = IMU.dmpGetFIFOPacketSize();
		delay(2000);
			
	} else {
		#ifdef _DEBUG
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			Serial.print( "DMP Initialization failed (code ");
			Serial.print( this->dmpStatus );
			Serial.println( ")" );
		#endif
	}

}

bool Stabilizer::readDMPAngles() {

	if (!this->dmpReady) return false;

	// Get IMU status
	this->imuStatus = IMU.getIntStatus();

	// Get current FIFO count
	this->fifoCount = IMU.getFIFOCount();

	// Check for overflow
	if ((this->imuStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || this->fifoCount >= 1024) {

		// Reset so we can continue cleanly
		IMU.resetFIFO();
		this->fifoCount = IMU.getFIFOCount();

		#ifdef _DEBUG
			Serial.print( "FIFO overflow!" );
			Serial.println( this->imuStatus, BIN );
		#endif

		// Otherwise, check for DMP data ready interrupt
	} else if (imuStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
		// wait for correct available data length, should be a VERY short wait
		uint32_t test = micros();
		while (fifoCount < packetSize) fifoCount = IMU.getFIFOCount();
    
		// read a packet from FIFO
		IMU.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		IMU.dmpGetQuaternion( &q, fifoBuffer );
		IMU.dmpGetGravity( &gravity, &q);
		IMU.dmpGetYawPitchRoll( angles, &q, &gravity);

		// Convert radians to angles
		angles[0] *= (180 / M_PI);
		angles[1] *= (180 / M_PI);
		angles[2] *= (180 / M_PI);


    // Get gyro values for inner loop stabilization also
    IMU.dmpGetGyro( gyro, fifoBuffer );

		return true;

	}

	return false;

}

void Stabilizer::calibrateIMU() {

	// Mean should be within these defined deadzones
	uint8_t accel_deadzone = 8;
	uint8_t gyro_deadzone = 4;

	// Variables to hold mean
	int16_t ax_mean = 0, ay_mean = 0, az_mean = 0, gx_mean = 0, gy_mean = 0, gz_mean = 0;

	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	// The accelerometer is preset with offsets from factory - need to read them as they affect the output
	this->ax_offset = IMU.getXAccelOffset();
	this->ay_offset = IMU.getYAccelOffset();
	this->az_offset = IMU.getZAccelOffset();

	#ifdef _DEBUG
		Serial.println("-- Calculates offsets for IMU --");
	#endif

	while (1) {

		int ready = 0;
		int32_t ax_buf = 0, ay_buf = 0, az_buf = 0, gx_buf = 0, gy_buf = 0, gz_buf = 0;

		for ( uint8_t i = 0; i < 200; i++ ) {
			IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			ax_buf += ax;
			ay_buf += ay;
			az_buf += az;
			gx_buf += gx;
			gy_buf += gy;
			gz_buf += gz;
			delay(1);
		}

		ax_mean = ax_buf / 200;
		ay_mean = ay_buf / 200;
		az_mean = az_buf / 200;
		gx_mean = gx_buf / 200;
		gy_mean = gy_buf / 200;
		gz_mean = gz_buf / 200;

		/* Calculate accel offsets from mean divide by 8 to scale to +/-8g */
		if (abs(ax_mean) <= accel_deadzone) ready++;
			else this->ax_offset -= (ax_mean / 8);

		if (abs(ay_mean) <= accel_deadzone) ready++;
			else this->ay_offset -= (ay_mean / 8);

		if (abs(16384 - az_mean) <= accel_deadzone) ready++;
			else this->az_offset += (16384 - az_mean) / 8;

		/* Calculate gyro offsets from mean divided by 4 to scale to +/-1000dps */
		if (abs(gx_mean) <= gyro_deadzone) ready++;
			else this->gx_offset -= gx_mean / gyro_deadzone;

		if (abs(gy_mean) <= gyro_deadzone) ready++;
			else this->gy_offset -= gy_mean / gyro_deadzone;

		if (abs(gz_mean) <= gyro_deadzone) ready++;
			else this->gz_offset -= gz_mean / gyro_deadzone;


		#ifdef _DEBUG
			Serial.println("-- Calculated offsets --");

			Serial.print("Accel (xyz):"); Serial.print('\t');
			Serial.print(ax_offset); Serial.print('\t');
			Serial.print(ay_offset); Serial.print('\t');
			Serial.println(az_offset);

			Serial.print("Gyro (xyz):"); Serial.print('\t');
			Serial.print(gx_offset); Serial.print('\t');
			Serial.print(gy_offset); Serial.print('\t');
			Serial.println(gz_offset);
		#endif

		// Set newly calculated offsets such that next iteration comes closer
		IMU.setXAccelOffset(ax_offset);
		IMU.setYAccelOffset(ay_offset);
		IMU.setZAccelOffset(az_offset);
		IMU.setXGyroOffset(gx_offset);
		IMU.setYGyroOffset(gy_offset);
		IMU.setZGyroOffset(gz_offset);

		// All accel and gyro means are within the margins (offsets work)
		if (ready == 6) break;
	}
}

void Stabilizer::setIMUOffsets( void ) {
	IMU.setXAccelOffset(ax_offset);
	IMU.setYAccelOffset(ay_offset);
	IMU.setZAccelOffset(az_offset);
	IMU.setXGyroOffset(gx_offset);
	IMU.setYGyroOffset(gy_offset);
	IMU.setZGyroOffset(gz_offset);
}

// Should be run everytime new data from sensors is ready
void Stabilizer::motorMixing( int16_t &s1, int16_t &s2, int16_t &s3, int16_t &s4 ){

	// Thrust needed to lift (should be found with altitude controller)
	int16_t thrust = 250;

  // Outer controller loops
	Yaw.run( 0, angles[0] );
	Roll.run( 0, angles[1] );
	Pitch.run( 0, angles[2] );

  int16_t yaw_out = (int16_t)Yaw.getOutput();
  int16_t roll_out = (int16_t)Roll.getOutput();
  int16_t pitch_out = (int16_t)Pitch.getOutput();

  RollSpeed.run( roll_out, -gyro[1] );
  PitchSpeed.run( pitch_out, gyro[0] );
  
  int16_t tau_roll = (int16_t)RollSpeed.getOutput();
	int16_t tau_pitch = (int16_t)PitchSpeed.getOutput();

  // Speeds 3, 4, 1, 2 because IMU is turned 90 degress x = y, and y = x
  s3 = constrain( thrust - yaw_out + tau_pitch - tau_roll, 0, 2000 ) ; 
  s4 = constrain( thrust + yaw_out + tau_pitch + tau_roll, 0, 2000 ) ; 
  s1 = constrain( thrust - yaw_out - tau_pitch + tau_roll, 0, 2000 ) ; 
  s2 = constrain( thrust + yaw_out - tau_pitch - tau_roll, 0, 2000 ) ; 

}
