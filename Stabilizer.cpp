#include "Stabilizer.h"

Stabilizer::Stabilizer(){

  ESC1 = new DShot(1);
  ESC2 = new DShot(2);
  ESC3 = new DShot(3);
  ESC4 = new DShot(4);

  /* Motor speed controllers */
  Motor1.setConstants(0.3, 1.5, 0);
  Motor2.setConstants(0.3, 1.5, 0);
  Motor3.setConstants(0.3, 1.5, 0);
  Motor4.setConstants(0.3, 1.5, 0);

  /* Integral windup */
  Motor1.setMaxIntegral(600.0);
  Motor2.setMaxIntegral(600.0);
  Motor3.setMaxIntegral(600.0);
  Motor4.setMaxIntegral(600.0);
  Motor1.absIntegral = true;
  Motor2.absIntegral = true;
  Motor3.absIntegral = true;
  Motor4.absIntegral = true;
  Motor1.startupIntegral = true;
  Motor2.startupIntegral = true;
  Motor3.startupIntegral = true;
  Motor4.startupIntegral = true;

  Altitude.startupIntegral = true;
  Altitude.absIntegral = true;
  Altitude.setMaxIntegral(2000);
  Altitude.setMaxOutput(500);

  Pitch.setMaxIntegral(6.0);
  Roll.setMaxIntegral(6.0);
  Yaw.setMaxIntegral(10.0);

  PitchRate.setMaxIntegral(10.0);
  RollRate.setMaxIntegral(10.0);

  RollPos.setMaxIntegral(25.0);
  RollPos.setMaxOutput(3);
  RollPos.startupIntegral = true;
  PitchPos.setMaxIntegral(25.0);
  PitchPos.setMaxOutput(3);
  PitchPos.startupIntegral = true;

  RollPos.setConstants(0.1, 0.03, 0);
  PitchPos.setConstants(0.1, 0.03, 0);
  
}

void Stabilizer::setup() {

	// Load config from EEPROM
  // config.save(); // to refresh config
	config.load();

	// Begin I2C bus at 400kHz
	Wire.begin();
	Wire.setClock(400000);

	// Begin UART communication for ESC telemetry
	TLM1.begin(115200);
	TLM2.begin(115200);
	TLM3.begin(115200);
	TLM4.begin(115200);

  // Init IMU for angle measurement
  initIMU();

  // Init TOF for height measurement
  initTOF();


  // PIN 15 as slave select (SS)
  pixy.init();
  delay(200);
  pixy.changeProg("line");
  

	// Setup of motor directions
	ESC1->setDirection(false);
	ESC2->setDirection(true);
	ESC3->setDirection(false);
	ESC4->setDirection(true);

  delay(500);

  armMotors();

	// Load PID parameters from config into controllers
	Yaw.setConstants( config.yawPID );
	Roll.setConstants( config.rollPID );
	Pitch.setConstants( config.pitchPID );
  YawRate.setConstants( config.yawRatePID );
  RollRate.setConstants( config.rollRatePID );
  PitchRate.setConstants( config.pitchRatePID );
  Altitude.setConstants( config.altPID );
}

void Stabilizer::initTOF( void ){

  if (!TOF.begin())
  {
    #ifdef _DEBUG
      Serial.println("No altitude sensor");
    #endif
  }

	TOF.setDistanceModeShort();
}

void Stabilizer::initIMU(){
	// Initialize the IMU with stardard parameters for calibration
	IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	IMU.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

	// this->calibrateIMU();
	// Now initiate with needed settings for DMP
	IMU.initialize();

	this->dmpStatus = IMU.dmpInitialize();

	// Load IMU offsets found using calibrateIMU()
	IMU.setXAccelOffset(930);
	IMU.setYAccelOffset(-1658);
	IMU.setZAccelOffset(972);
	IMU.setXGyroOffset(103);
	IMU.setYGyroOffset(6);
	IMU.setZGyroOffset(29);

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
		#endif

		// Otherwise, check for DMP data ready interrupt
	} else if (imuStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
		// wait for correct available data length, should be a VERY short wait

    uint32_t timeoutTimer = micros();
		while (fifoCount < packetSize && (micros() - timeoutTimer < 100) ){
		  fifoCount = IMU.getFIFOCount();
		}

    // Return false if while-loop was exited
    if( micros() - timeoutTimer > 100 ) {
      return false;
    } 
    
		// read a packet from FIFO
		IMU.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// Calculate Euler angles from quaternions and gravity
		IMU.dmpGetQuaternion( &q, fifoBuffer );
		IMU.dmpGetGravity( &gravity, &q);
		IMU.dmpGetYawPitchRoll( angles, &q, &gravity);

		// Convert radians to angles
		angles[0] *= (180 / M_PI);
		angles[1] *= (180 / M_PI);
		angles[2] *= (180 / M_PI);

    anglesFilt[0] = EMA( angles[0], anglesFilt[0], 0.25 );
    anglesFilt[1] = EMA( angles[1], anglesFilt[1], 0.25 );
    anglesFilt[2] = EMA( angles[2], anglesFilt[2], 0.25 );
    
    // Get gyro values for inner loop stabilization also
    IMU.dmpGetGyro( gyro, fifoBuffer );

    gyroTemp[0] = (float)gyro[0] / 100000.0; // Keep precision but not
    gyroTemp[1] = (float)gyro[1] / 100000.0;
    gyroTemp[2] = (float)gyro[2] / 100000.0;
    
    gyroFilt[0] = EMA( gyroTemp[0], gyroFilt[0], 0.15);
    gyroFilt[1] = EMA( gyroTemp[1], gyroFilt[1], 0.15);
    gyroFilt[2] = EMA( gyroTemp[2], gyroFilt[2], 0.15);
    
    // Get accelelometer values 

    /* IMU.dmpGetAccel( accel, fifoBuffer ); // Not used so lets save some processing

    if( anglesSettled == true ){

      accelDir[0] = accel[0] - (int16_t)(8192.0 * angles[1] * DEGTORAD); 
      accelDir[1] = accel[1] - (int16_t)(8192.0 * angles[2] * DEGTORAD); 
  
      float accelDirX = accelDir[0]; // EMA( accelDirX, accelDir[0], 0.01 );
      float accelDirY = accelDir[0]; // EMA( accelDirY, accelDir[1], 0.01 ); 
  
      uint32_t deltaT = micros() - velocityTimer; 
      velocityTimer = micros();
      
      velocity[0] +=  accelDirX * (float)(deltaT)/1000000.0;
      velocity[1] +=  accelDirY * (float)(deltaT)/1000000.0;
    
      velocity[0] *= 0.995;
      velocity[1] *= 0.995;

    }*/ 

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

// Should be run everytime new data from sensors is ready to calculate motor speeds
void Stabilizer::motorMixing( ){

  uint32_t startupTiming = 0;
  float startupRpm = 0;

	// Read angles from DMP (digital motion processor)
	readDMPAngles();

	// Read altitude from TOF sensor
	readAltitude();
  Altitude.run( heightRef, height );
    
  float altOffset = Altitude.getOutput() + (float)config.hoverOffset; // In RPM

  // Serial.println( altOffset );

	// Calculate yaw angle according to drones reference frame (0 = startup angle)
	float alpha_real = angles[0];
	float alpha_ref = yawRef;
	float droneYaw = alpha_real - alpha_ref;

	if( droneYaw > 180.0 ){
		yaw = droneYaw - 360.0;
	}
	else if( droneYaw < -180.0 ){
		yaw = droneYaw + 360.0;
	}
	else
		yaw = droneYaw; 
  
  getCamPosition();

  RollPos.run( 0, deltaRoll ); 
  PitchPos.run( 0, deltaPitch ); 

  float rollpos_out = RollPos.getOutput();
  float pitchpos_out = PitchPos.getOutput();


	// Outer controller loops
	Yaw.run( yawSetpoint, yaw );
	Roll.run( rollSetpoint, angles[1] );
	Pitch.run( pitchSetpoint, angles[2] );

	// Output from outer controllers
	float yaw_out = Yaw.getOutput();
	float roll_out = Roll.getOutput();
	float pitch_out = Pitch.getOutput();

	// Inner controller loops
	YawRate.run( yaw_out, gyroFilt[2] );
	RollRate.run( roll_out, -gyroFilt[1] );
	PitchRate.run( pitch_out, gyroFilt[0] );

	// Output from inner controllers (in RPM)
	float tau_roll  = RollRate.getOutput();
	float tau_pitch = PitchRate.getOutput();
	float tau_yaw   = YawRate.getOutput();
  
  
  // Motor start-up sequence
  if( rpmStartup == false ){
  
    rpmRef[0] = 2000;
    rpmRef[1] = 2000;
    rpmRef[2] = 2000;
    rpmRef[3] = 2000;
    
    if( (rpm[0] > 1500) && (rpm[1] > 1500) && (rpm[2] > 1500) && (rpm[3] > 1500) ){
      if( rpmTimerStarted == false ){
        rpmStartupTimer = millis();
        rpmTimerStarted = true;
      }
    }else{
      rpmTimerStarted = false;  
    }

    if( millis() - rpmStartupTimer > 2000 && rpmTimerStarted ){
      startupTiming = millis() - rpmStartupTimer;

      startupRpm = map( startupTiming, 2000, 5000, 2000, config.hoverOffset);
      rpmRef[0] = startupRpm;
      rpmRef[1] = startupRpm;
      rpmRef[2] = startupRpm;
      rpmRef[3] = startupRpm;
      
    }

    if( millis() - rpmStartupTimer > 5000 && rpmTimerStarted )
      rpmStartup = true;
    
  }else{
    // Motor mixing (MMA)
    rpmTimerStarted = false;
    rpmRef[0] = altOffset - tau_yaw - tau_pitch + tau_roll;
    rpmRef[1] = altOffset + tau_yaw - tau_pitch - tau_roll;
    rpmRef[2] = altOffset - tau_yaw + tau_pitch - tau_roll;
    rpmRef[3] = altOffset + tau_yaw + tau_pitch + tau_roll;   
  }

  // Serial.println(angles[1]);
  
  /* 
   *  Motor speed RPM test
   *  
   *  Serial.print( micros() ); Serial.print(';'); */
   
  /* Serial.print( rpm[0] ); Serial.print('\t');
  Serial.print( rpm[1] ); Serial.print('\t');
  Serial.print( rpm[2] ); Serial.print('\t');
  Serial.println( rpm[3] ); */

  /* Serial.print( RollRate.error ); Serial.print('\t');
  Serial.print( RollRate.output );Serial.print('\t');
  Serial.println( RollRate.integral );*/


  /* Serial.print( Altitude.error ); Serial.print('\t');
  Serial.print( Altitude.integral );Serial.print('\t');
  Serial.println( Altitude.output ); */
  
  
  /* Serial.print( millis() ); Serial.print(';');
  Serial.println( angles[2] ); */
  /* Serial.print( gyroFilt[0]); Serial.print(',');
  Serial.println( gyro[0]); // Serial.print(','); */
  //Serial.println( gyroFilt[2]);
	// Speeds 3, 4, 1, 2 because IMU is turned 90 degress x = y, and y = x

	

}

void Stabilizer::motorRPMControl( void ){

  static uint16_t tlmLostCount = 0;
  uint8_t tlmCount = 0;
  uint32_t now = micros();

  if( (now - ESC1->tlm.timestamp) < 2500 )
    tlmCount++;

  if( (now - ESC2->tlm.timestamp) < 2500 )
    tlmCount++;

  if( (now - ESC3->tlm.timestamp) < 2500 )
    tlmCount++;

  if( (now - ESC4->tlm.timestamp) < 2500 )
    tlmCount++;

  if( tlmCount == 4 ){

    tlmLostCount = 0;
    /* uint32_t times = micros() - lastRpmControl;
    lastRpmControl = micros();
    Serial.print("us: ");
    Serial.println( times ); */
    
    // Control Motor RPM
    rpm[0] = EMA( ESC1->tlm.rpm, rpm[0], 0.3 );
    rpm[1] = EMA( ESC2->tlm.rpm, rpm[1], 0.3 );
    rpm[2] = EMA( ESC3->tlm.rpm, rpm[2], 0.3 );
    rpm[3] = EMA( ESC4->tlm.rpm, rpm[3], 0.3 );    
    
    // Calculate RPM controller 
    Motor1.run( rpmRef[0], rpm[0] );
    Motor2.run( rpmRef[1], rpm[1] );
    Motor3.run( rpmRef[2], rpm[2] );
    Motor4.run( rpmRef[3], rpm[3] );

    // Calculate actual DShot output signal
    s1 = constrain( Motor1.getOutput(), 1, 2000 ) ; 
    s2 = constrain( Motor2.getOutput(), 1, 2000 ) ; 
    s3 = constrain( Motor3.getOutput(), 1, 2000 ) ; 
    s4 = constrain( Motor4.getOutput(), 1, 2000 ) ; 

    // Reset telemetry timestamp such that we run at same frequency as DShot
    ESC1->tlm.timestamp = 0;
    ESC2->tlm.timestamp = 0;
    ESC3->tlm.timestamp = 0;
    ESC4->tlm.timestamp = 0;
      
  }else{
    tlmLostCount++;
    if( tlmLostCount >= 10 ){
      motorsOn = false;
    }
  }
    
}


void Stabilizer::setMotorSpeeds( void ){

	if( motorsOn == true ){

		ESC1->setThrottle( s1 );
		ESC2->setThrottle( s2 );
		ESC3->setThrottle( s3 );
		ESC4->setThrottle( s4 );

	}else{

		// Send zero throttle
		setSameThrottle(1); // Send a 1 to get telemetry

	}

}

void Stabilizer::setSameThrottle( uint16_t value ){
  ESC1->setThrottle( value );
  ESC2->setThrottle( value );
  ESC3->setThrottle( value );
  ESC4->setThrottle( value );  
}

void Stabilizer::stopMotors( void ){
	ESC1->setThrottle( 0 );
	ESC2->setThrottle( 0 );
	ESC3->setThrottle( 0 );
	ESC4->setThrottle( 0 );
}

void Stabilizer::armMotors( void ){

	// Send 0 value for 1 second
	for( int i = 0; i < 500; i++ ){
		stopMotors();
		delayMicroseconds(2000);
	}

}

void Stabilizer::readAltitude( void ){

  TOF.startRanging(); //Write configuration bytes to initiate measurement

  height = TOF.getDistance() - ALT_OFFSET; //Get the result of the measurement from the sensor

  TOF.stopRanging();

	/* if( TOF.dataReady() ){

    height = EMA( (float)TOF.read() - ALT_OFFSET, height, 0.5 );
    if ( height < 0.0 )
      height = 0.0;

	}*/

}

void Stabilizer::getTelemetry( void ){

	// Only request telemetry once in a while
	/* if( micros() - lastTlmReq > DSHOT_TLM_INTERVAL ){

		lastTlmReq = micros();
		ESC1->requestTelemetry(); 
		ESC2->requestTelemetry(); 
		ESC3->requestTelemetry(); 
		ESC4->requestTelemetry();

	}*/

	// Read telemetry as soon as possible
	ESC1->readTelemetry( &TLM1 );
	ESC2->readTelemetry( &TLM2 );
	ESC3->readTelemetry( &TLM3 );
	ESC4->readTelemetry( &TLM4 );

}

void Stabilizer::setHome(){
	yawRef = angles[0]; 
  Serial.println( yawRef ); 
}


float Stabilizer::batteryVoltage( void ){

	float voltage = 0.0;

	voltage += ESC1->tlm.voltage;
	voltage += ESC2->tlm.voltage;
	voltage += ESC3->tlm.voltage;
	voltage += ESC3->tlm.voltage;

	if ( voltage != 0.0 ){
		return voltage / 4.0;
	}
	
	return 0.0;
}

void Stabilizer::getCamPosition( void ){
  
  int8_t a;
  int8_t b;

  int16_t tempRoll = 0;
  int16_t tempPitch = 0;
 
  pixy.line.getMainFeatures(LINE_ALL_FEATURES, false);

  if ( pixy.line.numVectors > 0 ){

    pixy.line.vectors->print();

    p1.y = pixy.line.vectors->m_y0; p1.x = pixy.line.vectors->m_x0; 
    p2.y = pixy.line.vectors->m_y1; p2.x = pixy.line.vectors->m_x1; 
    
    a = p1.y - p2.y;
    b = p1.x - p2.x;
    
    vectorAngle = atan2(b,a) * (180 / 3.1415);
    
    if ( p1.x > p2.x )
      tempRoll = 39 - ( (float)p1.x - 0.5 * b );
    else
      tempRoll = 39 - ( (float)p1.x + 0.5 * b );

    if ( p1.y > p2.y )
      tempPitch = 25.5 - ( (float)p1.y - 0.5 * a );
    else
      tempPitch = 25.5 - ( (float)p1.y + 0.5 * a );

    deltaPitch = EMA( tempPitch, deltaPitch, 0.2 );
    deltaRoll = EMA( tempRoll, deltaRoll, 0.2 );

    // Serial.println( pixy.line.vectors->m_index );
      
  }

}

float Stabilizer::EMA( float newSample, float oldSample, float alpha ){
  return ((alpha * newSample) + (1.0-alpha) * oldSample);  
}
