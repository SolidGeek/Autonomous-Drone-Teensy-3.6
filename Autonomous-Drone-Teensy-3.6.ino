#include <Wire.h>
#include "DSHOT.h"
#include "Settings.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Stabilizer.h"




// Initialise object for IMU
Stabilizer FC;

IntervalTimer speedTimer;

bool isConnected = false;
uint32_t lastAlive = 0;


int speedVal = 0;
int tempVal = 0;
int altitudeVal = 0;

#define TLM1 Serial2
#define TLM2 Serial3
#define TLM3 Serial4
#define TLM4 Serial5

#define WIFI Serial6

DShot ESC1(1);
DShot ESC2(2);
DShot ESC3(3);
DShot ESC4(4);

uint32_t lastTlm = 0;
uint32_t lastThrottle = 0;
uint8_t tlmNumber = 0;

uint8_t imuIntPin = 2;
volatile bool newAngles = false;
volatile int countDmp = 0;
volatile uint32_t lastSpeed = 0;

// Interrupt when IMU data is ready to be read
void dmpDataReady(){
  newAngles = true;
  countDmp++;
}

void setup() {

  Serial.begin(115200);
  WIFI.begin(115200);

  // Begin flight stablizer (IMU, altitude etc.)
  FC.setup();

  // Use the INTA pin on MPU6050 to know excatly when data is ready, and only here read it
  pinMode(imuIntPin, INPUT);
  attachInterrupt( digitalPinToInterrupt(imuIntPin), dmpDataReady, RISING);

  /* 
   *  Setup of telemetry UART ports
   */
  TLM1.begin(115200);
  TLM2.begin(115200);
  TLM3.begin(115200);
  TLM4.begin(115200);

  /* 
   *  Setup of motor direction 
   */
  ESC1.setDirection(false);
  ESC2.setDirection(true);
  ESC3.setDirection(false);
  ESC4.setDirection(true);

  delay(500);

  // Arm drone
  for( int i = 0; i < 500; i++ ){
    manuelThrottle(0,0,0,0);
    delayMicroseconds(500);
  }

  speedTimer.begin(interruptThrottle, 3000); // Update throttle at 333 Hz
  speedTimer.priority(0);


  altitude.setTimeout(500);
  if (!altitude.init()){
    Serial.println("No altitude sensor");
  }

  altitude.setDistanceMode(VL53L1X::Long);
  altitude.setMeasurementTimingBudget(50000); // 50000 us (50 ms) 
  // Start continuous readings at a rate of one measurement every 50 ms
  altitude.startContinuous(50);


  /* Makes sure the DMP values have settled */
  while( countDmp < 500 ){
    if (newAngles){
      FC.readDMPAngles();
      newAngles = false;  
    }
  } 

  FC.setHome();
  
  delay(1000);
}

char buffer[200] = {'\0'};
uint8_t dataCount = 0;

void loop() {

  if( millis() - lastAlive < 500 ) {
    isConnected = true;
  }else{
    isConnected = false;
    FC.motorsOn = false;
  }

  // New angle is ready, and there is at least 2900us to next dShot 
  if( newAngles && (micros()- lastSpeed <= 100) ) {

    dataCount++;

    FC.motorMixing( );

    // Send motor speed an angle at 100 Hz ( new angle at 100 Hz )
  
    sendTelemetry("ANGLE");
    sendTelemetry("SPEED");

    if( dataCount >= 5 ){

      sendTelemetry("BATTERY");
      dataCount = 0;
    }

    if( altitude.dataReady() ){
      altitudeVal = altitude.read();
      sendTelemetry("ALTITUDE");
    }
    
    newAngles = false;
  } 

  listenWiFi();

  if( millis() - lastTlm > 100 ){
    
    lastTlm = millis();

    tlmNumber++;
    if( tlmNumber > 3 ){
      tlmNumber = 0;  
    }

    switch( tlmNumber ){
      case 0:
        ESC1.requestTelemetry();  break;
      case 1: 
        ESC2.requestTelemetry(); break;
      case 2:
        ESC3.requestTelemetry();  break;
      case 3: 
        ESC4.requestTelemetry(); break;
    }
  }

  if( ESC1.readTelemetry( &TLM1 ) ){
    Serial.println( ESC1.tlm.voltage );
  }

  

}


void interruptThrottle(){
 
  lastSpeed = micros();
  
  manuelThrottle( FC.s1, FC.s2, FC.s3, FC.s4 );
  
}

void manuelThrottle( int16_t s1, int16_t s2, int16_t s3, int16_t s4 ){
 
  ESC1.setThrottle( s1 );
  ESC2.setThrottle( s2 );
  ESC3.setThrottle( s3 );
  ESC4.setThrottle( s4 );

}

bool valueFromString( char * packet, char * name, double * var ){

  char * start;
  char * end;
  size_t len; 

  char buf[20] = {'\0'};

  // Find start of parameter value
  if( start = strstr( packet, name )){

    start++; // Not interested in the param name

    // Find end of parameter value
    if( end = strpbrk(start, " \n") ){

      len = end - start;

      strncpy(buf, start, len);
      buf[len] = '\0';

      // Now convert the string in buf to a double
      *var = atof(buf);
     
      return true;

    }

  }

  return false;

}

void sendConfig() {

  memset(buffer, 0, sizeof(buffer));
  strcpy(buffer, "CONFIG");

  // Add roll config
  addFloatToBuffer( FC.config.rollKp, "P" );
  addFloatToBuffer( FC.config.rollKi, "I" );
  addFloatToBuffer( FC.config.rollKd, "D" );

  // Add pitch config
  addFloatToBuffer( FC.config.pitchKp, "P" );
  addFloatToBuffer( FC.config.pitchKi, "I" );
  addFloatToBuffer( FC.config.pitchKd, "D" );
  
  // Add yaw config
  addFloatToBuffer( FC.config.yawKp, "P" );
  addFloatToBuffer( FC.config.yawKi, "I" );
  addFloatToBuffer( FC.config.yawKd, "D" );

  // Add m
  addFloatToBuffer( FC.config.altKp, "P" );
  addFloatToBuffer( FC.config.altKi, "I" );
  addFloatToBuffer( FC.config.altKd, "D" );

  // Send also motor offsets
  addIntToBuffer( FC.config.hoverOffset, "H" );
  addIntToBuffer( FC.config.offset1, "A" );
  addIntToBuffer( FC.config.offset2, "B" );
  addIntToBuffer( FC.config.offset3, "C" );
  addIntToBuffer( FC.config.offset4, "D" );

  WIFI.println(buffer);
  
}

void addFloatToBuffer( float value , const char * name ){

  char tempBuffer[20] = {'\0'};
  
  sprintf(tempBuffer, "%.2f", value );
  strcat(buffer, " ");
  strcat(buffer, name);
  strcat(buffer, tempBuffer);  
  
}

void addIntToBuffer( int value , const char * name ){

  char tempBuffer[20] = {'\0'};
  
  sprintf(tempBuffer, "%d", value );
  strcat(buffer, " ");
  strcat(buffer, name);
  strcat(buffer, tempBuffer);  
  
}

void sendTelemetry( const char * type ){

  char tempBuffer[20] = {'\0'};
  memset(buffer, 0, sizeof(buffer));
  
  if( strcmp( type, "ANGLE" ) == 0 ) {
    
    strcpy(buffer, type);
  
    // Build telemetry buffer
    sprintf(tempBuffer, "%.2f", FC.angles[0]);
    strcat(buffer, " Y");
    strcat(buffer, tempBuffer);
  
    sprintf(tempBuffer, "%.2f", FC.angles[2]);
    strcat(buffer, " P");
    strcat(buffer, tempBuffer);
    
    sprintf(tempBuffer, "%.2f", FC.angles[1]);
    strcat(buffer, " R");
    strcat(buffer, tempBuffer);
  
    WIFI.println(buffer);
  }
  
  else if( strcmp( type, "ALTITUDE" ) == 0 ) {
    
    strcpy(buffer, type);
  
    sprintf(tempBuffer, "%d", altitudeVal);
    strcat(buffer, " A");
    strcat(buffer, tempBuffer);
  
    WIFI.println(buffer);
  }

  else if( strcmp( type, "BATTERY" ) == 0 ) {

    // Find voltage of battery from average

    float voltage = 0.0;

    /* for( uint8_t i = 0; i < 4; i++ ){
      voltage += telemetry[i].voltage; 
    }

    voltage = voltage/4.0;*/
    
    strcpy(buffer, type);
  
    sprintf(tempBuffer, "%.2f", voltage);
    strcat(buffer, " V");
    strcat(buffer, tempBuffer);
  
    WIFI.println(buffer);

  }

  else if( strcmp( type, "SPEED" ) == 0 ) {
  
    strcpy(buffer, type);
  
    // Build telemetry buffer
    sprintf(tempBuffer, "%d", FC.s1);
    strcat(buffer, " A");
    strcat(buffer, tempBuffer);
  
    sprintf(tempBuffer, "%d", FC.s2);
    strcat(buffer, " B");
    strcat(buffer, tempBuffer);
    
    sprintf(tempBuffer, "%d", FC.s3);
    strcat(buffer, " C");
    strcat(buffer, tempBuffer);

    sprintf(tempBuffer, "%d", FC.s4);
    strcat(buffer, " D");
    strcat(buffer, tempBuffer);
  
    WIFI.println(buffer);
  }
} 

void listenWiFi(){
  
  if( WIFI.available() > 0 ){

    memset(buffer, 0, sizeof(buffer));
    WIFI.readBytesUntil('\n', buffer, 100);

    // If app requests config
    if( strstr( buffer, "CONFIG" ) != NULL ){
      sendConfig();
    }

    if( strstr( buffer, "ALIVE" ) != NULL ){
      lastAlive = millis();  
    }

    if( strstr( buffer, "SAVE" ) != NULL ){
      FC.config.save();
    }
    
    if( strstr( buffer, "START" ) != NULL ){
      FC.motorsOn = true;
    }
    
    if( strstr( buffer, "STOP" ) != NULL ){
      FC.motorsOn = false;
    }

    if( strstr( buffer, "ROLL" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 5; // Remove the command from the string we operate on

      double kp = 0.0, ki = 0.0, kd = 0.0;
      
      valueFromString( ptr, "P", &kp );
      valueFromString( ptr, "I", &ki );
      valueFromString( ptr, "D", &kd );

      FC.config.rollKp = kp;
      FC.config.rollKi = ki;
      FC.config.rollKd = kd;
      FC.Roll.setConstants( kp, ki, kd );
      
    }

    if( strstr( buffer, "PITCH" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 6; // Remove the command from the string we operate on

      double kp = 0.0, ki = 0.0, kd = 0.0;
      
      valueFromString( ptr, "P", &kp );
      valueFromString( ptr, "I", &ki );
      valueFromString( ptr, "D", &kd );

      FC.config.pitchKp = kp;
      FC.config.pitchKi = ki;
      FC.config.pitchKd = kd;
      FC.Pitch.setConstants( kp, ki, kd );
      
    }

    if( strstr( buffer, "YAW" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 4; // Remove the command from the string we operate on

      double kp = 0.0, ki = 0.0, kd = 0.0;
      
      valueFromString( ptr, "P", &kp );
      valueFromString( ptr, "I", &ki );
      valueFromString( ptr, "D", &kd );

      FC.config.yawKp = kp;
      FC.config.yawKi = ki;
      FC.config.yawKd = kd;
      FC.Yaw.setConstants( kp, ki, kd );
      
    }

    if( strstr( buffer, "OFFSET" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 7; // Remove the command from the string we operate on

      double tempOffset1 = 0.0, tempOffset2 = 0.0, tempOffset3 = 0.0, tempOffset4 = 0.0, hoverOffset = 0.0;

      valueFromString( ptr, "H", &hoverOffset );
      valueFromString( ptr, "A", &tempOffset1 );
      valueFromString( ptr, "B", &tempOffset2 );
      valueFromString( ptr, "C", &tempOffset3 );
      valueFromString( ptr, "D", &tempOffset4 );

      FC.config.hoverOffset = (int16_t)hoverOffset;
      FC.config.offset1 = (int16_t)tempOffset1;
      FC.config.offset2 = (int16_t)tempOffset2;
      FC.config.offset3 = (int16_t)tempOffset3;
      FC.config.offset4 = (int16_t)tempOffset4;
      
    }
  }  
}
