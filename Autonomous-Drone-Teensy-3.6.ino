#include <Wire.h>
#include "DSHOT.h"
#include "Settings.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Stabilizer.h"

// Initialise object for IMU
Stabilizer controller;

Settings config;

IntervalTimer speedTimer;

int speedVal = 0;
int tempVal = 0;

#define TLM1 Serial2
#define TLM2 Serial3
#define TLM3 Serial4
#define TLM4 Serial5

#define WIFI Serial6

DShot ESC1(1);
DShot ESC2(2);
DShot ESC3(3);
DShot ESC4(4);



struct Telemetry{
  float temp; 
  float voltage;
  float amps;
  float ampHours;
  float rpm;
} telemetry[4];

uint32_t lastTlm = 0;
uint32_t lastThrottle = 0;
uint8_t tlmNumber = 0;

uint8_t imuIntPin = 2;
volatile bool newAngles = false;
volatile uint32_t lastSpeed = 0;

int16_t speedVal1 = 0, speedVal2 = 0, speedVal3 = 0, speedVal4 = 0;
int16_t speedOff1 = 8, speedOff2 = 0, speedOff3 = 9, speedOff4 = 3;

void dmpDataReady(){
  newAngles = true;
}

void setup() {

  Serial.begin(115200);
  WIFI.begin(115200);


  // Begin stablizer and IMU

  controller.setup();

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
  
  delay(1000);
}

char buffer[100] = {'\0'};
uint8_t dataCount = 0;

void loop() {

  // New angle is ready, and there is at least 2900us to next dShot 
  if( newAngles && (micros()- lastSpeed <= 100) ) {

    dataCount++;  
    controller.motorMixing( speedVal1, speedVal2, speedVal3, speedVal4 );

    if( dataCount >= 5 ){

      sendTelemetry("ANGLE");

      sendTelemetry("SPEED");
      
      dataCount = 0;
    }  
    newAngles = false;
  } 

  if( WIFI.available() > 0 ){

    WIFI.readBytesUntil('\n', buffer, 100);

    if( strstr( buffer, "ROLL" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 5; // Remove the command from the string we operate on

      double kp = 0.0, ki = 0.0, kd = 0.0;
      
      valueFromString( buffer, "P", &kp );
      valueFromString( buffer, "I", &ki );
      valueFromString( buffer, "D", &kd );

      controller.Roll.setConstants( kp, ki, kd );
      
    }

    if( strstr( buffer, "PITCH" ) != NULL ){
      
      char * ptr = buffer; 
      ptr += 6; // Remove the command from the string we operate on

      double kp = 0.0, ki = 0.0, kd = 0.0;
      
      valueFromString( buffer, "P", &kp );
      valueFromString( buffer, "I", &ki );
      valueFromString( buffer, "D", &kd );

      controller.Pitch.setConstants( kp, ki, kd );
      
    }
    
  }

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

  readTelemetry( tlmNumber );

}

void loadConfig(){
  
  config.load();

  // Set all PID constants from config (EEPROM)
  controller.Yaw.setConstants( config.yawKp, config.yawKi, config.yawKd );
  controller.Roll.setConstants( config.rollKp, config.rollKi, config.rollKd );
  controller.Pitch.setConstants( config.pitchKp, config.pitchKi, config.pitchKd );
  
}

void interruptThrottle(){
 
  lastSpeed = micros();
  // manuelThrottle( speedVal1, speedVal2, speedVal3, speedVal4 );
  manuelThrottle( speedVal1 + speedOff1, speedVal2 + speedOff2, speedVal3 + speedOff3, speedVal4 + speedOff4 );
  
}

void manuelThrottle( int16_t s1, int16_t s2, int16_t s3, int16_t s4 ){
 
  ESC1.setThrottle( s1 );
  ESC2.setThrottle( s2 );
  ESC3.setThrottle( s3 );
  ESC4.setThrottle( s4 );

}



bool readTelemetry( uint8_t index ){

  Stream * port;

  switch( index ){
    case 0: port = &TLM1; break;
    case 1: port = &TLM2; break;
    case 2: port = &TLM3; break;
    case 3: port = &TLM4; break;
  }

  uint8_t buf[10];
  uint8_t num = 0;
  bool tlmDone = false;
  uint32_t lastTime = 0;

  if( port->available() > 0 ){

    lastTime = micros();

    while( tlmDone == false && (micros() - lastTime) < 500 ) {

      if( port->available() ){
        buf[num++] = port->read(); 
        lastTime = micros();
      }
      
      if( num == 10 )
        tlmDone = true; 
      
    }
  }

  if( tlmDone == true ){
    
    telemetry[index].temp       = (float)(buf[0]);
    telemetry[index].voltage    = (float)((buf[1]<<8)|buf[2]) / 100.0;
    telemetry[index].amps       = (float)((buf[3]<<8)|buf[4]) / 100.0;
    telemetry[index].ampHours   = (float)((buf[5]<<8)|buf[6]);
    telemetry[index].rpm        = (float)((buf[7]<<8)|buf[8]) * 100.0 / 7.0; 

    Serial.print(index);
    Serial.print(":");
    Serial.println( telemetry[index].rpm );

    return true;

  }

  return false;

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

void sendTelemetry( const char * type ){

  char tempBuffer[20] = {'\0'};
  memset(buffer,0,sizeof(buffer));
  
  if( strcmp( type, "ANGLE" ) == 0 ) {
    
    strcpy(buffer, type);
  
    // Build telemetry buffer
    sprintf(tempBuffer, "%.2f", controller.angles[0]);
    strcat(buffer, " Y");
    strcat(buffer, tempBuffer);
  
    sprintf(tempBuffer, "%.2f", controller.angles[2]);
    strcat(buffer, " P");
    strcat(buffer, tempBuffer);
    
    sprintf(tempBuffer, "%.2f", controller.angles[1]);
    strcat(buffer, " R");
    strcat(buffer, tempBuffer);
  
    WIFI.println(buffer);
  }

  else if( strcmp( type, "SPEED" ) == 0 ) {
  
    strcpy(buffer, type);
  
    // Build telemetry buffer
    sprintf(tempBuffer, "%d", speedVal1);
    strcat(buffer, " A");
    strcat(buffer, tempBuffer);
  
    sprintf(tempBuffer, "%d", speedVal2);
    strcat(buffer, " B");
    strcat(buffer, tempBuffer);
    
    sprintf(tempBuffer, "%d", speedVal3);
    strcat(buffer, " C");
    strcat(buffer, tempBuffer);

    sprintf(tempBuffer, "%d", speedVal4);
    strcat(buffer, " D");
    strcat(buffer, tempBuffer);

    Serial.println( buffer );
  
    WIFI.println(buffer);
  }

  
} 
