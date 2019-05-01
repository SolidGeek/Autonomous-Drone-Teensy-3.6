#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
// Include SPIFFS filesystem
#include "FS.h"

ESP8266WebServer server(80);
WebSocketsServer websocket = WebSocketsServer(81);

const char * ssid = "Drone WiFi";
const char * password = "12345679";

uint32_t lastPackage = 0;

// Local path to save the GCode recordings
char recordPath[] = "/recording.txt";
char buffer[100] = {'\0'};

void setup()
{
  // Init Serial port for UART communication
  Serial.begin(115200);

  while (!Serial); // wait for serial port to connect
  
  initWiFi();
  initSPIFFS();
  initWebsocket();
  initWebserver();
}

void loop()
{

  websocket.loop();
  server.handleClient();
    
  if( Serial.available() > 0 ){

    // Clear buffer to get rid of old data
    memset(buffer,0,sizeof(buffer));
    
    Serial.readBytesUntil('\n', buffer, 100);
    if (WiFi.softAPgetStationNum() > 0) {
      websocket.broadcastTXT(buffer);
    }
  }
 
}

void initWebsocket( void ) {
  websocket.begin();
  websocket.onEvent(webSocketEvent);
}

void initWebserver( void ) {
  
  // Page handlers
  server.serveStatic( "/",                   SPIFFS, "/index.html" );         // Main website structure
  server.serveStatic( "/bootstrap.min.css",  SPIFFS, "/bootstrap.min.css" );  // Responsive framework for the GUI
  server.serveStatic( "/style.css",          SPIFFS, "/style.css" );          // Main website style
  server.serveStatic( "/script.js",          SPIFFS, "/script.js" );          // Javascript functionalities
  server.serveStatic( recordPath,            SPIFFS, recordPath );

  server.begin();
  
}

void initWiFi( void ) {
  if (! WiFi.softAP(ssid, password)) {
    Serial.println("WIFI FAILED");
  }
}

void initSPIFFS( void ) {
  // Begin file-system
  if ( !SPIFFS.begin() ) {
    Serial.println("FIlE FAIILED");
  }
}

// Process data being send from GUI by webSocket
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {

  // Pointer to hold reference to received payload
  char * packet;
  
  if ( type == WStype_TEXT )
  {
    lastPackage = millis();

    packet = (char *)payload;

    // Afterwards, just pass the data on to the uStepper
    Serial.println( packet );

  }
}
