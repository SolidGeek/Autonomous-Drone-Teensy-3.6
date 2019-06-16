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
uint8_t wifiChannel = 10;


const int buffer_size = 200;

// Local path to save the GCode recordings
char recordPath[] = "/recording.txt";
char buffer[buffer_size] = {'\0'};


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
    
  readUartData();
}

void readUartData() {
  if( Serial.available() > 0 ){
    // Clear buffer to get rid of old data
    memset(buffer,0,sizeof(buffer));

    // Read data coming from Teensy to buffer
    Serial.readBytesUntil('\n', buffer, buffer_size);

    // Send data over Websocket to PC
    websocket.broadcastTXT(buffer);
  }
}

// Process data being send from GUI by webSocket
void readWebsocketData(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if ( type == WStype_TEXT )
  {
    // Convert uint8_t pointer to char pointer
    char * packet = (char *)payload;

    // Send received packet over UART to Teensy
    Serial.write( packet );
    Serial.write( '\n' );
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
  // Only allow for one connection at the time
  if (! WiFi.softAP(ssid, password, wifiChannel )) {
    Serial.println("WIFI FAILED");
  }
}

void initSPIFFS( void ) {
  // Begin file-system
  if ( !SPIFFS.begin() ) {
    Serial.println("FIlE FAIILED");
  }
}
