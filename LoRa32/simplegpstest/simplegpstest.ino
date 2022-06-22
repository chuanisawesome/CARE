//#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//SoftwareSerial ss(4, 3); // GPS Module’s TX to D4 & RX to D3

// The TinyGPS++ object
TinyGPSPlus gps;

#define RXD2 16
#define TXD2 17

void setup(){
//  Serial2.begin(9600, SERIAL_8N1,2,17);
  Serial2.begin(9600, SERIAL_8N1,RXD2,TXD2);
  Serial.begin(115200);

  delay(500);
  
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  
//  ss.begin(9600);
}


void loop(){
  
  while (Serial2.available() > 0){
    byte gpsData = Serial2.read();
    Serial.write(gpsData);
    }
}
