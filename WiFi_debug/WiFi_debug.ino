#include <WiFiNINA.h>
int LINE_SENSOR_PIN = A0;

char ssid[] = "IDP_robot_wifi";        // your network SSID (name)
char pass[] = "group108";   

WiFiServer server(25586);
String currentLine = ""; 
int diff;

void setup() {
  pinMode(LINE_SENSOR_PIN,INPUT);
  Serial.begin(57600);
  Serial.println("Access Point Web Server");
  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);
  int status = WiFi.beginAP(ssid, pass);
  Serial.println("created wifi hotspot");
  // start the web server on port 25586
  server.begin();
}


void loop() {
  diff = analogRead(LINE_SENSOR_PIN);
  WiFiClient client = server.available(); 
  if (client and client.connected()) { 
    if (client.available()) {             
      char c = client.read();             
      if(c!='\n' and c!='\r'){
        currentLine+=c;
      }
      if (c == '\n') {
        String reply=wifi_command(currentLine);              
        client.println(reply);
        currentLine = ""; 
      }
    }
  } 
}

String wifi_command(String command){
  if(command=="test"){
    return "OK";
  }
  if(command=="C"){
    return String(diff);
  }
}