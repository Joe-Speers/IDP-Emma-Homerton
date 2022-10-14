/*
Wifi Debug code. Sets up a wifi hotspot (details in WifiDebug.h). Connects to the python program 'pc_debug.py'
Useful for sending text and data values back and forth from a pc.
Note: WiFi is slow! I have only been able to send data up to 20 times per second reliably.
*/
#include <WiFiNINA.h>
#include <String.h>

#include "include/WifiDebug.h"
#include "include/util.h"

//WiFiNINA objects
WiFiServer server(PORT);
WiFiClient client;


void WifiDebug::SetupHotspot() {
  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(SSID);
  WiFi.beginAP(SSID, PASSWORD);
  // start the web server on port 'PORT'
  server.begin();
  Serial.println("created wifi hotspot");
}

void WifiDebug::SendMessage(String msg){
  //sends a message to the PC
  if (client && client.connected()){
    client.println(msg);
  }
}

String WifiDebug::ReadCommand() { //checks for messages from the pc and returns one if avaliable.
  if(newLine){//resets currentLine if true
    currentLine="";
    newLine=false;
  }
  //checks if message avaliable
  if(!server.available()) return "";
  client = server.available(); 
  if (client.connected() && client.available()) { 
    while (client.available()) {//keep reading while data is avaliable
      char c = client.read();//reads 1 character at a time          
      if(c!='\n' && c!='\r'){ // if newline character encountered, ignore
        currentLine+=c; // add to message fragment
      }
      if (c == '\n') { // once end of message found, return the message
        if(currentLine=="test"){//if PC is testing connection, return OK
          SendMessage("OK");
          newLine=true;
          return "";
        }
        //otherwise return command
        newLine=true;//resets currentLine next time this function is called
        return currentLine;
      }
    }
  } 
  return "";//otherwise return nothing
}
