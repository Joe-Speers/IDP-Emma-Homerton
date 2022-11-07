/*
WifiDebug sets up a Wifi Hotspot and provides an interface to send messages back and forth from a PC.
A PC can connect to the hotspot and then use PuTTY in raw mode, connecting to IP 192.168.4.1 with the port 'PORT'.
Alternitivly pc_debug.py contains a GUI to communicate and has graphing capability.
*/

#pragma once

// details of wifi hotspot
#define SSID "steve"
#define PASSWORD "groupM108"
#define PORT 25586

class WifiDebug{
    public:
        void SetupHotspot(); //Sets up the Wifi hotspot
        void SendMessage(String msg); //sends a message to the PC
        String ReadCommand(); //checks for messages from the pc and returns one if avaliable. returns "" if none availiable
    private:
        bool newLine=false; //set to true when currentLine needs to be reset to ""
        String currentLine = ""; //stores a fragment of a message untill it has been fully recieved
};