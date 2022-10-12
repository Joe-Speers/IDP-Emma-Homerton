// details of wifi hotspot
#define SSID "IDP_robot_wifi"
#define PASSWORD "group108"
#define PORT 25586

class WifiDebug{
    public:
        void SetupHotspot(); //Sets up the Wifi hotspot
        void SendMessage(String msg); //sends a message to the PC
        String ReadCommand(); //checks for messages from the pc and returns one if avaliable. returns "" if none availiable
};