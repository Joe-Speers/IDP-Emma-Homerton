// details of wifi hotspot
#define SSID "steve"
#define PASSWORD "groupM108"
#define PORT 25586

class WifiDebug{
    public:
        void SetupHotspot(); //Sets up the Wifi hotspot
        void SendMessage(String msg); //sends a message to the PC
        String ReadCommand(); //checks for messages from the pc and returns one if avaliable. returns "" if none availiable
};