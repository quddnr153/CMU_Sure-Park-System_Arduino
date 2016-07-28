/****************************************************************
 * File: SureParkSystemArduinoV2.0
 * Project: Ajou University Summer CMU Software Engineering Program
 * Copyright: Copyright (c) 2016 DH1002 - "Cheony"
 * versions:
 * 1.0 July 20, 2016 - Initial version
 * 2.0 July 22, 2016 - Interim version
 * 3.0 July 26, 2016 - Interim version
 * 
 * Description:
 * 
 * This is a part of Sure-Park System. This program reads the sensor
 * calues and send them to Local server. It also receives the command
 * data from Local server and executes specific tasks (open the entry
 * gate and exit gate, and turn on the LED of exit or entry gate and 
 * parking space.
 * 
 * Parameters: None
 * 
 * Internal Methods:
 * 
 * void sendToServer() - Identify EntryBeamState and ExitBeamState 
 * from IR sensors which are EntryBeamRcvr and ExitBeamRcvr.
 * Identify Stall1SensorVal, 2, 3, and 4 from illumination sensors
 * which are StallSensorPins, 30, 31, 31, and 32. It calls recvFromServer()
 * function.
 * 
 * void recvFromServer() - Receive the command date from Local sercer.
 * It works a lot of tasks which are opening the gates, and turning 
 * on the LEDs.
 * 
 * void ProximityVal(int sensorIn) - returns an integer value from
 * the QTI sensor that is an indication of the amount of light be 
 * reflected into the sensor.
 *
 * void InitEntryExitLEDs() - Initializes the entry and exit LEDs
 * to ensure they are off.
 * 
 * void printConnectionStatus() - Show the status of connection
 * 
 * void turnOffParkingLEDs() - It turns off all of the LEDs of 
 * the parking spaces.
 ****************************************************************/

#include <Thread.h>
#include <ThreadController.h>
#include <SPI.h>
#include <WiFi.h>

// socket define
#define PORTID 550                 // IP socket port ID
char ssid[] = "ASUS_Guest2";           // The network SSID for CMU unsecure network
char c;                               // Character read from server
int status = WL_IDLE_STATUS;         // Network connection status
IPAddress server(192,168,1,80);    // The server's IP address
WiFiClient client;                  // The client (our) socket
IPAddress ip;                     // The IP address of the shield
IPAddress subnet;                 // The IP address of the shield
long rssi;                         // Wifi shield signal strength
byte mac[6];                       // Wifi shield MAC address

int delayValue = 500;

// ParkingStallSensor define
int StallSensorPins[4] = {30, 31, 32, 33};
long  StallSensorVals[4] = {0,0,0,0};
long  StallSVal = 25;
String stalls[4] = {"0000000006", "0000000007", "0000000008", "0000000009"};

// LED test define
#define EntryGateGreenLED 26
#define EntryGateRedLED   27
#define ExitGateGreenLED  28
#define ExitGateRedLED    29
#define ParkingStall1LED  22
#define ParkingStall2LED  23
#define ParkingStall3LED  24
#define ParkingStall4LED  25

// Parking status chk boolean, false -> unoccupied space, true -> occupied space
bool spaces[4] = {false,false,false,false};
bool preSpaces[4] = {false,false,false,false};

// device Flag 0,1 -> state update, 2 -> spot assign
String deviceFlag[] = {"0", "1", "2"};

String controllerID = "0000000001";

/*********************************************************************
* void sendToServer()
*
* Parameters: None           
* 
* Description:
* The goal of the fucntion is to send status of parking lot to Local server.
* To be specifc, status are following:
* entryID ("0000000000") means that the driver is in front of Entry gate.
* exitID  ("0000000001") means that the driver is in front of Exit gate.
* stall1  ("0000000002") means that the driver parked on the parking space 1.
* stall2  ("0000000003") means that the driver parked on the parking space 2.
* stall3  ("0000000004") means that the driver parked on the parking space 3.
* stall4  ("0000000005") means that the driver parked on the parking space 4.
***********************************************************************/
/* Arduino send data which are whether driver is infront of entry gate or not and on the spot or not to server */
void stallSensorChecking() {
  int chk = 0;

  for(int i = 0; i < 4; i++) {
    if (StallSensorVals[i] >= StallSVal) {
      spaces[i] = false;
      preSpaces[i] = false;
    }
  }
  
  // Checking thet whether the driver parked righit space or not
  while((preSpaces[0] == spaces[0]) && (preSpaces[0] == spaces[0]) && (preSpaces[0] == spaces[0]) &&(preSpaces[0] == spaces[0])){
    for (int i = 0; i < 4; i++) {
      if (StallSensorVals[i] < StallSVal) { // PARKING SPACE 1
        if (!spaces[i]) {
          spaces[i] = true;
          chk = i;
          client.println(deviceFlag[2] + "," + stalls[i]);
          client.flush();
          Serial.println("The driver parked on the parking spot " + stalls[i] + ".");
          turnOffParkingLEDs();
        }
      } 
    }
  }
  preSpaces[chk] = !preSpaces[chk];
}

/*********************************************************************
* void recvFromServer()
*
* Parameters: None           
* 
* Description:
* The goal of the fucntion is to receive commands from Local server.
* To be specifc, commands are following:
* '1' is to open the entry gate and turn on the First LED
* '2' is to open the entry gate and turn on the Second LED
* '3' is to open the entry gate and turn on the Third LED
* '4' is to open the entry gate and turn on the Fourth LED
* '5' is to open the exit gate
***********************************************************************/ 
void recvFromServer(){
  char tmp = ' ';
  char c = ' ';
  Serial.print("Local server send message : ");
  while (tmp != '\n') {
    if (client.available()) {
      tmp = client.read();
      if (('6' <= tmp) && (tmp <= '9')) {
        c = tmp;
      }
      Serial.write(tmp);
    }
  }

  Serial.print("The localserver sends the data which is ");
  Serial.println(c);

  // receiving date abot entry gate and parking space LED
  if ((c == '6')) { // recv the msg which is '3' from server - open the entry gate and turn on the First LED
    Serial.println("Turn on the spot 5 LED and open the enry gate");
    digitalWrite(ParkingStall1LED, HIGH);
    delay(delayValue);
    stallSensorChecking();
  } else if ((c == '7')) { // recv the msg which is '4' from server - open the entry gate and turn on the Second LED
    Serial.println("Turn on the spot 6 LED and open the enry gate");
    digitalWrite(ParkingStall2LED, HIGH);
    delay(delayValue);
    stallSensorChecking();
  } else if ((c == '8')) { // recv the msg which is '5' from server - open the entry gate and turn on the Third LED
    Serial.println("Turn on the spot 7 LED and open the enry gate");
    digitalWrite(ParkingStall3LED, HIGH);
    delay(delayValue);
    stallSensorChecking();
  } else if ((c == '9')) { // recv the msg which is '6' from server - open the entry gate and turn on the Fourth LED
    Serial.println("Turn on the spot 8 LED and open the enry gate");
    digitalWrite(ParkingStall4LED, HIGH);
    delay(delayValue);
    stallSensorChecking();
  }
}

void setup(){
  Serial.begin(9600);
  Serial.println("Attempting to connect to network...");
  Serial.print("SSID: ");
  Serial.println(ssid);
  // Attempt to connect to Wifi network.
  while ( status != WL_CONNECTED) { 
     Serial.print("Attempting to connect to SSID: ");
     Serial.println(ssid);
     status = WiFi.begin(ssid);
  }
  Serial.println( "Connected to network:" );
  Serial.println( "\n----------------------------------------" );
  // Print the basic connection and network information.
  printConnectionStatus();
  Serial.println( "\n----------------------------------------\n" );
  Serial.print("\nAttempting to connect to server...");
  
  if(client.connect(server, PORTID)){
    Serial.println("connected");
  }else{ // need to delay ??
  }
  
  // LED initiation
  pinMode(ParkingStall1LED, OUTPUT);
  pinMode(ParkingStall2LED, OUTPUT);
  pinMode(ParkingStall3LED, OUTPUT);
  pinMode(ParkingStall4LED, OUTPUT);
  digitalWrite(ParkingStall1LED, LOW);    // Standard LEDs are used for the parking stall
  digitalWrite(ParkingStall2LED, LOW);    // LEDs. Set the pin high and they light.
  digitalWrite(ParkingStall3LED, LOW);
  digitalWrite(ParkingStall4LED, LOW);
  
  // Stall sensor data
  stallChecking();

  // Send client register information to localServer
  registerDevices();
  Serial.println("Device register complete");
}

void loop(){
  // WiFi connection check every time
   if (client.connected()) {
    recvFromServer();
   } else {
     Serial.println();
     Serial.println("The socket connection between controller and localserver is disconnected.");
     //  Do something like open the gates
     client.stop();
     if(client.connect(server, PORTID)){
       Serial.println("The socket connection between controller and localserver is complete.");
       Serial.println("Registering the devices ....");
       registerDevices();
     }else{
       // need to delay ??
     }
   }
}

/*********************************************************************
* long ProximityVal(int Pin)
* Parameters:            
* int pin - the pin on the Arduino where the QTI sensor is connected.
*
* Description:
* QTI schematics and specs: http://www.parallax.com/product/555-27401
* This method initalizes the QTI sensor pin as output and charges the
* capacitor on the QTI. The QTI emits IR light which is reflected off 
* of any surface in front of the sensor. The amount of IR light 
* reflected back is detected by the IR resistor on the QTI. This is 
* the resistor that the capacitor discharges through. The amount of 
* time it takes to discharge determines how much light, and therefore 
* the lightness or darkness of the material in front of the QTI sensor.
* Given the closeness of the object in this application you will get
* 0 if the sensor is covered
***********************************************************************/
long ProximityVal(int Pin)
{
  long duration = 0;
  pinMode(Pin, OUTPUT);         // Sets pin as OUTPUT
  digitalWrite(Pin, HIGH);      // Pin HIGH
  delay(1);                     // Wait for the capacitor to stabilize

  pinMode(Pin, INPUT);          // Sets pin as INPUT
  digitalWrite(Pin, LOW);       // Pin LOW
  while (digitalRead(Pin)) {    // Count until the pin goes LOW (cap discharges)
    duration++;
  }
  return duration;              // Returns the duration of the pulse
}

/************************************************************************************************
* The following method prints out the connection information
************************************************************************************************/
void printConnectionStatus() 
{
  // Print the basic connection and network information: Network, IP, and Subnet mask
  ip = WiFi.localIP();
  Serial.print("Connected to ");
  Serial.print(ssid);
  Serial.print(" IP Address:: ");
  Serial.println(ip);
  subnet = WiFi.subnetMask();
  Serial.print("Netmask: ");
  Serial.println(subnet);
   
  // Print our MAC address.
  WiFi.macAddress(mac);
  Serial.print("WiFi Shield MAC address: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
   
  // Print the wireless signal strength:
  rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
} // printConnectionStatus

/*********************************************************************
* void turnOffParkingLEDs()
*
* Parameters: None           
* 
* Description:
* Turn off all parking LEDs
***********************************************************************/ 
void turnOffParkingLEDs()
{
  digitalWrite(ParkingStall1LED, LOW);
  digitalWrite(ParkingStall2LED, LOW);
  digitalWrite(ParkingStall3LED, LOW);
  digitalWrite(ParkingStall4LED, LOW);
}

void registerDevices ()
{
  String stallState[4];
  Serial.println("Registering the devices ....");
  stallChecking();
  for(int i = 0; i < 4; i++){
    if (StallSensorVals[i] < StallSVal){
      stallState[i] = "0";
    } else {
      stallState[i] = "1";
    }
  }
  client.print(controllerID +" ");
  client.flush();
  for(int i = 0; i < 4; i++) {
    client.print(stalls[i] + ",3,1," + stallState[i] + "," + controllerID + " ");
    client.flush();
  }
  client.print("\n");
  client.flush();
  delay(2000);
}

void stallChecking ()
{
  for(int i = 0; i < 4; i++){
    StallSensorVals[i] = ProximityVal(StallSensorPins[i]); //Check parking spaces
  }
}
