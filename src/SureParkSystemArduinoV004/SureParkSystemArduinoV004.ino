/****************************************************************
 * File: SureParkSystemArduinoV2.0
 * Project: Ajou University Summer CMU Software Engineering Program
 * Copyright: Copyright (c) 2016 DH1002 - "Cheony"
 * versions:
 * 1.0 July 20, 2016 - Initial version
 * 2.0 July 22, 2016 - Interim version
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
 * which are Stall1SensorPin, 2, 3, and 4. It calls recvFromServer()
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

#include <Servo.h> 
#include <Thread.h>
#include <ThreadController.h>
#include <SPI.h>
#include <WiFi.h>

// socket define
#define PORTID 550                 // IP socket port ID
char ssid[] = "ASUS_Guest2";           // The network SSID for CMU unsecure network
char c;                               // Character read from server
int status = WL_IDLE_STATUS;         // Network connection status
IPAddress server(192,168,1,23);    // The server's IP address
WiFiClient client;                  // The client (our) socket
IPAddress ip;                     // The IP address of the shield
IPAddress subnet;                 // The IP address of the shield
long rssi;                         // Wifi shield signal strength
byte mac[6];                       // Wifi shield MAC address

// GateSerrvo define
#define EntryGateServoPin 5
#define ExitGateServoPin 6
#define Open  90
#define Close 0
int delayValue = 500;
Servo EntryGateServo;
Servo ExitGateServo;

// ParkingStallSensor define
#define Stall1SensorPin 30
#define Stall2SensorPin 31
#define Stall3SensorPin 32
#define Stall4SensorPin 33
long  Stall1SensorVal;
long  Stall2SensorVal;
long  Stall3SensorVal;
long  Stall4SensorVal;
long  StallSVal = 25;

// LED test define
#define EntryGateGreenLED 26
#define EntryGateRedLED   27
#define ExitGateGreenLED  28
#define ExitGateRedLED    29
#define ParkingStall1LED  22
#define ParkingStall2LED  23
#define ParkingStall3LED  24
#define ParkingStall4LED  25

// EntryExitBeam define
#define EntryBeamRcvr  34 
#define ExitBeamRcvr   35
int EntryBeamState;
int ExitBeamState;

// entry status chk boolean, false -> no detection of any driver, true -> driver is parking or exiting
bool entryGateB = false;
bool exitGateB = false;

// Parking status chk boolean, false -> unoccupied space, true -> occupied space
bool space1 = false;
bool space2 = false;
bool space3 = false;
bool space4 = false;

/* Arduino send data which are whether driver is infront of entry gate or not and on the spot or not to server */
void sendToServer(){
  EntryBeamState = digitalRead(EntryBeamRcvr);  // Here we read the state of the entry beam.
  ExitBeamState = digitalRead(ExitBeamRcvr);  // Here we read the state of the exit beam.

  // Stall sensor data
  Stall1SensorVal = ProximityVal(Stall1SensorPin); //Check parking space 1
  Stall2SensorVal = ProximityVal(Stall2SensorPin); //Check parking space 2
  Stall3SensorVal = ProximityVal(Stall3SensorPin); //Check parking space 3
  Stall4SensorVal =  ProximityVal(Stall4SensorPin); //Check parking space 4

  // send protocol of whether driver is in front of enrty gate or not
  if (EntryBeamState == LOW && !entryGateB) { // if EntryBeamState is LOW the beam is broken
    Serial.println("Driver is in front of Entry gate.");
    client.println("1");
    // turn on the entry gate LED -> YELLOW for ready sign to enter the gate
    digitalWrite(EntryGateRedLED, LOW);
    digitalWrite(EntryGateGreenLED, LOW);
    recvFromServer();
  } else if (EntryBeamState == HIGH) { // Driver enter the parking lot or nobody is in front of entry gate
    EntryGateServo.write(Close);
    // turn on the exit gate LED -> RED
    digitalWrite(EntryGateRedLED, LOW);
    digitalWrite(EntryGateGreenLED, HIGH);
    delay(delayValue);
    entryGateB = false;
  }

  // send protocol of whether driver is in front of exit gate or not
  if (ExitBeamState == LOW && !exitGateB) { // if ExitBeamState is LOW the beam is broken
    Serial.println("Driver is in front of Exit gate.");
    client.println("2");
    // turn on the exit gate LED -> YELLOW for ready sign to leave the exit gate
    digitalWrite(ExitGateRedLED, LOW);
    digitalWrite(ExitGateGreenLED, LOW);
    recvFromServer();
  } else if (ExitBeamState == HIGH) { // Driver out or noboday is in front of exit gate
    ExitGateServo.write(Close);
    // turn on the exit gate LED -> RED
    digitalWrite(ExitGateRedLED, LOW);
    digitalWrite(ExitGateGreenLED, HIGH);
    delay(delayValue);
    exitGateB = false;
  }

  // Checking thet whether the driver parked righit space or not
  if (Stall1SensorVal < StallSVal && !entryGateB) { // PARKING SPACE 1
    if (!space1) {
      space1 = true;
      client.println("a");
      Serial.println("The driver parked on the parking space 1.");
      turnOffParkingLEDs();
    }
  } else if (Stall1SensorVal >= StallSVal) {
    if (space1) {
      space1 = false;
      Serial.println("The driver leaved on the parking space 1.");
      client.println("e");
    }
  }
  if (Stall2SensorVal < StallSVal && !entryGateB) { // PARKING SPACE 2
    if (!space2) {
      space2 = true;
      client.println("b");
      Serial.println("The driver parked on the parking space 2.");
      turnOffParkingLEDs();
    }
  } else if (Stall2SensorVal >= StallSVal) {
    if (space2) {
      space2 = false;
      Serial.println("The driver leaved on the parking space 2.");
      client.println("f");
    }
  }
  if (Stall3SensorVal < StallSVal && !entryGateB) { // PARKING SPACE 3
    if (!space3) {
      space3 = true;
      client.println("c");
      Serial.println("The driver parked on the parking space 3.");
      turnOffParkingLEDs();
    }
  } else if (Stall3SensorVal >= StallSVal) {
    if (space3) {
      space3 = false;
      Serial.println("The driver leaved on the parking space 3.");
      client.println("g");
    }
  }
  if (Stall4SensorVal < StallSVal && !entryGateB) { // PARKING SPACE 4
    if (!space4) {
      space4 = true;
      client.println("d");
      Serial.println("The driver parked on the parking space 4.");
      turnOffParkingLEDs();
    }
  } else if (Stall4SensorVal >= StallSVal) {
    if (space4) {
      space4 = false;
      Serial.println("The driver leaved on the parking space 4.");
      client.println("h");
    }
  }
}

/* Arduino receive data which are whether the gate open or not and which spot LED is on or not from server */
void recvFromServer(){
  char tmp = ' ';
  char c = ' ';
  Serial.print("Local server send message : ");
  while (tmp != '\n') {
    if (client.available()) {
      tmp = client.read();
      if (('1' <= tmp && tmp <= '7') || ('a' <= tmp && tmp <= 'd')) {
        c = tmp;
      }
      Serial.write(tmp);
    }
  }

  Serial.print("C = ");
  Serial.println(c);

  // receiving date abot entry gate and parking space LED
  if ((c == '3') && (EntryBeamState == LOW)) { // recv the msg which is '3' from server - open the entry gate and turn on the First LED
    Serial.println("enter 3 and entry");
    entryGateB = true;
    EntryGateServo.write(Open);
    delay(delayValue);
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    delay(delayValue);
    digitalWrite(ParkingStall1LED, HIGH);
    delay(delayValue);
  } else if ((c == '4') && (EntryBeamState == LOW)) { // recv the msg which is '4' from server - open the entry gate and turn on the Second LED
    Serial.println("enter 4 and entry");
    entryGateB = true;
    EntryGateServo.write(Open);
    delay(delayValue);
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    delay(delayValue);
    digitalWrite(ParkingStall2LED, HIGH);
    delay(delayValue);
  } else if ((c == '5') && (EntryBeamState == LOW)) { // recv the msg which is '5' from server - open the entry gate and turn on the Third LED
    Serial.println("enter 5 and entry");
    entryGateB = true;
    EntryGateServo.write(Open);
    delay(delayValue);
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    delay(delayValue);
    digitalWrite(ParkingStall3LED, HIGH);
    delay(delayValue);
  } else if ((c == '6') && (EntryBeamState == LOW)) { // recv the msg which is '6' from server - open the entry gate and turn on the Fourth LED
    Serial.println("enter 6 and entry");
    entryGateB = true;
    EntryGateServo.write(Open);
    delay(delayValue);
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    delay(delayValue);
    digitalWrite(ParkingStall4LED, HIGH);
    delay(delayValue);
  }

  // receiving date abot exit gate
  if ((c == '7') && (ExitBeamState == LOW)) { // rect the msg which is '7' from server - open the exit gate
    Serial.println("The driver is exiting....");
    exitGateB = true;
    ExitGateServo.write(Open);
    delay(delayValue);
    // turn on the exit gate LED -> GREEN
    digitalWrite(ExitGateRedLED, HIGH);
    digitalWrite(ExitGateGreenLED, LOW);
    delay(delayValue);
  }
}

void setup(){
  Serial.begin(9600);
  
  Serial.println("Attempting to connect to network...");
  Serial.print("SSID: ");
  Serial.println(ssid);
  // Attempt to connect to Wifi network.
  while ( status != WL_CONNECTED) 
  { 
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
  }else{
    // need to delay ??
  }
  
  InitEntryExitLEDs();   // You have to do this to turn off the entry LEDs

  pinMode(EntryBeamRcvr, INPUT);     // Make entry IR rcvr an input
  digitalWrite(EntryBeamRcvr, HIGH); // enable the built-in pullup

  pinMode(ExitBeamRcvr, INPUT);      // Make exit IR rcvr an input
  digitalWrite(ExitBeamRcvr, HIGH);  // enable the built-in pullup

  // LED initiation
  pinMode(EntryGateGreenLED, OUTPUT);    // This section makes all the LED pins outputs.
  pinMode(EntryGateRedLED, OUTPUT);
  pinMode(ExitGateGreenLED, OUTPUT);
  pinMode(ExitGateRedLED, OUTPUT);
  pinMode(ParkingStall1LED, OUTPUT);
  pinMode(ParkingStall2LED, OUTPUT);
  pinMode(ParkingStall3LED, OUTPUT);
  pinMode(ParkingStall4LED, OUTPUT);
  digitalWrite(EntryGateGreenLED, HIGH);  // The gate LEDs are turned off by setting their pins
  digitalWrite(EntryGateRedLED, LOW);    // high. The reason for this is that they are
  digitalWrite(ExitGateGreenLED, HIGH);   // 3 color LEDs with a common annode (+). So setting
  digitalWrite(ExitGateRedLED, LOW);     // any of the other 3 legs low turns on the LED.
  digitalWrite(ParkingStall1LED, LOW);    // Standard LEDs are used for the parking stall
  digitalWrite(ParkingStall2LED, LOW);    // LEDs. Set the pin high and they light.
  digitalWrite(ParkingStall3LED, LOW);
  digitalWrite(ParkingStall4LED, LOW);

  // Map servo to pin(gatelifttest)
  EntryGateServo.attach(EntryGateServoPin);
  ExitGateServo.attach(ExitGateServoPin);

  // Gate close initiation
  Serial.println( "Close Both Gates" );  //Here we close both gates
  EntryGateServo.write(Close); 
  ExitGateServo.write(Close);  
//  delay( delayValue );

  // Stall sensor data
  Stall1SensorVal = ProximityVal(Stall1SensorPin); //Check parking space 1
  Stall2SensorVal = ProximityVal(Stall2SensorPin); //Check parking space 2
  Stall3SensorVal = ProximityVal(Stall3SensorPin); //Check parking space 3
  Stall4SensorVal =  ProximityVal(Stall4SensorPin); //Check parking space 4
}

void loop(){
  sendToServer();
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

/*********************************************************************
* void InitEntryExitLEDs()
*
* Parameters: None           
* 
* Description:
* The entry and exit LEDs are 3 way LEDs with a common annode. This means
* that you pull the other legs low to lite the appropriate colored LED.
* The problem is that when you turn on the CPU, the pins are typically low
* meaning that the LEDs will be on. This method, simply ensures they are 
* off.
***********************************************************************/    
void InitEntryExitLEDs()
{
  int i;
  for (i=26; i<=29; i++){
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
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

/* Turn off the parking LEDs */
void turnOffParkingLEDs()
{
  digitalWrite(ParkingStall1LED, LOW);
  digitalWrite(ParkingStall2LED, LOW);
  digitalWrite(ParkingStall3LED, LOW);
  digitalWrite(ParkingStall4LED, LOW);
}
