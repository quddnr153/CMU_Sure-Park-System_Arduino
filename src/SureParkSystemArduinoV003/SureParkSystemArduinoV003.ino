#include <Servo.h> 
#include <Thread.h>
#include <ThreadController.h>
#include <SPI.h>
#include <WiFi.h>

// socket define
#define PORTID 550              // IP socket port ID
char ssid[] = "ASUS_Guest2";              // The network SSID for CMU unsecure network
char c;                           // Character read from server
int status = WL_IDLE_STATUS;      // Network connection status
IPAddress server(192,168,1,23);  // The server's IP address
WiFiClient client;                // The client (our) socket
IPAddress ip;                     // The IP address of the shield
IPAddress subnet;                 // The IP address of the shield
long rssi;                        // Wifi shield signal strength
byte mac[6];                      // Wifi shield MAC address

// ThreadController that will controll all threads which are sendThread and recvThread
ThreadController controll = ThreadController();

// Send Thread (as a pointer)
Thread* sendThread = new Thread();
// Receive Thread (not pointer)
Thread recvThread = Thread();

// GateSerrvo define
#define EntryGateServoPin 5
#define ExitGateServoPin 6
#define Open  90
#define Close 0
int delayValue = 100;
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

// Parking status chk boolean
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
  if (EntryBeamState == LOW) { // if EntryBeamState is LOW the beam is broken
    Serial.println("Driver is in front of Entry gate.");
    client.println("1");
    // turn on the entry gate LED -> YELLOW for ready sign to enter the gate
    digitalWrite(EntryGateRedLED, LOW);
    digitalWrite(EntryGateGreenLED, LOW);
  } else { // Driver enter the parking lot or nobody is in front of entry gate
    EntryGateServo.write(Close);
    // turn on the exit gate LED -> RED
    digitalWrite(EntryGateRedLED, LOW);
    digitalWrite(EntryGateGreenLED, HIGH);
  }

  // send protocol of whether driver is in front of exit gate or not
  if (ExitBeamState == LOW) { // if ExitBeamState is LOW the beam is broken
    Serial.println("Driver is in front of Exit gate.");
    client.println("2");
    // turn on the exit gate LED -> YELLOW for ready sign to leave the exit gate
    digitalWrite(ExitGateRedLED, LOW);
    digitalWrite(ExitGateGreenLED, LOW);
  } else { // Driver out or noboday is in front of exit gate
    ExitGateServo.write(Close);
    // turn on the exit gate LED -> RED
    digitalWrite(ExitGateRedLED, LOW);
    digitalWrite(ExitGateGreenLED, HIGH);
  }

  // Checking thet whether the driver parked righit space or not
  if (Stall1SensorVal < 100) { // PARKING SPACE 1
    if (!space1) {
      space1 = true;
      client.println("a");
    }
    turnOffParkingLEDs();
  } else {
    if (space1) space1 = false;
  }
  if (Stall2SensorVal < 100) { // PARKING SPACE 2
    if (!space2) {
      space2 = true;
      client.println("b");
    }
    turnOffParkingLEDs();
  } else {
    if (space2) space2 = false;
  }
  if (Stall3SensorVal < 100) { // PARKING SPACE 3
    if (!space3) {
      space3 = true;
      client.println("c");
    }
    turnOffParkingLEDs();
  } else {
    if (space3) space3 = false;
  }
  if (Stall4SensorVal < 100) { // PARKING SPACE 4
    if (!space4) {
      space4 = true;
      client.println("d");
    }
    turnOffParkingLEDs();
  } else {
    if (space4) space4 = false;
  }
}

/* Arduino receive data which are whether the gate open or not and which spot LED is on or not from server */
void recvFromServer(){
  char c = ' ';
  Serial.print("Local server send message : ");
  while (c != '\n') {
    if (client.available()) {
      c = client.read();
      Serial.write(c);
    }
  }

  // receiving date abot entry gate and parking space LED
  if ((c == '3') && (EntryBeamState == LOW)) { // recv the msg which is '3' from server - open the entry gate and turn on the First LED
    EntryGateServo.write(Open);
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    digitalWrite(ParkingStall1LED, HIGH);
  } else if ((c == '4') && (EntryBeamState == LOW)) { // recv the msg which is '4' from server - open the entry gate and turn on the Second LED
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    digitalWrite(ParkingStall2LED, HIGH);
  } else if ((c == '5') && (EntryBeamState == LOW)) { // recv the msg which is '5' from server - open the entry gate and turn on the Third LED
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    digitalWrite(ParkingStall3LED, HIGH);
  } else if ((c == '6') && (EntryBeamState == LOW)) { // recv the msg which is '6' from server - open the entry gate and turn on the Forth LED
    // turn on the entry gate LED -> GREEN for entering sign to enter the gate
    digitalWrite(EntryGateRedLED, HIGH);
    digitalWrite(EntryGateGreenLED, LOW);
    digitalWrite(ParkingStall4LED, HIGH);
  }

  // receiving date abot exit gate
  if ((c == '7') && (ExitBeamState == LOW)) { // rect the msg which is '7' from server - open the exit gate
    ExitGateServo.write(Open);
    // turn on the exit gate LED -> GREEN
    digitalWrite(ExitGateRedLED, HIGH);
    digitalWrite(ExitGateGreenLED, LOW);
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

  // Thread configuration
  // Configure sendThread
  sendThread->onRun(recvFromServer);
  sendThread->setInterval(100);
  
  // Configure sendThread
  recvThread.onRun(recvFromServer);
  recvThread.setInterval(100);
  
  // Adds both threads to the controller
  controll.add(sendThread);
  controll.add(&recvThread); // & to pass the pointer to it

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
  // run ThreadController
  // this will check every thread inside ThreadController,
  // if it should run. If yes, he will run it;
  controll.run();

  // Rest of code
  float h = 3.1415;
  h/=2;
}

/* ParkingStallSensor read */
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

