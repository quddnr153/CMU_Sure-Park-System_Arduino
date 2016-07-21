#include <Servo.h> 
#include <Thread.h>
#include <ThreadController.h>

// ThreadController that will controll all threads which are sendThread and recvThread
ThreadController controll = ThreadController();

// Send Thread (as a pointer)
Thread* sendThread = new Thread();
// Receive Thread (not pointer)
Thread recvThread = Thread();
// Test
Thread testT = Thread();




#define EntryGateServoPin 5
#define ExitGateServoPin 6
#define Open  90
#define Close 0

int delayValue = 100;
Servo EntryGateServo;
Servo ExitGateServo;
/*  GateLiftTest define 값  */


/* ParkingStallSensor define 값 */
#define Stall1SensorPin 30
#define Stall2SensorPin 31
#define Stall3SensorPin 32
#define Stall4SensorPin 33

long  Stall1SensorVal;
long  Stall2SensorVal;
long  Stall3SensorVal;
long  Stall4SensorVal;
/* ParkingStallSensor define 값 */

/* LED test define 값*/
#define EntryGateGreenLED 26
#define EntryGateRedLED   27
#define ExitGateGreenLED  28
#define ExitGateRedLED    29
#define ParkingStall1LED  22
#define ParkingStall2LED  23
#define ParkingStall3LED  24
#define ParkingStall4LED  25
/* LED test define 값*/

/* EntryExitBeam define 값 */
#define EntryBeamRcvr  34 
#define ExitBeamRcvr   35

int EntryBeamState;
int ExitBeamState;
/* EntryExitBeam define 값 */


// Arduino send data which are whether driver is infront of entry gate or not and on the spot or not to server
void sendToServer(){
  Serial.println("---------------------------------------------");
  Serial.print("Send data to Server: ");
  ExitBeamState = digitalRead(ExitBeamRcvr);  // Here we read the state of the exit beam.

  if (ExitBeamState == LOW) { // if ExitBeamState is LOW the beam is broken     
    Serial.println("Exit beam broken - Someone comes entry gate.");
    /* 빨간등 끄고 파란불 킴 */
    digitalWrite(ExitGateRedLED, HIGH);
    Serial.println( "Turn on entry green LED" );
    digitalWrite(ExitGateGreenLED, LOW);
    
    /* exit gete 오픈*/
    Serial.println( "Open Exit Gate" );    //Here we open the exit gate
    ExitGateServo.write(Open);
//  delay( delayValue );
  } else {
    Serial.println("Exit beam is not broken. - Nobody is in front on entry gate");
    /* exit gete 클로즈*/
    Serial.println( "Close Exit Gate" );   //Here we close the exit gate
    ExitGateServo.write(Close);
    /* 파란불 끄고 빨간등 킴 */
    digitalWrite(ExitGateGreenLED, HIGH);
    Serial.println( "Turn on entry green LED" );
    digitalWrite(ExitGateRedLED, LOW);  
  }
  Serial.println("---------------------------------------------");
  Serial.println();
}

// Arduino receive data which are whether the gate open or not and which spot LED is on or not from server
void recvFromServer(){
  Serial.println("---------------------------------------------");
  Serial.print("Receive data from Server: ");
  Serial.println(millis());
  
  EntryBeamState = digitalRead(EntryBeamRcvr);  // Here we read the state of the entry beam.

  if (EntryBeamState == LOW) { // if EntryBeamState is LOW the beam is broken
    Serial.println("Entry beam broken");
    /* 빨간등 끄고 파란불 킴 */
    digitalWrite(EntryGateRedLED, HIGH);
    Serial.println( "Turn on entry green LED" );
    digitalWrite(EntryGateGreenLED, LOW);

    /* entry gete 오픈*/
    Serial.println( "Open Entry Gate" );   //Here we open the entry gate
    EntryGateServo.write(Open);
//  delay( delayValue );
    
    /* 1번 센서 켬*/
    Serial.println( "Turn on stall 1 LED" );
    digitalWrite(ParkingStall1LED, HIGH);
  } else {
    Serial.println("Entry beam is not broken.");
    /* entry gete 클로즈*/
    Serial.println( "Close Entry Gate" );  //Here we close the entry gate
    EntryGateServo.write(Close);
    
    /* 파란불 끄고 빨간등 킴 */
    digitalWrite(EntryGateGreenLED, HIGH);
    digitalWrite(EntryGateRedLED, LOW);
  }
}

// callback for recvThread
void boringCallback(){
  if(Stall1SensorVal<20) {
    Serial.println( "Turn off stall 1 LED" );
    digitalWrite(ParkingStall1LED, LOW);
    }
}


void setup(){
  InitEntryExitLEDs();   // You have to do this to turn off the entry LEDs

  pinMode(EntryBeamRcvr, INPUT);     // Make entry IR rcvr an input
  digitalWrite(EntryBeamRcvr, HIGH); // enable the built-in pullup

  pinMode(ExitBeamRcvr, INPUT);      // Make exit IR rcvr an input
  digitalWrite(ExitBeamRcvr, HIGH);  // enable the built-in pullup

  /* LED 값 초기화*/
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
  /* LED 값 초기화*/

  // Map servo to pin(gatelifttest)
  EntryGateServo.attach(EntryGateServoPin);
  ExitGateServo.attach(ExitGateServoPin);

  Serial.begin(9600);

  // Configure sendThread
  sendThread->onRun(recvFromServer);
  sendThread->setInterval(100);

  // Configure sendThread
  recvThread.onRun(boringCallback);
  recvThread.setInterval(100);

  // test t
  testT.onRun(sendToServer);
  testT.setInterval(100);

  // Adds both threads to the controller
  controll.add(sendThread);
  controll.add(&recvThread); // & to pass the pointer to it
  controll.add(&testT);


  /* gete 초기화*/
  Serial.println( "Close Both Gates" );  //Here we close both gates
  EntryGateServo.write(Close); 
  ExitGateServo.write(Close);  
//  delay( delayValue );

  /* 센서 체크*/
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

/* ParkingStallSensor에서 쓰는 함수*/
long ProximityVal(int Pin)
{
  long duration = 0;
  pinMode(Pin, OUTPUT);         // Sets pin as OUTPUT
  digitalWrite(Pin, HIGH);      // Pin HIGH
//    delay(1);                     // Wait for the capacitor to stabilize

  pinMode(Pin, INPUT);          // Sets pin as INPUT
  digitalWrite(Pin, LOW);       // Pin LOW
  while(digitalRead(Pin)){       // Count until the pin goes LOW (cap discharges)
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
