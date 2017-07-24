//TugOS.ino

#define TUG_ID 3 
// Needs to be different for each tugboat

// Might be different for each tugboat
#define leftMotorForward 5
#define leftMotorBackward 6
#define rightMotorForward 9
#define rightMotorBackward 10

#define deadband 30 
//smallest motor output allowed, to avoid damaging motor
#define maxout 255-deadband

#include <ros.h>
#include <tugboat_control/PushingForce.h>
#include <tugboat_control/Thrust.h>

tugboat_control::Thrust cmd;
tugboat_control::PushingForce push;

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(8,7);
const uint64_t rxPipe = 0xF0F0F0F0E1LL;
const uint64_t txPipe = 0xF0F0F0F0D2LL;

#define OK_LED A2
#define NOT_OK_LED A3
#define WAIT_TIME 100 //ms

#include "HX711.h" 
// HX711.DOUT - pin #A1, HX711.PD_SCK - pin #A0
HX711 scale(A1, A0);
// parameter "gain" is ommited; the default value 
//128 is used by the library
#define SCALE -200591.f //To get N

int leftMotor = 0;
int rightMotor = 0;
#define dm 20 //max change in motor output per step

//Functions

bool getNewCommand()
{ 
  radio.startListening();
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
  {
    if (millis() - started_waiting_at > WAIT_TIME)
    {
      timeout = true;
    }
  }
  
  if( timeout) 
  { 
    //OK(false);
    cmd.thrust = 0;
    cmd.ccwturn = 0;
  }
  else 
  { // New radio message
   
    bool done = false;
    tugboat_control::Thrust newcmd;
    while(!done){
      done = radio.read( &newcmd, sizeof(
        tugboat_control::Thrust) ); 
    }
      
    radio.stopListening();

    //OK(true);
    if(newcmd.ID == TUG_ID)
    {
      cmd = newcmd;
      return true;
    }
  }
  return false;
}

void adjustThrust() {
	//Mapping from +- 100% thrust/cwturn to PWM signals
  // to motor. Need to be tested
	int newLeftMotor = 1.5*cmd.thrust + cmd.ccwturn;
	int newRightMotor = 1.5*cmd.thrust - cmd.ccwturn;
  
  if(newLeftMotor > leftMotor + dm){
    leftMotor += dm;
  } else if(newLeftMotor < leftMotor - dm){
    leftMotor -= dm;
  } else {
    leftMotor = newLeftMotor;
  }
        
	if (leftMotor > maxout) {
		analogWrite(leftMotorForward, 255);
		analogWrite(leftMotorBackward, 0);  
	}  else if (leftMotor > 0) {
		analogWrite(leftMotorForward, leftMotor+ 
      deadband);
		analogWrite(leftMotorBackward, 0);  
	} else if(leftMotor < -maxout) {
		analogWrite(leftMotorForward, 0);
		analogWrite(leftMotorBackward, 255); 
	} else if(leftMotor < 0) {
		analogWrite(leftMotorForward, 0);
		analogWrite(leftMotorBackward, -leftMotor + 
      deadband);    
	} else {
		analogWrite(leftMotorForward, 0);
		analogWrite(leftMotorBackward, 0);  
	}

  if(newRightMotor > rightMotor + dm){
    rightMotor += dm;
  } else if(newRightMotor < rightMotor - dm){
    rightMotor -= dm;
  } else {
    rightMotor = newRightMotor;
  }
        
	if (rightMotor > maxout) {
		analogWrite(rightMotorForward, 255);
		analogWrite(rightMotorBackward, 0); 
	}else if (rightMotor > 0) {
		analogWrite(rightMotorForward, rightMotor + 
      deadband);
		analogWrite(rightMotorBackward, 0);    
	} else if(rightMotor < -maxout) {
		analogWrite(rightMotorForward, 0);
		analogWrite(rightMotorBackward, 255); 
	} else if(rightMotor < 0) {
		analogWrite(rightMotorForward, 0);
		analogWrite(rightMotorBackward, -rightMotor +
      deadband); 
	} else {
	  analogWrite(rightMotorForward, 0);
	  analogWrite(rightMotorBackward, 0);      
	}
}

void sendPushingForce(){
  push.force = scale.get_units(1);
  
  radio.stopListening();
  bool ok = radio.write( &push, sizeof(
    tugboat_control::PushingForce) );
  if(ok)
  {
    OK(true);
  }
  else
  {
    OK(false);
  }
  radio.startListening();
}

void OK(bool isOK){
  if(isOK){
    digitalWrite(OK_LED, HIGH);
    digitalWrite(NOT_OK_LED, LOW);
  }
  else {
    digitalWrite(OK_LED, LOW);
    digitalWrite(NOT_OK_LED, HIGH);
  }    
}

void setup() 
{
  Serial.begin(57600);
  
  pinMode(OK_LED, OUTPUT);
  pinMode(NOT_OK_LED, OUTPUT);
  OK(false);
  
  radio.begin();
  radio.setRetries(15,15);
  radio.openWritingPipe(txPipe);
  radio.openReadingPipe(1,rxPipe);
  radio.startListening();
  
  scale.set_scale(SCALE);       
  // this value is obtained by calibrating the scale
  // with known weights; see the README for details
  scale.tare();			// reset the scale to 0
  scale.power_up();
  
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  
  cmd.ID = TUG_ID;  
  cmd.thrust = 0;  
  cmd.ccwturn = 0;
  
  push.ID = TUG_ID;
  push.force = 0;
  adjustThrust(); // init with all motors still
  
  OK(true);
  
  while(!getNewCommand())
  { //Reset scale when first command is received
    scale.tare();
  }
}

void loop()
{
  unsigned long start = millis();
  if(getNewCommand()){
    sendPushingForce();
    OK(true);
  }
  else
  {
    OK(false);
  }
  adjustThrust();
  unsigned long delayTime = millis() - start;
  if(delayTime < 100){
    delay(delayTime);
  }
}

