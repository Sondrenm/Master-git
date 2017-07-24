//ROS_RadioRx.ino

#include <ros.h>
#include <tugboat_control/PushingForce.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

ros::NodeHandle  nh;
tugboat_control::PushingForce push;
ros::Publisher pub_push("pushingForce", &push);

RF24 radio(8,7);
const uint64_t rxPipe = 0xF0F0F0F0D2LL;

#define OK_LED A2
#define NOT_OK_LED A3
#define WAIT_TIME 100 //ms

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
  nh.initNode();
  nh.advertise(pub_push);
  
  radio.begin();
  radio.setRetries(15,15); 
  radio.openReadingPipe(1,rxPipe);
  radio.startListening();
  
  pinMode(OK_LED, OUTPUT);
  pinMode(NOT_OK_LED, OUTPUT);
  OK(false);
}

void loop()
{
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
    OK(false);
  }
  else 
  { 
    bool done = false;
    tugboat_control::PushingForce got_push;
    while(!done){
      done = radio.read( &got_push, sizeof(
        tugboat_control::PushingForce) );
    }
    radio.stopListening();
    radio.startListening();
    OK(true);
    push = got_push;
    pub_push.publish( &push ); 
  } 
  nh.spinOnce();
}
