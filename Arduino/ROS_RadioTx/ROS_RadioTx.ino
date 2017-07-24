//ROS_RadioTx.ino

#include <ros.h>
#include <tugboat_control/Thrust.h>
#include <std_msgs/Int16.h>


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(8,7);
const uint64_t txPipe = 0xF0F0F0F0E1LL;

#define OK_LED A2
#define NOT_OK_LED A3

void transmitThrust(const tugboat_control::Thrust& cmd)
{
    radio.stopListening();
    bool ok = radio.write( &cmd, sizeof(cmd) );
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

ros::NodeHandle  nh;
tugboat_control::Thrust cmd;
ros::Subscriber<tugboat_control::Thrust> sub_thrust(
  "thrust", &transmitThrust);
std_msgs::Int16 percentloss;
float success = 0;
float fail = 0;
ros::Publisher pub_com("messageLoss", &percentloss);

void OK(bool isOK){
  if(isOK){
    digitalWrite(OK_LED, HIGH);
    digitalWrite(NOT_OK_LED, LOW);
    success += 1.0;
  }
  else {
    digitalWrite(OK_LED, LOW);
    digitalWrite(NOT_OK_LED, HIGH);
    fail += 1.0;
  }    
}

void setup()
{
  //Serial.begin(57600);
  
  nh.initNode();
  nh.subscribe(sub_thrust);
  nh.advertise(pub_com);
  
  pinMode(OK_LED, OUTPUT);
  pinMode(NOT_OK_LED, OUTPUT);
  OK(false);
  
  radio.begin();
  radio.setRetries(15,15);
  radio.openWritingPipe(txPipe);
  radio.startListening();
  percentloss.data = 0;
  OK(true);
  
}

void loop()
{
  percentloss.data = (int long)(1000.0*fail /(fail 
    + success));
  pub_com.publish(&percentloss);
  nh.spinOnce();
  delay(10);
}