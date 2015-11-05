/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
bool leftOn = false;
bool rightOn = false;
int left = 9;
int right = 10;
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);

void messageCb( const std_msgs::String& toggle_msg){
  String command = toggle_msg.data;

  //Check if motors are currently on
  leftOn = digitalRead(left);
  rightOn = digitalRead(right);

  //Log if motors are on
  if (leftOn) {
    debug_msg.data = "LEFT ON";
    debug.publish( &debug_msg);
  }
  if (rightOn) {
    debug_msg.data = "RIGHT ON";
    debug.publish( &debug_msg);
  }

  if(command == "left") {
    if(leftOn) {
      digitalWrite(left, LOW);
      debug_msg.data = "STOP LEFT";
      debug.publish( &debug_msg);
    } else {
      digitalWrite(left, HIGH);
      debug_msg.data = "START LEFT";
      debug.publish( &debug_msg);
    }
  }

  else if(command =="right") {
    if (rightOn) {
      digitalWrite(right, 0);
      debug_msg.data = "STOP RIGHT";
      debug.publish( &debug_msg);
    } else {
      digitalWrite(right, 100);
      debug_msg.data = "START RIGHT";
      debug.publish( &debug_msg);
    }
  }

}

ros::Subscriber<std_msgs::String> sub("toggle_led", &messageCb );


void setup()
{
  Serial.begin(57600);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
