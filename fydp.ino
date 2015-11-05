/*
 * FYDP Arduino-ROS Bridge
 * Actuates motor depending on ROS commands
 */

#include <ros.h>
#include <std_msgs/String.h>

// INITIALIZE VARIABLES
ros::NodeHandle nh;
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);

bool leftOn = false;
bool rightOn = false;

int left = 9;
int right = 10;

// Log movements
void logmove(direction,on) {
  if(direction == left) {
    debug_msg.data = "LEFT";
  }
  if (direction == right) {
    debug_msg.data = "RIGHT";
  }

  if (on) {
    debug_msg.data += " | ON ";
  } else {
    debug_msg.data += " | OFF"
  }
}

// Activate motor depending on direction
void move(dir,speed) {
  bool on = digitalRead(dir);
  logmove(dir,on);

  if (on) {
    digitalWrite(dir, LOW);
  } else {
    digitalWrite(dir, HIGH);
  }
}

// Callback function that is executed when a command is published
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

ros::Subscriber<std_msgs::String> sub("command", &messageCb );

void setup()
{
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
