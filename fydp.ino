/*
 * FYDP Arduino-ROS Bridge
 * Actuates motor depending on ROS commands
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>


// INITIALIZE VARIABLES
ros::NodeHandle nh;
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);

bool leftOn = false;
bool rightOn = false;

int left = 9;
int right = 10;

// Debug message
void d(String s) {
    char s_arr[100];
    s.toCharArray(s_arr,100);
    debug_msg.data = s_arr;
    debug.publish( &debug_msg);
}

// Log movements
void logmove(int dir, bool on) {
  String d_str = "CURRENTLY: ";

  if(dir == left) {
    d_str += "LEFT";
  }
  if (dir == right) {
    d_str += "RIGHT";
  }

  if (on) {
    d_str += " | ON ";
  } else {
    d_str += " | OFF ";
  }

  d(d_str);
}

// Activate motor depending on direction
void motor(int dir,int speed) {
  // bool on = digitalRead(dir); TODO: use analogRead to figure out the motors speed? or just keep track of the value
  // logmove(dir,on);

  analogWrite(dir, speed);
}

void move(float x,float r) {
  if (x > 0) {
    int speed = 255;
    motor(left,speed);
    // motor(right,speed);
    d("MOVE FORWARD");
  }
  else {
    motor(left,0);
  }

  if (r > 0) {
    int speed = 100;
    // motor(right,speed);
    d("TURN LEFT");
  }
}

// Subscribe to keyboard teleop commands
void teleop( const geometry_msgs::Twist& tele_msg) {
  float x = tele_msg.linear.x;
  float r = tele_msg.angular.z; //Rotation

  move(x,r);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", &teleop );

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
