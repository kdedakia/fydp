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

  char d_arr[100];
  d_str.toCharArray(d_arr,100);
  debug_msg.data = d_arr;
  debug.publish( &debug_msg);
}

// Activate motor depending on direction
void motor(int dir,int speed) {
  // debug_msg.data = "IN MOTOR";
  // debug.publish( &debug_msg);
  bool on = digitalRead(dir);
  // logmove(dir,on);

  if (on) {
    digitalWrite(dir, LOW);
  } else {
    digitalWrite(dir, HIGH);
  }
}

void motor_test(int dir,int speed){
  analogWrite(left,speed);
}

void move(float x,float r) {
  if (x > 0) {
    int speed = 255;
    // motor(left,speed);
    // motor(right,speed);
    motor_test(left,speed);
    d("MOVE FORWARD");
  }
  else {
    motor_test(left,0);
  }

  if (r > 0) {
    int speed = 100;
    // motor(right,speed);
    d("TURN LEFT");
  }
}

// Callback function that is executed when a command is published
void messageCb( const std_msgs::String& toggle_msg){
  String command = toggle_msg.data;

}

void teleop( const geometry_msgs::Twist& tele_msg) {
  float x = tele_msg.linear.x;
  float r = tele_msg.angular.z; //Rotation
  String log;
  char x_string[10];
  char r_string[10];
  dtostrf(x,7,3,x_string);
  dtostrf(r,7,3,r_string);
  // d("X VELOCITY");
  // d(x_string);
  // d("ROTATION");
  // d(r_string);

  move(x,r);
}

// ros::Subscriber<std_msgs::String> sub("command", &messageCb );
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
  // delay(1);
  delay(1);
}
