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

void move(float ax,float ar) {
  if (ax > 0) {
    int speed = 255;
    motor(left,speed);
    // motor(right,speed);
    d("MOVE FORWARD");
  }
  else {
    motor(left,0);
  }

  if (ar > 0) {
    int speed = 100;
    // motor(right,speed);
    d("TURN LEFT");
  }
}

float start_time = 0.0;

void startUp(dir) {
  if (start_time == 0.0) {
    start_time = millis();
    motor(dir,100);
  }
  else if(millis() - start_time > 500) {
    motor(dir,150);
  }
  else if(millis() - start_time > 1000) {
    motor(dir,200);
  }

  t1 = t2;
  t2 = millis();

}

float t1;
float t2 = millis();
float x1;
float x2;
float r1;
float r2;
float ax;
float ar;

// Subscribe to keyboard teleop commands
void teleop( const geometry_msgs::Twist& tele_msg) {
  t1 = t2;
  t2 = millis();
  old_x = x;
  old_r = r;
  x = tele_msg.linear.x;
  r = tele_msg.angular.z; //Rotation

  ax = (x2 - x1) / (t2 - t1);
  ar = (r2 - r1) / (t2 - t1);

  move(ax,ar);
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
