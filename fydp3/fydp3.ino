/*
 * FYDP Arduino-ROS Bridge
 * Actuates motor depending on ROS commands
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <string.h>


// INITIALIZE PINS
const int leftPwm = 6;
const int leftDir = 7;

const int rightPwm = 5;
const int rightDir = 4;


// INITIALIZE VARIABLES
ros::NodeHandle nh;
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);
float x;
float r;
const float r_thresh = 1.0;

// Debug message
void d(String s) {
    char s_arr[100];
    s.toCharArray(s_arr,100);
    debug_msg.data = s_arr;
    debug.publish( &debug_msg);
}


void teleop( const geometry_msgs::Twist& tele_msg) {
  x = tele_msg.linear.x * 510;
  r = tele_msg.angular.z; //Rotation

  char temp[20];
  dtostrf(r,6,4,temp);
  d(temp);
  if (fabs(r) > r_thresh) {
      rotate(r);
  } else {
      move(x);
  }
}

void rotate(int r) {
    int speed = 100;
    // Go Left
    if (r >= 0) {
      //analogWrite(leftPwm, speed);
      //digitalWrite(leftDir, LOW);
      analogWrite(rightPwm, speed);
      digitalWrite(rightDir, HIGH);
    }
    // Go Right
    else {
      analogWrite(leftPwm, speed);
      digitalWrite(leftDir, HIGH);
      //analogWrite(rightPwm, speed);
      //digitalWrite(rightDir, LOW);
    }
}

void move(int speed) {
  if (speed >= 0) {
    analogWrite(leftPwm, speed);
    digitalWrite(leftDir, LOW);
    analogWrite(rightPwm, speed);
    digitalWrite(rightDir, LOW);
  }
  else {
    analogWrite(leftPwm, -speed);
    digitalWrite(leftDir, HIGH);
    analogWrite(rightPwm, -speed);
    digitalWrite(rightDir, HIGH);
  }
}


ros::Subscriber<geometry_msgs::Twist> sub("/teleop_velocity_smoother/raw_cmd_vel", &teleop );


void setup()
{
  pinMode(leftPwm, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(rightPwm, OUTPUT);
  pinMode(rightDir, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug);
}


void loop()
{
  nh.spinOnce();
  delay(10);
}
