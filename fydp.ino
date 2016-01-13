/*
 * FYDP Arduino-ROS Bridge
 * Actuates motor depending on ROS commands
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <string.h>


// INITIALIZE VARIABLES
ros::NodeHandle nh;
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);

bool controller = true;
int controller_delay = 400; //ms delay between commands

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

// TODO: Fine Tune acceleration Values for use with PS3 Controller
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

bool x_move = false;
bool r_move = false;
float start_move = 0;
unsigned long end_move_time;

void move2(float ax,float ar) {
  int speed = 255;
  // If no move being executed: check to see if one should be
  if ( millis() - end_move_time > controller_delay) {
    if (abs(ax) > abs(ar) && abs(ax) > 0 && !x_move && !r_move) {
      x_move = true;
      start_move = ax;
      d("Move: Start");
    } else if (abs(ar) > 0 && !x_move && !r_move){
      r_move = true;
      start_move = ar;
      d("Rotate: Start");
    }
  }

  //Moving Forward
  if (x_move) {
    // Start motors on + acceleration change
    if (start_move > 0 && ax >= 0) {
      motor(left,speed);
      motor(right,speed);
      d("Forward");
    }
    // Shut down motors on - acceleration change
    else if(start_move > 0 && ax < 0) {
      x_move = false; //End x move
      motor(left,0);
      motor(right,0);
      d("Move: Stop");
      end_move_time = millis();
    }
    // Moving backwards
    else if (start_move < 0 && ax <= 0) {
      // TODO: PWM reverse?
      d("Reverse");
    }
    else if (start_move < 0 && ax > 0) {
      x_move = false;
      motor(left,0);
      motor(right,0);
      d("Move: Stop");
      end_move_time = millis();
    }
  }
  else if (r_move) {
    if(start_move > 0 && ar >= 0) {
      motor(left,speed);
      motor(right,0);
      d("Left");
    }
    else if(start_move > 0 && ar < 0) {
      r_move = false; //End r move
      motor(left,0);
      motor(right,0);
      d("Rotate: Stop");
      end_move_time = millis();
    }
    // Turn Right
    else if(start_move < 0 && ar <= 0) {
      motor(left,0);
      motor(right,speed);
      d("Right");
    }
    else if(start_move < 0 && ar > 0) {
      r_move = false;
      motor(left,0);
      motor(right,0);
      d("Rotate:Stop");
      end_move_time = millis();
    }
  }

}


float start_time = 0.0;
unsigned long t1;
unsigned long t2 = millis();
float x1;
float x2;
float r1;
float r2;
float ax;
float ar;

void startUp(int dir) {
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



// TODO: verify this is correct
// Subscribe to keyboard teleop commands
void teleop( const geometry_msgs::Twist& tele_msg) {
  t1 = t2;
  t2 = millis();
  x1 = x2;
  r1 = r2;
  x2 = tele_msg.linear.x;
  r2 = tele_msg.angular.z; //Rotation

  ax = (x2 - x1) / (t2 - t1);
  ar = (r2 - r1) / (t2 - t1);

  char temp[20];

  dtostrf(ax,6,4,temp);
  // d(temp);
  dtostrf(ar,6,4,temp);
  // d(temp);
  // d("XXXXXXXXX");

  move2(ax,ar);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", &teleop );
//ros::Subscriber<geometry_msgs::Twist> sub("/teleop_velocity_smoother/raw_cmd_vel", &teleop );

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
  delay(100);
}
