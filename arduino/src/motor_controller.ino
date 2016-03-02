// Author: Sung Jik Cha
// Credits:
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

#define leftPwm 6
#define leftDir 7
#define rightPwm 11
#define rightDir 13
#define encodPinA1      3     // encoder A pinn
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      9
#define LOOPTIME        100   // PID loop time(ms)

/*
  WIRING CONFIGURATION


  Top Shield: RIGHT
	White: 9
	Yellow: 2
	Blue: Vcc
	Green: GND
	Black: B
	Red: A


  Bottom Shield: LEFT
	White: 8
	Yellow: 3
	Blue: Vcc
	Green: GND
	Black: A
	Red: B

*/

unsigned long lastMilli = 0;       // loop timing
// unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
  // d("Req1: " + String(rpm_req1));
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", &handle_cmd);
// ros::Subscriber<geometry_msgs::Twist> sub("/teleop_velocity_smoother/raw_cmd_vel", &handle_cmd );

geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
std_msgs::String debug_msg;
ros::Publisher debug("debug", &debug_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(debug);
 nh.advertise(rpm_pub);

 pinMode(leftPwm, OUTPUT);
 pinMode(leftDir, OUTPUT);
 pinMode(encodPinA1, INPUT);
 pinMode(encodPinB1, INPUT);
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, encoder1, RISING);

 pinMode(rightPwm, OUTPUT);
 pinMode(rightDir, OUTPUT);
 pinMode(encodPinA2, INPUT);
 pinMode(encodPinB2, INPUT);
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);
}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);

    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2,PWM_val2,rpm_req2, rpm_act2);

    if (rpm_req1 == 0.0 && rpm_act1 == 0.0) {
      PWM_val1 = 0;
    }
    if (rpm_req2 == 0.0 && rpm_act2 == 0.0) {
      PWM_val2 = 0;
    }

    d("RPM REQ1: " + String(rpm_req1));
    d("RPM ACTUAL1: " + String(rpm_act1));
    d("PWM 1: " + String(PWM_val1));

    d("RPM REQ2: " + String(rpm_req2));
    d("RPM ACTUAL2 : " + String(rpm_act2));
    d("PWM 2: " + String(PWM_val2));

    if(PWM_val1 > 0) {
      digitalWrite(leftDir,LOW);
    } else if (PWM_val1 < 0) {
      digitalWrite(leftDir,HIGH);
    }

    if(PWM_val2 > 0) {
      digitalWrite(rightDir,LOW);
    } else if (PWM_val2 < 0) {
      digitalWrite(rightDir,HIGH);
    }

    analogWrite(leftPwm,abs(PWM_val1));
    analogWrite(rightPwm,abs(PWM_val2));

    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  // if(time-lastMilliPub >= LOOPTIME) {
  // //  publishRPM(time-lastMilliPub);
  //   lastMilliPub = time;
  // }
}

// Debug message
void d(String s) {
    char s_arr[100];
    s.toCharArray(s_arr,100);
    debug_msg.data = s_arr;
    debug.publish( &debug_msg);
}

void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  error = targetValue-currentValue;

  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }

  float resolution = 255.0;
  new_pwm = constrain(double(command)*MAX_RPM/resolution + pidTerm, -MAX_RPM, MAX_RPM);

  new_cmd = resolution*new_pwm/MAX_RPM;
  return int(new_cmd);
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}


void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}
