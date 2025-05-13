#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

// PID biến
volatile long xungL = 0, xungR = 0;
double tocdoL = 0, tocdodatL = 0;
double tocdoR = 0, tocdodatR = 0;
double T = 0.01;  // 10ms

// PID thông số
double kp = 3, ki = 0.98, kd = 0.01;

// PID trái
double E = 0, E1 = 0, E2 = 0;
double aL, bL, gL, OutputL = 0, LastOutputL = 0;

// PID phải
double F = 0, F1 = 0, F2 = 0;
double aR, bR, gR, OutputR = 0, LastOutputR = 0;

// ROS Node
ros::NodeHandle nh;

// Nhận lệnh cmd_vel
void cmdVelCallback(const geometry_msgs::Twist &msg) {
  double linear = msg.linear.x;
  double angular = msg.angular.z;
  double L = 0.32;  // khoảng cách 2 bánh
  double R = 0.05;  // bán kính bánh xe

  double vR = (linear + angular * L / 2.0) / (2 * PI * R) * 60; // RPM
  double vL = (linear - angular * L / 2.0) / (2 * PI * R) * 60;

  tocdodatR = vR;
  tocdodatL = vL;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

// Gửi tốc độ thực tế
std_msgs::Float32 velL_msg, velR_msg;
ros::Publisher pubL("wheel_left_vel", &velL_msg);
ros::Publisher pubR("wheel_right_vel", &velR_msg);

// Ngắt encoder
void demxungL() { xungL++; }
void demxungR() { xungR++; }

// PID control
void pid() {
  // Tính tốc độ thực
  tocdoL = (xungL / 320.0) * (60.0 / T);
  xungL = 0;
  tocdoR = (xungR / 320.0) * (60.0 / T);
  xungR = 0;

  // PID trái
  E = tocdodatL - tocdoL;
  aL = 2*T*kp + ki*T*T + 2*kd;
  bL = ki*T*T - 4*kd - 2*T*kp;
  gL = 2*kd;
  OutputL = (aL*E + bL*E1 + gL*E2 + 2*T*LastOutputL) / (2*T);
  LastOutputL = OutputL; E2 = E1; E1 = E;
  if(OutputL > 255) OutputL = 255;
  if(OutputL < 0) OutputL = 0;

  analogWrite(4, OutputL);
  digitalWrite(10, HIGH); digitalWrite(11, LOW);

  // PID phải
  F = tocdodatR - tocdoR;
  aR = 2*T*kp + ki*T*T + 2*kd;
  bR = ki*T*T - 4*kd - 2*T*kp;
  gR = 2*kd;
  OutputR = (aR*F + bR*F1 + gR*F2 + 2*T*LastOutputR) / (2*T);
  LastOutputR = OutputR; F2 = F1; F1 = F;
  if(OutputR > 255) OutputR = 255;
  if(OutputR < 0) OutputR = 0;

  analogWrite(5, OutputR);
  digitalWrite(12, HIGH); digitalWrite(13, LOW);

  // Publish tốc độ thực
  velL_msg.data = tocdoL;
  velR_msg.data = tocdoR;
  pubL.publish(&velL_msg);
  pubR.publish(&velR_msg);
}

void setup() {
  // Encoder
  pinMode(2, INPUT_PULLUP); attachInterrupt(0, demxungL, FALLING);
  pinMode(3, INPUT_PULLUP); attachInterrupt(1, demxungR, FALLING);

  // Motor trái
  pinMode(4, OUTPUT); pinMode(10, OUTPUT); pinMode(11, OUTPUT);

  // Motor phải
  pinMode(5, OUTPUT); pinMode(12, OUTPUT); pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pubL);
  nh.advertise(pubR);

  Timer1.initialize(10000); // 10ms
  Timer1.attachInterrupt(pid);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
