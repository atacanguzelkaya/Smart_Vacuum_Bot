#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Left encoder
int Left_Encoder_PinA = 2;
int Left_Encoder_PinB = 3;
//Right Encoder
int Right_Encoder_PinA = 8;
int Right_Encoder_PinB = 9;

// Pin variables for motors.
const int right_pwm_pin = 10;
const int right_dir_pin = 11;
const int left_pwm_pin = 4;
const int left_dir_pin = 5;
const int vacuum_en_pin = 6;
const int vacuum_dir_pin = 7;
const bool left_fwd = true;
const bool right_fwd = true;

// Default_speed.
const int default_vel = 250;
int state_vel = default_vel;
const int max_vel = 255;

// Robot dimensions. In cm.
const float wheel_dist = 36.0;
ros::NodeHandle  nh;

// ROS stuff.

std_msgs::Float32 int_msg_right;
ros::Publisher motor_right_pub("motor_right", &int_msg_right);

std_msgs::Float32 int_msg_left;
ros::Publisher motor_left_pub("motor_left", &int_msg_left);

void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
  analogWrite(vacuum_dir_pin, 0);
}

void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  // Flipped r and l. Added steering scaler.
  float right_cmd = (-z_rotation*1.8)/2.0 + x;
  float left_cmd = 2.0*x - right_cmd;
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);
  digitalWrite(vacuum_en_pin, HIGH);
  digitalWrite(vacuum_dir_pin, HIGH);
  
  int right_write = int( default_vel * right_cmd);
  int left_write = int( default_vel * left_cmd );
 
  if (x == 0 && z_rotation == 0){
      MoveStop();
  }
     // Advertise the arduino command.
  int abs_left_write =  abs(left_write);
  int abs_right_write = abs(right_write);
  int_msg_right.data = right_write;
  int_msg_left.data = left_write; 
  analogWrite(right_pwm_pin, abs_right_write);
  analogWrite(left_pwm_pin,  abs_left_write);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT); // sets pin A pullup
  pinMode(Left_Encoder_PinB, INPUT); // sets pin B pullup
  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT); // sets pin A pullup
  pinMode(Right_Encoder_PinB, INPUT); // sets pin B pullup
  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(vacuum_en_pin, OUTPUT);
  pinMode(vacuum_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
    // Send forward command.
  analogWrite(right_pwm_pin, default_vel);
  analogWrite(left_pwm_pin, default_vel);
  delay(500);
  MoveStop();

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(motor_right_pub);
  nh.advertise(motor_left_pub);
  Serial.begin(57600);
}
void loop() {
  motor_right_pub.publish(&int_msg_right);
  motor_left_pub.publish(&int_msg_left);
  nh.spinOnce();
  delay(5);
}
