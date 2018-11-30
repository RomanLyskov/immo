#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Servo.h>
#include <Encoder.h>  
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS 20
#define MOTOR_PIN_1 9 // ------
#define MOTOR_PIN_2 10 // Relocate pins for best wiring and using interrupt pins with motor encoder
#define WHEEL_PIN 8 // ------


ros::NodeHandle  nh;
Servo wheel;
Encoder encoderRight (2, 4);
Encoder encoderLeft (3, 5);

void drive_cb(const geometry_msgs::Twist& cmd_vel){
  float linear = map(cmd_vel.linear.x * 1000, -1000, 1000, -255, 255);
  float angular = map(cmd_vel.angular.z * 100, -300, 300, 1000, 2000);
  wheel.write(angular);
  if (linear > 1){
    analogWrite(MOTOR_PIN_1, linear);
    analogWrite(MOTOR_PIN_2, 0);
  }
  else if (linear < -1){
    analogWrite(MOTOR_PIN_2, -linear);
    analogWrite(MOTOR_PIN_1, 0);
  }
  else if (linear == 0){
    analogWrite(MOTOR_PIN_2, 0);
        analogWrite(MOTOR_PIN_1, 0);
        }
  }

nav_msgs::Odometry en_r_msg;
nav_msgs::Odometry en_l_msg;

ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);
ros::Publisher encoder_r_pub( "odometry/encoder/right", &en_r_msg);
ros::Publisher encoder_l_pub( "odometry/encoder/left", &en_l_msg);

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  
  wheel.attach(WHEEL_PIN);  
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(encoder_r_pub);
  nh.advertise(encoder_l_pub);
  nh.subscribe(drive_sub);
  }

void loop(){
  nh.spinOnce();
  delay(10);
}
