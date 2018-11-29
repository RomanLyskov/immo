#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS 20
#define WHEEL_PIN 4
#define MOTOR_PIN_1 3
#define MOTOR_PIN_2 5

ros::NodeHandle  nh;
Servo wheel;

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

ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  wheel.attach(WHEEL_PIN);
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(drive_sub);
  }

void loop(){
  nh.spinOnce();
  delay(10);
