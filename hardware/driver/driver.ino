#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS          20
#define MOTOR_PIN_1      9
#define MOTOR_PIN_2      10
#define RUDDER_PIN       8
#define ENCODER_RIGHT_A  2
#define ENCODER_RIGHT_B  4
#define ENCODER_LEFT_A   3
#define ENCODER_LEFT_B   5
#define NUM_JOINTS       3

bool encodersALast[2] = {LOW, LOW}; //0-LEFT, 1-RIGHT
char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel", "rudder"};
float state_pos[NUM_JOINTS] = {0, 0, 0};
float state_vel[NUM_JOINTS] = {0, 0, 0};
float state_eff[NUM_JOINTS] = {0, 0, 0};
unsigned long last_ms;

ros::NodeHandle nh;

Servo rudder;
float linear;
float angular;

void drive_cb(const geometry_msgs::Twist& cmd_vel){
  linear = map(cmd_vel.linear.x * 1000, -1000, 1000, -255, 255);
  angular = map(cmd_vel.angular.z * 1000, -3000, 3000, 1000, 2000);
  rudder.write(angular);
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

sensor_msgs::JointState state_msg;

ros::Publisher state_pub("joint_states", &state_msg);
ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  rudder.attach(RUDDER_PIN);
  rudder.write(1500);

  //nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(drive_sub);
  nh.advertise(state_pub);

  state_msg.header.frame_id =  "/driver_states";
  state_msg.name_length = NUM_JOINTS;
  state_msg.velocity_length = NUM_JOINTS;
  state_msg.position_length = NUM_JOINTS;
  state_msg.effort_length = NUM_JOINTS;
  state_msg.name = state_names;
  state_msg.position = state_pos;
  state_msg.velocity = state_vel;
  state_msg.effort = state_eff;
  }

void loop(){
  if((millis() - last_ms) >= RATE_MS){
    last_ms = millis();
    state_pos[0] = -encoderCount(ENCODER_LEFT_A, ENCODER_LEFT_B, 0);  //left
    state_pos[1] = encoderCount(ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0); //right
    state_pos[2] = angular;
    state_msg.header.stamp = nh.now();
    state_pub.publish(&state_msg);
  }
  nh.spinOnce();
  delay(5);
}

int encoderCount(int encoderA, int encoderB, int side) {
  int count = 0;
  int n = digitalRead(encoderA);
  if ((encodersALast[side] == LOW) && (n == HIGH)) {
    if (digitalRead(encoderB) == LOW) {
      count--;
    } else {
      count++;
    }
  }
  encodersALast[side] = n;
  return count;
}
