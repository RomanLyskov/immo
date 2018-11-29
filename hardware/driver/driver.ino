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
//unsigned long last_ms;

void drive_cb(const geometry_msgs::Twist& cmd_vel)
{
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

//sensor_msgs::JointState state_msg;

//#define NUM_JOINTS 3

//char *state_names[NUM_JOINTS] = {"lwheel", "rwheel", "rudder"};
//float state_pos[NUM_JOINTS] = {0, 0, 0};
//float state_vel[NUM_JOINTS] = {0, 0, 0};
//float state_eff[NUM_JOINTS] = {0, 0, 0};

//ros::Publisher state_pub("joint_states", &state_msg);

void setup() {
pinMode(MOTOR_PIN_1, OUTPUT);
pinMode(MOTOR_PIN_2, OUTPUT);
wheel.attach(WHEEL_PIN);

//  nh.getHardware()->setBaud(500000);
nh.initNode();
nh.subscribe(drive_sub);
//  nh.advertise(state_pub);

//  state_msg.header.frame_id =  "/driver_states";
//  state_msg.name_length = NUM_JOINTS;
//  state_msg.velocity_length = NUM_JOINTS;
//  state_msg.position_length = NUM_JOINTS;
//  state_msg.effort_length = NUM_JOINTS;
//  state_msg.name = state_names;
//  state_msg.position = state_pos;
//  state_msg.velocity = state_vel;
//  state_msg.effort = state_eff;
}

void loop(){
//  if((millis() - last_ms) >= RATE_MS){
//    last_ms = millis();
//    // TODO: Get values
//    state_pos[0] = 1;
//    state_pos[1] = 12212;
//    state_pos[2] = 32323;
//    state_msg.header.stamp = nh.now();
//    state_pub.publish(&state_msg);
//  }

  nh.spinOnce();
  delay(10);
}
