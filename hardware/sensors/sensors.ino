#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define RATE_MS 10

ros::NodeHandle nh;

sensor_msgs::Range ir1_msg;
sensor_msgs::Range ir2_msg;

ros::Publisher ir1_pub( "range/front_left/ir", &ir1_msg);
ros::Publisher ir2_pub( "range/front_right/ir", &ir2_msg);

unsigned long last_ms;

void setup()
{
  nh.getHardware()->setBaud(500000);
  nh.initNode();

  nh.advertise(ir1_pub);
  nh.advertise(ir2_pub);
  
  ir1_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir1_msg.header.frame_id =  "/front_left_ir";
  ir1_msg.field_of_view = 0.001;
  ir1_msg.min_range = 0.03;
  ir1_msg.max_range = 0.4;

  ir2_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir2_msg.header.frame_id =  "/front_right_ir";
  ir2_msg.field_of_view = 0.001;
  ir2_msg.min_range = 0.03;
  ir2_msg.max_range = 0.4;
}

void loop()
{
  if((millis() - last_ms) > RATE_MS){
    last_ms = millis();

    ir1_msg.range = getIRRange(0);
    ir1_msg.header.stamp = nh.now();
    ir1_pub.publish(&ir1_msg);

    ir2_msg.range = getIRRange(1);
    ir2_msg.header.stamp = nh.now();
    ir2_pub.publish(&ir2_msg);
  }

  nh.spinOnce();
}

float getIRRange(int analogPin) {
  return analogRead(analogPin);
}














