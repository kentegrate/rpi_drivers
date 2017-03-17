#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#ifdef HAS_BCM2835
#include <rpi_drivers/tb6552.hpp>
TB6552Impl tb6552;
#else
#include <rpi_drivers/mock_tb6552.hpp>
MockTB6552 tb6552;
#endif
ros::Publisher pwm_pub;

void velocityCB(std_msgs::Float32::ConstPtr velocity){
  if(velocity->data == 0){
    tb6552.standBy();
  }
  else if(velocity->data > 0){
    tb6552.standBy();
    std_msgs::UInt16 pwm_msg;
    pwm_msg.data = velocity->data * 4096;
    pwm_pub.publish(pwm_msg);
    tb6552.turnForward();
  }
  else{
    tb6552.standBy();
    std_msgs::UInt16 pwm_msg;
    pwm_msg.data = velocity->data * -4096;
    pwm_pub.publish(pwm_msg);
    tb6552.turnBackward();
  }
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "tb6552_node");
  ros::NodeHandle nh;
  int in1_pin, in2_pin, stby_pin, pwm_channel;
  std::string pwm_ns;
  nh.param("in1_pin", in1_pin, 13);
  nh.param("in2_pin", in2_pin, 19);
  nh.param("stby_pin", stby_pin, 26);
  nh.param("pwm_channel", pwm_channel, 0);
  nh.param<std::string>("pwm_ns", pwm_ns, "pwm");

  pwm_pub = nh.advertise<std_msgs::UInt16> (pwm_ns + "/channel/" + std::to_string(pwm_channel), 10);
  
  ros::Subscriber velocity_sub = nh.subscribe("channel/" + std::to_string(pwm_channel), 10, velocityCB);
    
  tb6552.initialize(in1_pin, in2_pin, stby_pin, pwm_channel);
  ros::spin();
  return 0;
}
