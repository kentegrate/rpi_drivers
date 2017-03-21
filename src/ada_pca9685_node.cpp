#include <ros/ros.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <rpi_drivers/ada_pca9685.hpp>
Ada_ServoDriver pca9685;


void pwmCB(std_msgs::UInt16::ConstPtr length, int channel){
  pca9685.setPWM(channel, 0, length->data);
}

void frequencyCB(std_msgs::Float32::ConstPtr freq){
  pca9685.setPWMFreq(freq->data);
}
int main(int argc, char* argv[]){

  ros::init(argc, argv, "pca9685_node");
  ros::NodeHandle nh("~");
  ros::Subscriber freq_sub = nh.subscribe("frequency", 1, frequencyCB);
  std::vector<ros::Subscriber> pwm_subs;

  for(int i = 0; i < 16; i++){
    
    pwm_subs.push_back(nh.subscribe<std_msgs::UInt16> ("channel/" + std::to_string(i), 10, boost::bind(pwmCB, _1, i)));
  }

  int address;
  nh.param("address", address, 0x55);
  pca9685.init();
  pca9685.reset();
  usleep(100000);
  pca9685.setPWMFreq(600);
  
  ros::spin();
  return 0;
}
