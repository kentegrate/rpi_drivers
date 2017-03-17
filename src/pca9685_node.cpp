#include <ros/ros.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#ifdef HAS_BCM2835
  #include <rpi_drivers/pca9685.hpp>
  PCA9685Impl pca9685;
#else
  #include <rpi_drivers/mock_pca9685.hpp>
  MockPCA9685 pca9685;
#endif


void pwmCB(std_msgs::UInt16::ConstPtr length, int channel){
  pca9685.setPWM(channel, length->data);
}

void frequencyCB(std_msgs::Float32::ConstPtr freq){
  pca9685.setFrequency(freq->data);
}
int main(int argc, char* argv[]){

  ros::init(argc, argv, "pca9685_node");
  ros::NodeHandle nh;
  ros::Subscriber freq_sub = nh.subscribe("frequency", 1, frequencyCB);
  std::vector<ros::Subscriber> pwm_subs;

  for(int i = 0; i < 16; i++){
    
    pwm_subs.push_back(nh.subscribe<std_msgs::UInt16> ("channel/" + std::to_string(i), 10, boost::bind(pwmCB, _1, i)));
  }

  int address;
  nh.param("address", address, 0x55);
  pca9685.initialize(address);
  
  int frequency;
  nh.param("frequency", frequency, 1200);
  pca9685.setFrequency(frequency);

  int enable_pin;
  nh.param("enable_pin", enable_pin, 27);
  pca9685.enableOutput(enable_pin);

  for(int i = 0; i < 16; i++){
    pca9685.setPWM(i, 0);
  }

  ros::spin();

  pca9685.finalize(enable_pin);
  return 0;
}
