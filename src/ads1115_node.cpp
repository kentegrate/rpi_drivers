#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <gtest/gtest.h>

#ifdef HAS_BCM2835
  #include <rpi_drivers/ads1115.hpp>
  ADS1115Impl adc;
#else
  #include <rpi_drivers/mock_ads1115.hpp>
  MockADS1115 adc;
#endif

sensor_msgs::BatteryState createBatteryMsg(float voltage_raw, float current_raw){
  sensor_msgs::BatteryState msg;
  msg.header.stamp = ros::Time::now();
  msg.voltage = voltage_raw;
  msg.current = current_raw;
  return msg;
}

int main(int argc, char* argv[]){
  
  ros::init(argc, argv, "ads1115_node");
  ros::NodeHandle nh;
  adc.setMode(ADS1115_MODE_CONTINUOUS);
  adc.setRate(ADS1115_RATE_860);

  ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);
  ros::Rate rate(100);
  
  uint16_t muxes[] = {ADS1115_MUX_P0_NG, ADS1115_MUX_P1_NG};
  int current_port = 0;
  while(ros::ok()){
    adc.setMultiplexer(muxes[0]);
    float voltage_raw = adc.getMilliVolts();
    adc.setMultiplexer(muxes[1]);
    float current_raw = adc.getMilliVolts();
    sensor_msgs::BatteryState msg = createBatteryMsg(voltage_raw, current_raw);
    battery_pub.publish(msg);
    rate.sleep();
  }
  return 0;
}

    
