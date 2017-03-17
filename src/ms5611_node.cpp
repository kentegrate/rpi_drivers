#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#ifdef HAS_BCM2835
  #include <rpi_drivers/ms5611.hpp>
  MS5611 ms5611;
#else
  #include <rpi_drivers/mock_ms5611.hpp>
  MockMS5611 ms5611;
#endif

int main(int argc, char* argv[]){
  ros::init(argc, argv, "ms5611_node");
  ros::NodeHandle nh;

  ms5611.initialize();

  ros::Publisher pressure_pub = nh.advertise<sensor_msgs::FluidPressure>("pressure", 10);
  ros::Publisher temp_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 10);
  
  ros::Rate rate(100);
  ms5611.update();
  while(ros::ok()){
    float pressure = ms5611.getPressure();
    sensor_msgs::FluidPressure pressure_msg;
    pressure_msg.header.stamp = ros::Time::now();
    pressure_msg.fluid_pressure = pressure;
    pressure_pub.publish(pressure_msg);

    float temp = ms5611.getTemperature();
    sensor_msgs::Temperature temp_msg;
    temp_msg.header.stamp = ros::Time::now();
    temp_msg.temperature = temp;
    temp_pub.publish(temp_msg);
    
    ms5611.update();
    rate.sleep();
  }
  return 0;
}
