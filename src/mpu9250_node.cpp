#include <ros/ros.h>
#include <rcopter_msgs/Imu.h>

#ifdef HAS_BCM2835
  #include <rpi_drivers/mpu9250.hpp>
  MPU9250Impl mpu;
#else
  #include <rpi_drivers/mock_mpu9250.hpp>
  MockMPU9250 mpu;  
#endif

rcopter_msgs::Imu createImuMsg(float* accel, float* gyro, float* mag){
  rcopter_msgs::Imu msg;
  msg.linear_acceleration.x = accel[0];
  msg.linear_acceleration.y = accel[1];
  msg.linear_acceleration.z = accel[2];

  msg.angular_velocity.x = gyro[0];
  msg.angular_velocity.y = gyro[1];
  msg.angular_velocity.z = gyro[2];

  msg.magnetic_field.x = mag[0];
  msg.magnetic_field.y = mag[1];
  msg.magnetic_field.z = mag[2];

  return msg;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "mpu9250_node");
  ros::NodeHandle nh;
  mpu.initialize(1, 0x01);
  float accel[3];
  float gyro[3];
  float mag[3];

  ros::Publisher pub = nh.advertise<rcopter_msgs::Imu>("imu", 10);
  ros::Rate rate(100);

  while(ros::ok()){
    mpu.getMotion9(accel, accel + 1, accel + 2,
		   gyro, gyro + 1, gyro + 2,
		   mag, mag + 1, mag + 2);
    pub.publish(createImuMsg(accel, gyro, mag));
    rate.sleep();
  }
  return 0;
}
    

