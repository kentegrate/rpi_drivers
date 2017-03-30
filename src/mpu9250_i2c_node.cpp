#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#ifdef HAS_BCM2835
  #include <rpi_drivers/mpu9250_i2c.hpp>
MPU9250 mpu(0x68);
#else
  #include <rpi_drivers/mock_mpu9250.hpp>
  MockMPU9250 mpu;  
#endif

sensor_msgs::Imu createImuMsg(float* accel, float* gyro){
  sensor_msgs::Imu msg;
  msg.linear_acceleration.x = accel[0];
  msg.linear_acceleration.y = accel[1];
  msg.linear_acceleration.z = accel[2];

  msg.angular_velocity.x = gyro[0];
  msg.angular_velocity.y = gyro[1];
  msg.angular_velocity.z = gyro[2];

  return msg;
}

sensor_msgs::MagneticField createMagMsg(float *mag){
  sensor_msgs::MagneticField msg;
  msg.magnetic_field.x = mag[0];
  msg.magnetic_field.y = mag[1];
  msg.magnetic_field.z = mag[2];

  return msg;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "mpu9250_node");
  ros::NodeHandle nh("~");
  int beginStatus = mpu.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
    if(beginStatus < 0) {
    fprintf(stderr, "IMU initialization unsuccessful\n");
    fprintf(stderr, "Check IMU wiring or try cycling power\n");
    return 0;
  }

  float accel[3];
  float gyro[3];
  float mag[3];
  
  float accel_offset[3];
  float gyro_offset[3];
  float mag_offset[3];
  for(int i = 0; i < 3; i++){
    accel_offset[i] = 0;
    gyro_offset[i] = 0;
    mag_offset[i] = 0;
  }

  
  ros::Rate rate(100);
  
  
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data_raw", 10);
  ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag", 10);

    for(int i = 0; i < 500; i++){
        mpu.getMotion9(accel, accel + 1, accel + 2,
		   gyro, gyro + 1, gyro + 2,
		   mag, mag + 1, mag + 2);
        rate.sleep();
        for(int j = 0; j < 3; j++){
          accel_offset[j] += accel[j];
          gyro_offset[j] += gyro[j];
          mag_offset[j] += mag[j];
        }
  }
  for(int i = 0; i < 3; i++){
    accel_offset[i] /= 500;
    gyro_offset[i] /= 500;
    mag_offset[i] /= 500;
  }

  
  while(ros::ok()){
    mpu.getMotion9(accel, accel + 1, accel + 2,
                   gyro, gyro + 1, gyro + 2,
                   mag, mag + 1, mag + 2);
    for(int i = 0; i < 3; i++){
      accel[i] -= accel_offset[i];
      gyro[i] -= gyro_offset[i];
      mag[i] -= mag_offset[i];
    }

    imu_pub.publish(createImuMsg(accel, gyro));
    mag_pub.publish(createMagMsg(mag));
    
    rate.sleep();
  }
  return 0;
}
    

