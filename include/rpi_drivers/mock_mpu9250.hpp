#include <gmock/gmock.h>
#include <rpi_drivers/mpu9250.hpp>

class MockMPU9250 : public MPU9250 {
 public:
  MOCK_METHOD2(initialize,
      bool(int, int));
  MOCK_METHOD0(testConnection,
      bool());
  MOCK_METHOD2(WriteReg,
      char(uint8_t WriteAddr, char WriteData));
  MOCK_METHOD1(ReadReg,
      char(uint8_t WriteAddr));
  MOCK_METHOD3(ReadRegs,
      void(uint8_t ReadAddr, char *ReadBuf, unsigned int Bytes));
  MOCK_METHOD1(set_gyro_scale,
      unsigned int(int scale));
  MOCK_METHOD1(set_acc_scale,
      unsigned int(int scale));
  MOCK_METHOD0(calib_acc,
      void());
  MOCK_METHOD0(calib_mag,
      void());
  MOCK_METHOD0(read_temp,
      void());
  MOCK_METHOD0(read_acc,
      void());
  MOCK_METHOD0(read_gyro,
      void());
  MOCK_METHOD0(read_mag,
      void());
  MOCK_METHOD0(read_all,
      void());
  MOCK_METHOD0(whoami,
      unsigned int());
  MOCK_METHOD0(AK8963_whoami,
      uint8_t());
  MOCK_METHOD9(getMotion9,
      void(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz));
  MOCK_METHOD6(getMotion6,
      void(float *ax, float *ay, float *az, float *gx, float *gy, float *gz));
};
