#include <gmock/gmock.h>
#include <rpi_drivers/pca9685.hpp>

class MockPCA9685 : public PCA9685 {
 public:
  MOCK_METHOD2(writeByte,
      uint8_t(uint8_t regAddr, char *data));
  MOCK_METHOD3(writeBytes,
      uint8_t(uint8_t regAddr, char *data, uint32_t bytes));
  MOCK_METHOD2(readByte,
      uint8_t(uint8_t regAddr, char *data));
  MOCK_METHOD3(writeBit,
      uint8_t(uint8_t regAddr, uint8_t bitNum, char data));
  MOCK_METHOD1(initialize,
      void(int address));
  MOCK_METHOD1(finalize,
      void(int enable_pin));
  MOCK_METHOD0(testConnection,
      bool());
  MOCK_METHOD1(enableOutput,
      void(int enable_pin));
  MOCK_METHOD1(disableOutput,
      void(int enable_pin));
  MOCK_METHOD0(getFrequency,
      float());
  MOCK_METHOD1(setFrequency,
      void(float frequency));
  MOCK_METHOD0(sleep,
      void());
  MOCK_METHOD0(restart,
      void());
  MOCK_METHOD3(setPWM,
      void(uint8_t channel, uint16_t offset, uint16_t length));
  MOCK_METHOD2(setPWM,
      void(uint8_t channel, uint16_t length));
  MOCK_METHOD2(setPWMmS,
      void(uint8_t channel, float length_mS));
  MOCK_METHOD2(setPWMuS,
      void(uint8_t channel, float length_uS));
  MOCK_METHOD2(setAllPWM,
      void(uint16_t offset, uint16_t length));
  MOCK_METHOD1(setAllPWM,
      void(uint16_t length));
  MOCK_METHOD1(setAllPWMmS,
      void(float length_mS));
  MOCK_METHOD1(setAllPWMuS,
      void(float length_uS));
};
