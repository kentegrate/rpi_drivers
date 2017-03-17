#include <gmock/gmock.h>
#include <rpi_drivers/ms5611.hpp>

class MockMS5611 : public MS5611 {
 public:
  MOCK_METHOD2(WriteReg,
      char(uint8_t WriteAddr, char WriteData));
  MOCK_METHOD1(ReadReg,
      char(uint8_t WriteAddr));
  MOCK_METHOD3(ReadRegs,
      void(uint8_t ReadAddr, char *ReadBuf, unsigned int Bytes));
  MOCK_METHOD0(initialize,
      void());
  MOCK_METHOD0(testConnection,
      bool());
  MOCK_METHOD1(refreshPressure,
      void(uint8_t));
  MOCK_METHOD0(readPressure,
      void());
  MOCK_METHOD1(refreshTemperature,
      void(uint8_t));
  MOCK_METHOD0(readTemperature,
      void());
  MOCK_METHOD0(calculatePressureAndTemperature,
      void());
  MOCK_METHOD0(update,
      void());
  MOCK_METHOD0(getTemperature,
      float());
  MOCK_METHOD0(getPressure,
      float());
};
