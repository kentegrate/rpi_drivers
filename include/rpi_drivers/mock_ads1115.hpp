#include <gmock/gmock.h>
#include <rpi_drivers/ads1115.hpp>

class MockADS1115 : public ADS1115 {
 public:
  MOCK_METHOD0(initialize,
      void());
  MOCK_METHOD0(testConnection,
      bool());
  MOCK_METHOD0(getConversion,
      int16_t());
  MOCK_METHOD1(setOpStatus,
      void(uint16_t op));
  MOCK_METHOD0(getMultiplexer,
      uint16_t());
  MOCK_METHOD1(setMultiplexer,
      void(uint16_t mux));
  MOCK_METHOD0(getGain,
      uint16_t());
  MOCK_METHOD1(setGain,
      void(uint16_t gain));
  MOCK_METHOD0(getMode,
      uint16_t());
  MOCK_METHOD1(setMode,
      void(uint16_t mode));
  MOCK_METHOD0(getRate,
      uint16_t());
  MOCK_METHOD1(setRate,
      void(uint16_t rate));
  MOCK_METHOD0(getMilliVolts,
      float());
  MOCK_METHOD1(setComparatorMode,
      void(uint16_t comparatorMode));
  MOCK_METHOD1(setComparatorPolarity,
      void(uint16_t polarit));
  MOCK_METHOD1(setComparatorLatchEnabled,
      void(uint16_t latchStatus));
  MOCK_METHOD1(setComparatorQueueMode,
      void(uint16_t queueMode));
};
