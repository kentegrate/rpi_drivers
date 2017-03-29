#include <gmock/gmock.h>
#include <rpi_drivers/tb6552.hpp>

class MockTB6552 : public TB6552 {
 public:
  MOCK_METHOD4(initialize,
      void(int in1_pin, int in2_pin, int stby_pin, int pwm_channel));
  MOCK_METHOD0(turnForward,
      void());
  MOCK_METHOD0(turnBackward,
      void());
  MOCK_METHOD0(standBy,
      void());
  MOCK_METHOD0(shortBrake,
      void());
  MOCK_METHOD0(stop,
      void());
  MOCK_METHOD0(wakeUp,
      void());
  MOCK_METHOD0(getPWMChannel,
      int());
};
