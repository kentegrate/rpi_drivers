class TB6552{
public:
  virtual void initialize(int in1_pin, int in2_pin, int stby_pin, int pwm_channel) = 0;
  virtual void turnForward() = 0;
  virtual void turnBackward() = 0;
  virtual void standBy() = 0;
  virtual void shortBrake() = 0;
  virtual void stop() = 0;
  virtual void wakeUp() = 0;
  virtual void getPWMChannel() = 0;
private:
  int in1_pin;
  int in2_pin;
  int stby_pin;
  int pwm_channel;
};
