class TB6552{
public:
  virtual void initialize(int in1_pin, int in2_pin, int stby_pin, int pwm_channel) = 0;
  virtual void turnForward() = 0;
  virtual void turnBackward() = 0;
  virtual void standBy() = 0;
  virtual void shortBrake() = 0;
  virtual void stop() = 0;
  virtual void wakeUp() = 0;
  virtual int getPWMChannel() = 0;
};

class TB6552Impl : public TB6552{
public:
  void initialize(int in1_pin, int in2_pin, int stby_pin, int pwm_channel);
  void turnForward();
  void turnBackward();
  void standBy();
  void shortBrake();
  void stop();
  void wakeUp();
  int getPWMChannel();
private:
  int in1_pin;
  int in2_pin;
  int stby_pin;
  int pwm_channel;

};
