#include <rpi_drivers/tb6552.hpp>
#include <bcm2835.h>
void TB6552Impl::initialize(int in1_pin, int in2_pin, int stby_pin, int pwm_channel){
  bcm2835_init();
  this->in1_pin = in1_pin;
  this->in2_pin = in2_pin;
  this->stby_pin = stby_pin;
  this->pwm_channel = pwm_channel;
  bcm2835_gpio_fsel(in1_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(in2_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(stby_pin, BCM2835_GPIO_FSEL_OUTP);
  standBy();
}

void TB6552Impl::turnForward(){
  bcm2835_gpio_write(in1_pin, HIGH);
  bcm2835_gpio_write(in2_pin, LOW);
  wakeUp();
}
void TB6552Impl::turnBackward(){
  bcm2835_gpio_write(in1_pin, LOW);
  bcm2835_gpio_write(in2_pin, HIGH);
  wakeUp();
}
  
void TB6552Impl::standBy(){
  bcm2835_gpio_write(stby_pin, LOW);
}
void TB6552Impl::wakeUp(){
  bcm2835_gpio_write(stby_pin, HIGH);
}

void TB6552Impl::shortBrake(){
  bcm2835_gpio_write(in1_pin, HIGH);
  bcm2835_gpio_write(in2_pin, HIGH);
  wakeUp();
}

void TB6552Impl::stop(){
  bcm2835_gpio_write(in1_pin, LOW);
  bcm2835_gpio_write(in2_pin, LOW);
  wakeUp();
}

int TB6552Impl::getPWMChannel(){
  return pwm_channel;
}
