#include <rpi_drivers/tb6552.hpp>
#include <bcm2835.h>

void initialize(int in1_pin, int in2_pin, int stby_pin, int pwm_channel){
  this->in1_pin = in1_pin;
  this->in2_pin = in2_pin;
  this->stby_pin = stby_pin;
  this->pwm_channel = pwm_channel;
  bcm2835_gpio_fsel(in1_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(in2_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(stby_pin, BCM2835_GPIO_FSEL_OUTP);
  standBy();
}

void turnForward(){
  bcm2835_gpio_write(in1_pin, HIGH);
  bcm2835_gpio_write(in2_pin, LOW);
  wakeUp();
}
void turnBackward(){
  bcm2835_gpio_write(in1_pin, LOW);
  bcm2835_gpio_write(in2_pin, HIGH);
  wakeUp();
}
  
void standBy(){
  bcm2835_gpio_write(stby_pin, LOW);
}
void wakeUp(){
  bcm2835_gpio_write(stby_pin, HIGH);
}

void shortBrake(){
  bcm2835_gpio_write(in1_pin, HIGH);
  bcm2835_gpio_write(in2_pin, HIGH);
  wakeUp();
}

void stop(){
  bcm2835_gpio_write(in1_pin, LOW);
  bcm2835_gpio_write(in2_pin, LOW);
  wakeUp();
}

int getPWMChannel(){
  return pwm_channel;
}
