#include <rpi_drivers/ada_pca9685.h>
Ada_ServoDriver::Ada_ServoDriver()
{
}

void Ada_ServoDriver::write8(uint8_t addr, uint8_t d)
{
  uint8_t sendData[2];

  sendData[0] = addr;
  sendData[1] = d;
  if (write(_i2c, sendData, 2) != 2)
    {
      printf("Faild to send i2c\n");
    }
}

uint8_t Ada_ServoDriver::read8(uint8_t addr)
{
  uint8_t sendData;
  uint8_t readData;

  sendData = addr;
  if (write(_i2c, &sendData, 1) != 1)
    {
      printf("Failed to send i2c @read\n");
    }
  else
    {
      if (read(_i2c, &readData, 1) != 1)
	{
	  printf("Failed to read i2c\n");
	}
    }

  return readData;
}

void Ada_ServoDriver::reset(void)
{
  write8(PCA9685_MODE1, 0x0);
}

void Ada_ServoDriver::setPWMFreq(float freq)
{
  float prescaleval = 25000000;

  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  printf("Estimated pre-scale: %f\n", prescaleval);

  uint8_t prescale = floor(prescaleval + 0.5);
  printf("Final pre-scale: %d\n", prescale);

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10;
  write8(PCA9685_MODE1, newmode);
  write8(PCA9685_PRESCALE, prescale);
  write8(PCA9685_MODE1, oldmode);
  sleep(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);
}

void Ada_ServoDriver::setPWM(uint8_t srvNo, uint16_t onTime, uint16_t offTime)
{
  uint8_t sendData[5];

  sendData[0] = LED0_ON_L + 4 * srvNo;
  sendData[1] = (uint8_t)(0x00ff & onTime);
  sendData[2] = (uint8_t)((0xff00 & onTime) >> 8);
  sendData[3] = (uint8_t)(0x00ff & offTime);
  sendData[4] = (uint8_t)((0xff00 & offTime) >> 8);

  if (write(_i2c, sendData, 5) != 5)
    {
      printf("Faild to send i2c @setPWM\n");
    }
}

#define SERVO_CONTROL_FREQUENCY 60
#define SERVO_CENTER_PULSE_WIDTH_US 1520
#define SERVO_CENTER_PULSE_WIDTH_US_FUTABA_OLD  1310
#define SERVO_RANGE_PULSE_WIDTH_US 1600

void Ada_ServoDriver::setServoPulse(uint8_t ch, double pulseWidth_us)
{
  double pulselength;
  double pulseWidth;

  // 1秒=1000000usを60Hzで割ってパルス長を算出。
  pulselength = 1000000 / SERVO_CONTROL_FREQUENCY;
  // 12bit(2^12=4096)分解能相当へ。1分解能当たりの時間算出。
  pulselength /= 4096;
  // PWMのパルス設定値を算出。
  pulseWidth = pulseWidth_us / pulselength;

  // PWM値設定。
  //  setPWM(channel, on_timing, off_timing)
  //  channelで指定したチャネルのPWM出力のon(0→1）になるタイミングと
  //  off(1→0)になるタイミングを0～4095で設定する。
  setPWM(ch, 0, pulseWidth);
}
void Ada_ServoDriver::init(){
  const char *i2cFilename = "/dev/i2c-1";
  int driverAddress = 0x55;
  _i2cAddr = driverAddress;
  int i2c;
  if ((i2c = open(i2cFilename, O_RDWR)) < 0)
    {
      printf("Faild to open i2c port\n");
      exit(1);
    }

  //
  if (ioctl(i2c, I2C_SLAVE, driverAddress) < 0)
    {
      printf("Unable to get bus access to talk to slave\n");
      exit(1);
    }
  _i2c = i2c;
}
