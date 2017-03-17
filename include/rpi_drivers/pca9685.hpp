/*
PCA9685 driver code is placed under the BSD license.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
Copyright (c) 2014, Emlid Limited
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*From Navio Resository adapted for bcm_2835 library by Ken Takaki
https://github.com/emlid/Navio/blob/master/C%2B%2B/Navio/PCA9685.h
 */

#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string>

#define PCA9685_DEFAULT_ADDRESS     0x40 // All address pins low, Navio default

#define PCA9685_RA_MODE1            0x00
#define PCA9685_RA_MODE2            0x01
#define PCA9685_RA_LED0_ON_L        0x06
#define PCA9685_RA_LED0_ON_H        0x07
#define PCA9685_RA_LED0_OFF_L       0x08
#define PCA9685_RA_LED0_OFF_H       0x09
#define PCA9685_RA_ALL_LED_ON_L     0xFA
#define PCA9685_RA_ALL_LED_ON_H     0xFB
#define PCA9685_RA_ALL_LED_OFF_L    0xFC
#define PCA9685_RA_ALL_LED_OFF_H    0xFD
#define PCA9685_RA_PRE_SCALE        0xFE

#define PCA9685_MODE1_RESTART_BIT   7
#define PCA9685_MODE1_EXTCLK_BIT    6
#define PCA9685_MODE1_AI_BIT        5
#define PCA9685_MODE1_SLEEP_BIT     4
#define PCA9685_MODE1_SUB1_BIT      3
#define PCA9685_MODE1_SUB2_BIT      2
#define PCA9685_MODE1_SUB3_BIT      1
#define PCA9685_MODE1_ALLCALL_BIT   0

#define PCA9685_MODE2_INVRT_BIT     4
#define PCA9685_MODE2_OCH_BIT       3
#define PCA9685_MODE2_OUTDRV_BIT    2
#define PCA9685_MODE2_OUTNE1_BIT    1
#define PCA9685_MODE2_OUTNE0_BIT    0

class PCA9685 {
 public:
  virtual uint8_t writeByte(uint8_t regAddr, char *data) = 0;
  virtual uint8_t writeBytes(uint8_t regAddr, char *data, uint32_t bytes) = 0;
  virtual uint8_t readByte(uint8_t regAddr, char *data) = 0;
  virtual uint8_t writeBit(uint8_t regAddr, uint8_t bitNum, char data) = 0;

  virtual void initialize(int address) = 0;
  virtual void finalize() = 0;
  virtual bool testConnection() = 0;

  virtual void enableOutput(int enable_pin) = 0;
  virtual void disableOutput(int enable_pin) = 0;

  virtual float getFrequency() = 0;
  virtual void setFrequency(float frequency) = 0;

  virtual void sleep() = 0;
  virtual void restart() = 0;

  virtual void setPWM(uint8_t channel, uint16_t offset, uint16_t length) = 0;
  virtual void setPWM(uint8_t channel, uint16_t length) = 0;
  virtual void setPWMmS(uint8_t channel, float length_mS) = 0;
  virtual void setPWMuS(uint8_t channel, float length_uS) = 0;

  virtual void setAllPWM(uint16_t offset, uint16_t length) = 0;
  virtual void setAllPWM(uint16_t length) = 0;
  virtual void setAllPWMmS(float length_mS) = 0;
  virtual void setAllPWMuS(float length_uS) = 0;

 private:
  float frequency;
};

#endif // PCA9685_ HPP
