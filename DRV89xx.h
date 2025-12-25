/*
  DRV89xx.h - Library for controlling brushed DC motors using a DRV89xx
  Currently written and tested for the ESP32 processor
  Created by Joseph Duchesne, September 23, 2020
  Adapted by Carter Rommelfanger for STM, December 23, 2025
  Licensed under BSD 3 Clause 
*/
#ifndef DRV89xx_h
#define DRV89xx_h

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include "stm32g474xx.h"
#include "DRV89xxMotor.h"
#include "CustomTypes.h"
#include "STM32Utils.h"
#include <cstdio>

#define DRV89xx_REGISTER_READ 0x4000u
#define DRV89xx_MAX_MOTORS 6
#define DRV89xx_CONFIG_WRITE_START 0x7u
#define DRV89xx_CONFIG_BYTES 0x25u
#define DRV89xx_UPDATE_START 0x08u
#define DRV89xx_UPDATE_END 0x16u

#define DRV89xx_FORWARD 1
#define DRV89xx_BRAKE 0
#define DRV89xx_REVERSE -1

#define MAX_NUM_FAULTS_ALLOWED 10

#define IS_BIT_SET(a, n) ((a & (1 << (n))) != 0)

typedef struct {
	GPIO_TypeDef* port;
	uint16_t 	  pin;
} GPIOPinPort;

class DRV89xx
{

  public:
    DRV89xx(GPIOPinPort cs, GPIOPinPort nFault, GPIOPinPort nSleep, SPI_HandleTypeDef* hSPI, SysClock* clk);
    void begin();

    void configMotor(byte motor_id, byte hb1, byte hb2, byte pwm_channel, byte reverse_delay);
    byte writeRegister(byte address, byte value);
    byte readRegister(byte address);
    void readErrorStatus(bool print, bool reset);
    void disableErroredMotors(byte statusRegister);
    void writeConfig();
    void updateConfig(bool errorCheck);
    void updateConfig();

    void setMotor(byte motor, byte speed, byte direction){ 
      config_changed_ = true;
      _motor[motor].set(speed, direction); 
    };

    void disableMotor(byte motor){ 
      config_changed_ = true;
      _motor[motor].disable();
    };

    void debugConfig() {
      char buff[32];
      for(unsigned int i = DRV89xx_CONFIG_WRITE_START; i < DRV89xx_CONFIG_BYTES; i++) {
        sprintf(buff, "0x%02X: ", i);
        printf("%s\r\n", buff);
        for(int j = 7; j >= 0; j--) printf("%i", _config_cache[i] & (1 << j));
        printf("\r\n");
      }
    };

    
  private:
    GPIOPinPort _cs, _nFault, _nSleep;
    SysClock* _clk;
    SPI_HandleTypeDef* _spi;
    bool begin_called_ = false;
    bool config_changed_ = false;
    byte _config_cache[DRV89xx_CONFIG_BYTES] = {0};  // Fully initalize the config cache as 0
    DRV89xxMotor _motor[DRV89xx_MAX_MOTORS];
    byte _numFaultsOccurred = 0;

    int findMotorWithHB(byte hb1, byte hb2);

    bool _isDRV8192 = false;
};

#endif
