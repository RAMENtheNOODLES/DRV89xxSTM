#include "stm32g4xx_hal.h"
#include "stm32g474xx.h"
#include "DRV89xx.h"
#include "DRV89xxRegister.h"
#include <cstdio>
#include "error_irq.h"

DRV89xx::DRV89xx(GPIOPinPort cs_pin, GPIOPinPort fault_pin, GPIOPinPort sleep_pin, SPI_HandleTypeDef* hSPI, SysClock* clk) :
		_cs(cs_pin), _nFault(fault_pin), _nSleep(sleep_pin), _spi(hSPI), _clk(clk) {}

void DRV89xx::begin() {
	printf("DRV89xx begin called\r\n");
	if (begin_called_)
		return;  // ignore duplicate begin calls
	begin_called_ = true;
	printf("and run\r\n");

	// SPI Should already be configured correctly

	// Setup pins
//  pinMode(_cs_pin, OUTPUT);
	HAL_GPIO_WritePin(_nSleep.port, _nSleep.pin, GPIO_PIN_SET); // enable chip
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);

	// Configure device
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_1] = 0b11111111; // Disable open load detection on channels 1-8
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_2] = 0b11001111; // Disable errors from open load detection, open load detect on channels 9-12
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_3] = 0b10000000; // set Overcurrent protection to the most forgiving setting
	_config_cache[(int) DRV89xxRegister::SR_CTRL_1] = 0b11111111; // Set slew rate to 2.5us vrs default 0.6us on half bridges (1-8)
	_config_cache[(int) DRV89xxRegister::SR_CTRL_2] = 0b00001111; // Set slew rate to 2.5us vrs default 0.6us on half bridges (9-12)
	_config_cache[(int) DRV89xxRegister::PWM_FREQ_CTRL] = 0xFF; // Set all 4 PWM channels to 2kHz (max speed), TODO: Possibly make this a configurable option?
	// TODO: Possibly make FW_CTRL_1+ an option?
	writeConfig();
}

void DRV89xx::configMotor(byte motor_id, byte hb1, byte hb2, byte pwm_channel,
		byte reverse_delay) {
	config_changed_ = true;
	_motor[motor_id] = DRV89xxMotor(hb1, hb2, pwm_channel, reverse_delay, _clk);
}

byte DRV89xx::writeRegister(byte address, byte value) {

	uint8_t tx[2] = { address & 0x3F, value };
	uint8_t rx[2] = {};
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_spi, tx, rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);
	uint16_t ret = (address << 8) | value;
	HAL_Delay(1);  // Give the chip a chance to write
	return ret;
}

byte DRV89xx::readRegister(byte address) {

	uint8_t tx[2] = { DRV89xx_REGISTER_READ | (address << 8), 0};
	uint8_t rx[2];
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_RESET);
	// uint16_t ret = SPI.transfer16(DRV89xx_REGISTER_READ | (address << 8));
	HAL_SPI_TransmitReceive(_spi, tx, rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);
	return (((uint16_t) rx[0] << 8) | rx[1]) & 0xFF;
}

/**
 * Reads the current error statuses and
 */
void DRV89xx::readErrorStatus(bool print, bool reset) {

	if (HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET) {
		if (print)
			printf("Error detected\r\n");
	} else {
		if (print)
			printf("No DRV errors seen\r\n");
	}

	if (print) {
		printf("Status: 0x%04X\r\n", readRegister(0x00));
		printf("Overcurrent: 0x%04X 0x%04X 0x%04X\r\n", readRegister(0x03), readRegister(0x02), readRegister(0x01));
		printf("Open Load: 0x%04X 0x%04X 0x%04X\r\n", readRegister(0x06), readRegister(0x05), readRegister(0x04));
	}

	if ((HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET) && reset) {
		if (print) {
			printf("Attempting to reset!\r\n");
		}
		_config_cache[(int) DRV89xxRegister::CONFIG_CTRL] = 0b00000001; // clear fault
		writeConfig();  // try writing the config again, just in case
		_config_cache[(int) DRV89xxRegister::CONFIG_CTRL] = 0b00000000; // and go back to normal

		if (HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET) { // still have an error, stop everything
			printf("Cannot reset nFault, resetting everything...\r\n");
			HAL_GPIO_WritePin(_nSleep.port, _nSleep.pin, GPIO_PIN_RESET); // disable chip
			HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET); // disable chip select
			error_irq_trigger(ErrorCodes::DRVError);
		}
	}

	return;
}

void DRV89xx::writeConfig() {
	// Flush the 28 bytes of cache
//	SPI.beginTransaction(_spi_settings);
	for (byte i = DRV89xx_CONFIG_WRITE_START; i < DRV89xx_CONFIG_BYTES; i++) {
		DRV89xx::writeRegister(i, _config_cache[i]);
	}
//	SPI.endTransaction();
}

void DRV89xx::updateConfig() {
	if (!config_changed_)
		return;  // ignore duplicate writes
	config_changed_ = false;
	// Serial.println("Writing config update");

#ifdef DEBUG_DRV89xx_MOTORS
	readErrorStatus(true, true);
#else
	readErrorStatus(false, true);
#endif

	byte i;
	for (i = 0; i < DRV89xx_MAX_MOTORS; i++) {
		_motor[i].applyConfig(_config_cache);
	}
//	SPI.beginTransaction(_spi_settings);
	for (i = DRV89xx_UPDATE_START; i <= DRV89xx_UPDATE_END; i++) {
		DRV89xx::writeRegister(i, _config_cache[i]);
	}
//	SPI.endTransaction();
}
