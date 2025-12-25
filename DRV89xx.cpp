#include "stm32g4xx_hal.h"
#include "stm32g474xx.h"
#include "DRV89xx.h"
#include "DRV89xxRegister.h"
#include <cstdio>
#include "error_irq.h"

DRV89xx::DRV89xx(GPIOPinPort cs_pin, GPIOPinPort fault_pin,
		GPIOPinPort sleep_pin, SPI_HandleTypeDef *hSPI, SysClock *clk) :
		_cs(cs_pin), _nFault(fault_pin), _nSleep(sleep_pin), _spi(hSPI), _clk(
				clk) {
}

void DRV89xx::begin() {
	printf("\r\nDRV89xx begin called\r\n");
	if (begin_called_)
		return;  // ignore duplicate begin calls
	begin_called_ = true;
	printf("and run\r\n");

	// SPI Should already be configured correctly

	// Setup pins
//  pinMode(_cs_pin, OUTPUT);
	HAL_GPIO_WritePin(_nSleep.port, _nSleep.pin, GPIO_PIN_SET); // enable chip
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);

	byte status = readRegister(0x07);
	_isDRV8192 = (status & 0x70) == 0;

	// Configure device
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_1] = 0b11111111; // Disable open load detection on channels 1-8
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_2] = 0b11001111; // Disable errors from open load detection, open load detect on channels 9-12
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_3] = 0b10010000; // set Overcurrent protection to the most forgiving setting
	_config_cache[(int) DRV89xxRegister::SR_CTRL_1] = 0x00;//0b11111111; // Set slew rate to 2.5us vrs default 0.6us on half bridges (1-8)
	_config_cache[(int) DRV89xxRegister::SR_CTRL_2] = 0x00;//0b00001111; // Set slew rate to 2.5us vrs default 0.6us on half bridges (9-12)
#if IS_DRV8912
	_config_cache[(int) DRV89xxRegister::PWM_FREQ_CTRL] = 0xFF; // Set all 4 PWM channels to 2kHz (max speed), TODO: Possibly make this a configurable option?
#else
	_config_cache[(int) DRV89xxRegister::PWM_FREQ_CTRL_1] = 0xFF; // Set all 4 PWM channels to 2kHz (max speed), TODO: Possibly make this a configurable option?
	_config_cache[(int) DRV89xxRegister::PWM_FREQ_CTRL_2] = 0xFF;
#endif
	_config_cache[(int) DRV89xxRegister::OLD_CTRL_4] = 0x00;
	// TODO: Possibly make FW_CTRL_1+ an option?

	writeConfig();
}

void DRV89xx::configMotor(byte motor_id, byte hb1, byte hb2, byte pwm_channel,
		byte reverse_delay) {
	config_changed_ = true;

	printf("Configuring motor (%i), HB1 (%i), HB2 (%i), PWM (%i), RD (%i)\r\n",
			motor_id, hb1, hb2, pwm_channel, reverse_delay);

	_motor[motor_id] = DRV89xxMotor(hb1, hb2, pwm_channel, reverse_delay, _clk);
}

byte DRV89xx::writeRegister(byte address, byte value) {

	uint8_t tx[2] = { value, (uint8_t) (address & 0x3F) };
	uint8_t rx[2] = { };
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_spi, tx, rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);
	uint16_t ret = ((uint16_t) address << 8) | value;
	HAL_Delay(1);  // Give the chip a chance to write
	return ret;
}

byte DRV89xx::readRegister(byte address) {
	if (address == 0xFF) return 0x00;
	uint8_t tx[2] = { 0x00, (uint8_t) ((address | 0x40) & 0x7F) };
	uint8_t rx[2] = {};
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_RESET);
	// uint16_t ret = SPI.transfer16(DRV89xx_REGISTER_READ | (address << 8));
	HAL_SPI_TransmitReceive(_spi, tx, rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET);
	return (((uint16_t) rx[1] << 8) | rx[0]); //& 0xFF;
}

/**
 * Reads the current error statuses and
 */
void DRV89xx::readErrorStatus(bool print, bool reset) {

	byte statusReg = 0;

	if (HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET) {
		if (print)
			printf("Error detected\r\n");
	} else {
#ifdef DEBUG_DRV89xx_MOTORS
		if (print)
			printf("No DRV errors seen\r\n");
#endif
		return;
	}
	statusReg = readRegister(0x00);
	if (print) {
		printf("Status: 0x%04X\r\n", statusReg);
		printf("Overcurrent: 0x%04X 0x%04X 0x%04X\r\n", readRegister(0x03),
				readRegister(0x02), readRegister(0x01));
		printf("Open Load: 0x%04X 0x%04X 0x%04X\r\n", readRegister(0x06),
				readRegister(0x05), readRegister(0x04));
	}

	if ((HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET)
			&& reset) {
		if (print) {
			printf("Attempting to reset DRV!\r\n");
		}
		_config_cache[(int) DRV89xxRegister::CONFIG_CTRL] = 0b00000001; // clear fault
		writeConfig();  // try writing the config again, just in case
		_config_cache[(int) DRV89xxRegister::CONFIG_CTRL] = 0b00000000; // and go back to normal

		statusReg = readRegister(0x00);

		if (HAL_GPIO_ReadPin(_nFault.port, _nFault.pin) == GPIO_PIN_RESET) { // still have an error, disable troubled motors
			printf("Cannot reset nFault...\r\n");
			HAL_GPIO_WritePin(_cs.port, _cs.pin, GPIO_PIN_SET); // disable chip select

			if (++_numFaultsOccurred > MAX_NUM_FAULTS_ALLOWED) {
				HAL_GPIO_WritePin(_nSleep.port, _nSleep.pin, GPIO_PIN_SET); // put chip to sleep
				printf("Max number of faults occurred\r\n");
				error_irq_trigger(ErrorCodes::DRVError);
			}

			disableErroredMotors(statusReg); // Check for troubled motors; disable them
		}
	}

	return;
}

/**
 * Reads the status register and reads the specific registers
 * and disables troubled motors.
 * Will shutdown everything if fault affects the whole chip
 */
void DRV89xx::disableErroredMotors(byte statusRegister) {
	bool updateCfg = false;

	if (IS_BIT_SET(statusRegister, 4)) { // Overload error; selectively shutdown motors
		printf("Over-load Protection Active... Disabling Troubled Motors...\r\n");
		byte reg1 = readRegister((byte) DRV89xxRegister::OLD_STAT_1);
		byte reg2 = readRegister((byte) DRV89xxRegister::OLD_STAT_2);
		byte reg3 = readRegister((byte) DRV89xxRegister::OLD_STAT_3);

		printf("Reg1: %X, Reg2: %X, Reg3: %X\r\n", reg1, reg2, reg3);

		for (size_t i = 0; i < 8; i++) {
			if (IS_BIT_SET(reg1, i) || IS_BIT_SET(reg1, i + 1)) { // Half-bridge n (1-4)
				printf("Disabling motor #%i\r\n", i);
				disableMotor(i);
				updateCfg = true;
			}
			if (IS_BIT_SET(reg2, i) || IS_BIT_SET(reg2, i + 1)) { // Half-bridge n (5-8)
				printf("Disabling motor #%i\r\n", i);
				disableMotor(i + 8);
				updateCfg = true;
			}
			if (IS_BIT_SET(reg3, i) || IS_BIT_SET(reg3, i + 1)) { // Half-bridge n (9-12)
				printf("Disabling motor #%i\r\n", i);
				disableMotor(i + 16);
				updateCfg = true;
			}
		}
	}
	if (IS_BIT_SET(statusRegister, 3)) { // Over-current error; selectively shutdown motors
		printf("Over-current Protection Active... Disabling Troubled Motors...\r\n");
		byte reg1 = readRegister((byte) DRV89xxRegister::OCP_STAT_1);
		byte reg2 = readRegister((byte) DRV89xxRegister::OCP_STAT_1);
		byte reg3 = readRegister((byte) DRV89xxRegister::OCP_STAT_1);

		printf("Reg1: %X, Reg2: %X, Reg3: %X\r\n", reg1, reg2, reg3);

		for (size_t i = 0; i < 8; i++) {
			if (IS_BIT_SET(reg1, i) || IS_BIT_SET(reg1, i + 1)) { // Half-bridge n (1-4)
				int motorNum = findMotorWithHB(i + 1, i + 2);
				printf("Disabling motor #%i\r\n", motorNum);
				disableMotor(motorNum);
				updateCfg = true;
			}
			if (IS_BIT_SET(reg2, i) || IS_BIT_SET(reg2, i + 1)) { // Half-bridge n (5-8)
				int motorNum = findMotorWithHB(i + 8, i + 9);
				printf("Disabling motor #%i\r\n", motorNum);
				disableMotor(motorNum);
				updateCfg = true;
			}
			if (IS_BIT_SET(reg3, i) || IS_BIT_SET(reg3, i + 1)) { // Half-bridge n (9-12)
				int motorNum = findMotorWithHB(i + 16, i + 17);
				printf("Disabling motor #%i\r\n", motorNum);
				disableMotor(motorNum);
				updateCfg = true;
			}
		}
	}

	if (updateCfg) updateConfig(false);

	if (IS_BIT_SET(statusRegister, 6)) // Over temp error; shut everything down
		error_irq_trigger(ErrorCodes::DRVOTSDError);
	if (IS_BIT_SET(statusRegister, 2)) // Under-voltage error; shut everything down
		error_irq_trigger(ErrorCodes::DRVUVLOError);
	if (IS_BIT_SET(statusRegister, 1)) // Over-voltage error; shut everything down
		error_irq_trigger(ErrorCodes::DRVOVPError);

	if (IS_BIT_SET(statusRegister, 0)) // No Power on Reset detected
		return;
	if (IS_BIT_SET(statusRegister, 7)) // reserved
		return;
	// Unknown error; shut everything down
	HAL_GPIO_WritePin(_nSleep.port, _nSleep.pin, GPIO_PIN_RESET); // disable chip
	printf("Unknown status: %X\r\n", statusRegister);
	error_irq_trigger(ErrorCodes::DRVError);

	return;
}

void DRV89xx::writeConfig() {
	// Flush the 28 bytes of cache
	for (byte i = DRV89xx_CONFIG_WRITE_START; i < DRV89xx_CONFIG_BYTES; i++) {
		DRV89xx::writeRegister(i, _config_cache[i]);
	}
}

void DRV89xx::updateConfig(bool errorCheck) {
	if (!config_changed_)
		return;  // ignore duplicate writes
	config_changed_ = false;

	if (errorCheck) readErrorStatus(true, true);

	byte i;
	for (i = 0; i < DRV89xx_MAX_MOTORS; i++) {
		_motor[i].applyConfig(_config_cache);
	}
	for (i = DRV89xx_UPDATE_START; i <= DRV89xx_UPDATE_END; i++) {
		DRV89xx::writeRegister(i, _config_cache[i]);
	}
}

void DRV89xx::updateConfig() {
	updateConfig(true);
}

/**
 * Attempts to find the motor using the two half-bridges; Returns the first motor
 * index using HB1 if it doesn't use both.
 * Returns -1 if it can't find a motor using either half-bridges
 */
int DRV89xx::findMotorWithHB(byte hb1, byte hb2) {
	int out = -1;

	for (byte i = 0; i < DRV89xx_MAX_MOTORS; i++) {
		HalfBridgePair hbp = _motor[i].getHalfBridges();

		if (hbp.HB1 == hb1 || hbp.HB2 == hb2) { // either partial or exact match
			out = i;
			break;
		}
	}

	return out;
}
