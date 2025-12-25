#include "stm32g4xx_hal.h"
#include "stm32g474xx.h"
#include "DRV89xxMotor.h"
#include <cstdio>

// #define DEBUG_DRV89xx_MOTORS true

DRV89xxMotor::DRV89xxMotor(byte hb1, byte hb2, byte pwm_channel,
		byte reverse_delay, SysClock *clk) :
		_reverse_delay(reverse_delay), _pwm_channel(pwm_channel), _clk(clk) {
	// populate the half bridge configurations with register and offset settings
	populateHalfbridgeOffsets(0, hb1);
	populateHalfbridgeOffsets(1, hb2);
}

// This function pre-calculates register IDs and bitshift offsets for the selected half-bridge configuration
void DRV89xxMotor::populateHalfbridgeOffsets(byte offset, byte half_bridge) {

	_half_bridge[offset].id = half_bridge;
	if (half_bridge == 0)
		return;

	_half_bridge[offset].enable_register = (byte) DRV89xxRegister::OP_CTRL_1
			+ (half_bridge - 1) / 4; // cache the register address. HB1-4 are OP_CTRL_1, 5-8 are _2, and 9-12 are _3
	_half_bridge[offset].pwm_map_register =
			(byte) DRV89xxRegister::PWM_MAP_CTRL_1 + (half_bridge - 1) / 4; // Same as before, 2 bits per PWM map entry
	_half_bridge[offset].pwm_ctrl_register = (byte) DRV89xxRegister::PWM_CTRL_1
			+ (half_bridge - 1) / 8;  // PWM_CTRL_2 if hb 9+

			// This bitshift is used for PWM ctrl (1 bytes per hb)
	_half_bridge[offset].bitshift_1 = ((half_bridge - 1) % 8); // hb1 = bitshift 0, hb2 = bitshift 1, ..., hb12 = bitshift 4

	// This bitshift is used for PWM map, and ctrl (2 bytes per hb)
	_half_bridge[offset].bitshift_2 = ((half_bridge - 1) % 4) * 2; // hb1 = bitshift 0, hb2 = bitshift 2, ..., hb12 = bitshift 6
}

void DRV89xxMotor::applyConfig(byte *settings) {
	if (_enabled) {
		switch (_direction) { // this section is verbose verbose, but much clearer than the more line-efficient logic would be
		case 1:  // forward
			// printfln("Motor forward");
			if (_clk->millis() - _last_reverse > _reverse_delay) {
				setBridgeLowsideDisablePWM(settings, _half_bridge[0]); // hb1 LOW, no PWM
				setBridgeHSPWM(settings, _half_bridge[1]); // hb2 PWM, high
				setPWMFrequency(settings, _speed); // set PWM duty cycle
				_last_forward = _clk->millis();
			} else {  // brake for _reverse_delay upon direction reversing
				setBridgeLowsideDisablePWM(settings, _half_bridge[0]); // hb1 LOW, no PWM
				setBridgeLowsideDisablePWM(settings, _half_bridge[1]); // hb2 LOW, no PWM
			}

			break;
		case -1: // reverse
			//printfln("Motor reverse");
			if (_clk->millis() - _last_forward > _reverse_delay) {
				setBridgeHSPWM(settings, _half_bridge[0]); // hb1 PWM, high
				setBridgeLowsideDisablePWM(settings, _half_bridge[1]); // hb2 LOW, no PWM
				setPWMFrequency(settings, _speed); // set PWM duty cycle
				_last_reverse = _clk->millis();
			} else {  // brake for _reverse_delay upon direction reversing
				setBridgeLowsideDisablePWM(settings, _half_bridge[0]); // hb1 LOW, no PWM
				setBridgeLowsideDisablePWM(settings, _half_bridge[1]); // hb2 LOW, no PWM
			}

			break;

		default: // brake
			//printfln("Motor braking");
			setBridgeLowsideDisablePWM(settings, _half_bridge[0]); // hb1 LOW, no PWM
			setBridgeLowsideDisablePWM(settings, _half_bridge[1]); // hb2 LOW, no PWM
			break; // lol
		}
	} else { // motor is disabled, set gates to ZZ (free spinning), disable PWM channel for bridges
		setBridgeOpen(settings, _half_bridge[0]);
		setBridgeOpen(settings, _half_bridge[1]);
		//printfln("Motor disabled");
	}
}
void DRV89xxMotor::disable() {
	_enabled = false;
}

void DRV89xxMotor::set(byte speed, int8_t direction) {
	_enabled = true;
	_speed = speed;
	_direction = direction;
}

void DRV89xxMotor::setBridgeLowsideDisablePWM(byte *settings,
		DRV89xxHalfBridge &bridge) {
	if (bridge.id == 0)
		return;  // unconfigured

#ifdef DEBUG_DRV89xx_MOTORS
	printf("HBridge %i\r\n", bridge.id);
	printf("Brake HBridge L: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2);
	printf("Brake HBridge H: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2 + HIGH_SIDE);
	printf("Brake HBridge PWM: 0x%04X clear bit 0x%02X\r\n",
			bridge.pwm_ctrl_register, bridge.bitshift_1);
#endif

	BIT_SET(settings[bridge.enable_register], bridge.bitshift_2); // enable lowside of half bridge
	BIT_CLEAR(settings[bridge.enable_register], bridge.bitshift_2 + HIGH_SIDE); // disable highside of half bridge
	BIT_CLEAR(settings[bridge.pwm_ctrl_register], bridge.bitshift_1); // disable PWM on this half bridge
}

void DRV89xxMotor::setBridgeHSPWM(byte *settings, DRV89xxHalfBridge &bridge) {
	if (bridge.id == 0)
		return;  // unconfigured

#ifdef DEBUG_DRV89xx_MOTORS
	printf("HBridge %i\r\n", bridge.id);
	printf("Brake HBridge L: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2);
	printf("Brake HBridge H: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2 + HIGH_SIDE);
	printf("Brake HBridge PWM: 0x%04X clear bit 0x%02X\r\n",
			bridge.pwm_ctrl_register, bridge.bitshift_1);
	printf("Setting PWM channel: 0x%04X bytes at 0x%02X, 0x%02X: %i\r\n",
			bridge.pwm_map_register, bridge.bitshift_2, bridge.bitshift_2 + 1,
			_pwm_channel);
#endif

	BIT_SET(settings[bridge.enable_register], bridge.bitshift_2 + HIGH_SIDE); // enable highside of half bridge
	BIT_CLEAR(settings[bridge.enable_register], bridge.bitshift_2); // disable low side of half bridge
	BIT_CLEAR(settings[bridge.pwm_ctrl_register], bridge.bitshift_1); // disable PWM on this half bridge
	/*
	BIT_SET(settings[bridge.pwm_ctrl_register], bridge.bitshift_1); // enable PWM on this half bridge
	BIT_WRITE(settings[bridge.pwm_map_register], bridge.bitshift_2,
			(_pwm_channel & 0b1));  // write the low bit
	BIT_WRITE(settings[bridge.pwm_map_register], bridge.bitshift_2 + 1,
			(_pwm_channel & 0b10) >> 1); // write the high bit
	*/
}

void DRV89xxMotor::setBridgeOpen(byte *settings, DRV89xxHalfBridge &bridge) {
	if (bridge.id == 0)
		return;  // unconfigured

#ifdef DEBUG_DRV89xx_MOTORS
	printf("HBridge %i\r\n", bridge.id);
	printf("Brake HBridge L: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2);
	printf("Brake HBridge H: 0x%04X clear bit 0x%02X\r\n",
			bridge.enable_register, bridge.bitshift_2 + HIGH_SIDE);
	printf("Brake HBridge PWM: 0x%04X clear bit 0x%02X\r\n",
			bridge.pwm_ctrl_register, bridge.bitshift_1);
#endif

	BIT_CLEAR(settings[bridge.enable_register], bridge.bitshift_2); // disable lowside of half bridge
	BIT_CLEAR(settings[bridge.enable_register], bridge.bitshift_2 + HIGH_SIDE); // disable highside of half bridge
	BIT_CLEAR(settings[bridge.pwm_ctrl_register], bridge.bitshift_1); // disable PWM on this half bridge
}

void DRV89xxMotor::setPWMFrequency(byte *settings, byte _speed) {
	// printf("Setting PWM channel[");
	// printf(_pwm_channel);
	// printf("]: ");
	// printf(_speed);
	settings[(int) DRV89xxRegister::PWM_DUTY_CTRL_1 + _pwm_channel] = _speed;
}

HalfBridgePair DRV89xxMotor::getHalfBridges() {
	return {_half_bridge[0].id, _half_bridge[1].id};
}
