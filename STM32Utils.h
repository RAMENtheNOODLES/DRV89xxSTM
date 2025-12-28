/*
 * STM32Utils.h
 *
 *  Created on: Dec 23, 2025
 *      Author: rommelfangerc
 */

#ifndef STM32UTILS_H_
#define STM32UTILS_H_

//#define DEBUG_DRV89xx_MOTORS

#include <cstdint>

class SysClock {
public:
	SysClock(uint64_t* sysUptimeMS) : _sysUptimeMS(sysUptimeMS) {}

	uint64_t millis() {
		return *_sysUptimeMS;
	}

private:
	uint64_t* _sysUptimeMS;
};

#endif /* STM32UTILS_H_ */
