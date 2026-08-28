/*
 * ads_helper.h
 *
 *  Created on: Sep 14, 2025
 *      Author: sara.shabbir-khan
 */

#ifndef INC_ADS_HELPER_H_
#define INC_ADS_HELPER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "../../ads131m0x/ads131m0x.h"
#include "main.h"

void initADS(void);
uint8_t PTAM_SPITransfer(uint8_t *Tx, uint8_t *Rx, uint32_t len);
float ADCmVToBar(float reading);


#endif /* INC_ADS_HELPER_H_ */
