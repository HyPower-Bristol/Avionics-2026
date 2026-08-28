/*
 * can.h
 *
 *  Created on: Sep 14, 2025
 *      Author: sara.shabbir-khan
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

void InitDefaultCANHeader(CAN_TxHeaderTypeDef *TxHeader);
void applyHeaderID(CAN_TxHeaderTypeDef *TxHeader, uint16_t CAN_ID);


#endif /* INC_CAN_H_ */
