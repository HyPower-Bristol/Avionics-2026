/*
 * can.c
 *
 *  Created on: Sep 14, 2025
 *      Author: sara.shabbir-khan
 */

#include "can.h"

void InitDefaultCANHeader(CAN_TxHeaderTypeDef *TxHeader){
	TxHeader->ExtId = 0;
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = 4;
	TxHeader->TransmitGlobalTime = DISABLE;

};

void applyHeaderID(CAN_TxHeaderTypeDef *TxHeader, uint16_t CAN_ID){
	TxHeader->StdId=CAN_ID;
};



