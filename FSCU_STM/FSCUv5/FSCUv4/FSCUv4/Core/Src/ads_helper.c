#include "ads_helper.h"

extern SPI_HandleTypeDef hspi3;


uint8_t PTAM_SPITransfer(uint8_t *Tx, uint8_t *Rx, uint32_t len){
	HAL_StatusTypeDef Err = HAL_ERROR;

		if (Tx != NULL && Rx != NULL) {
			Err = HAL_SPI_TransmitReceive(&hspi3, Tx, Rx, len, 100);
		} else if (Tx != NULL) {
			Err = HAL_SPI_TransmitReceive(&hspi3, Tx, noRxRequiredBuff, len, 100);
		} else if (Rx != NULL) {
			Err = HAL_SPI_Receive(&hspi3, Rx, len, 100);
		}

		if (Err == HAL_OK) {
			return ADS131_OK;
		}
		return ADS131_FAILED;
};

void PTAM_syncpin(uint8_t Sig) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, Sig);
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	trig_int++;
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle The Output (LED) Pin
    }
}


void ADS_CSPin(uint8_t cs){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, cs);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, cs);

}
#define ADC_shunt_resistor 1000.f
float ADCmVToBar(float reading){
	float ADC_mA;
	ADC_mA = reading / ADC_shunt_resistor;
	return ADC_mA * (60.f / 16.f);
}

void initADS(void){
	ADS131_SET_FXN_CSPIN(PTAM, ADS_CSPin);
	ADS131_SET_FXN_SYNCPIN(PTAM, PTAM_syncpin);
	ADS131_SET_FXN_SPITRANSFER(PTAM, PTAM_SPITransfer);
	ADS131_SET_FXN_DELAYMS(PTAM, HAL_Delay);
if (ads131_init(&PTAM) != ADS131_OK){
	  wrong++;
};

ads131_set_osr(&PTAM, ADS131_OSR_16384);   // 1kSPS
ads131_set_gain(&PTAM, ADS131_CHANNEL0, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL1, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL2, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL3, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL4, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL5, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL6, ADS131_GAIN_1);
ads131_set_gain(&PTAM, ADS131_CHANNEL7, ADS131_GAIN_1);
}
