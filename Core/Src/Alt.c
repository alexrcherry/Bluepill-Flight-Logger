/*
 * Alt.c
 *
 *  Created on: Jan 14, 2023
 *      Author: alexr
 */
#include "Alt.h"
#include "main.h"

void enable_alt(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

void disable_alt(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
}

//read prom
void read_prom(SPI_HandleTypeDef *hspi1, uint16_t *prom_array){
	//reset altimiter to load prom values
	enable_alt();
	uint8_t reset_cmd = 0x1E;
	uint8_t status = HAL_SPI_Transmit(hspi1, &reset_cmd, 1, 100);
	HAL_Delay(5);
	disable_alt();

	uint16_t value = 0;

	for (uint8_t i = 0; i < 8; ++i){
		uint8_t address = (0xA0 | ((i) << 1));
		value = 0;

		enable_alt();
		status = HAL_SPI_Transmit(hspi1, &address, 1, 100);
		status = HAL_SPI_Receive(hspi1, &value, 2, 100);
		disable_alt();

		prom_array[i] = value;
	}
}

//read compensated pressure
void read_temp_press(SPI_HandleTypeDef *hspi1, uint16_t *prom, int32_t *temp_press){
	uint8_t D1_OSR_256 = 0x40;
	uint8_t read_ADC = 0x00;
	//read pressure
	uint32_t D1 = 0;
	uint8_t uncomp_pres[3] = {0};
	enable_alt();
	HAL_SPI_Transmit(hspi1, &D1_OSR_256, 1, 10);
	HAL_Delay(1);
	disable_alt();

	enable_alt();
	HAL_SPI_Transmit(hspi1, &read_ADC, 1, 10);
	HAL_SPI_Receive(hspi1, &uncomp_pres, 3, 10);
	disable_alt();

	D1 = ((uint32_t) uncomp_pres[0] << 16) | ((uint32_t) uncomp_pres[1] << 8) | (uint32_t) uncomp_pres[2];

	//read temp
	uint32_t D2 = 0;
	uint8_t D2_OSR_256 = 0x50;
	uint8_t uncomp_temp[3] = {0};
	enable_alt();
	HAL_SPI_Transmit(hspi1, &D2_OSR_256, 1, 10);
	HAL_Delay(1);
	disable_alt();

	enable_alt();
	HAL_SPI_Transmit(hspi1, &read_ADC, 1, 10);
	HAL_SPI_Receive(hspi1, &uncomp_temp, 3, 10);
	disable_alt();

	D2 = ((uint32_t) uncomp_temp[0] << 16) | ((uint32_t) uncomp_temp[1] << 8) | (uint32_t) uncomp_temp[2];

	//compensation
	int32_t dT = D2 - prom[4]*pow(2,8);
	int32_t TEMP = 2000+dT*prom[5]/pow(2,23);
	int64_t OFF = prom[1]*pow(2,17)+(prom[3]*dT)/pow(2,6);
	int64_t SENS = prom[0]*pow(2,16)+(prom[2]*dT)/pow(2,7);
	int32_t P = (D1*SENS/pow(2,21)-OFF)/pow(2,15);

	temp_press[0] = TEMP;
	temp_press[1] = P;
}
//led
void led(uint8_t val){
	if (val == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	}
}
