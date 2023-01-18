/*
 * Alt.h
 *
 *  Created on: Jan 14, 2023
 *      Author: alexr
 */

#ifndef INC_ALT_H_
#define INC_ALT_H_

#include "stm32f1xx_hal.h"

void enable_alt(void);
void disable_alt(void);
void read_prom(SPI_HandleTypeDef *hspi1, uint16_t *prom_array);
void read_temp_press(SPI_HandleTypeDef *hspi1, uint16_t *prom, int32_t *temp_press);
void led(uint8_t val);

#endif /* INC_ALT_H_ */
