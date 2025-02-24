/*
 * main.h
 *
 *  Created on: Dec 14, 2024
 *      Author: anhhu
 */

#ifndef MAIN_H_
#define MAIN_H_
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

const uint16_t LEDS = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
const uint16_t LED[4] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};

void init();
void loop();

void delay();


#endif /* MAIN_H_ */
