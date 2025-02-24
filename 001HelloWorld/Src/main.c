
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>

#define ADC_BASE_ADDR 		0x40012000UL
#define ADC_CR1_REG_OFFSET 	0x04UL
#define ADC_CR1_REG_ADDR 	(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)
#define RCC_BASE_ADDR		0x40023800UL
#define RCC_APB2_ENR_OFFSET 0x44UL
#define RCC_APB2_ENR_ADDR 		(RCC_BASE_ADDR + RCC_APB2_ENR_OFFSET)

int main()
{
	*((volatile uint32_t *) RCC_APB2_ENR_ADDR) |= (1 << 8);

	*((volatile uint32_t *) ADC_CR1_REG_ADDR) |= (1 << 8);

	while(1);
    return 0;
}

