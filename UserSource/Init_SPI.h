#ifndef Init_SPI_H
#define Init_SPI_H

#include "stm32f0xx.h" 

#define CS_ON    GPIO_ResetBits(GPIOA, GPIO_Pin_6) //GPIOA->BSRR |= GPIO_BSRR_BR4;// //
#define CS_OFF    GPIO_SetBits(GPIOA, GPIO_Pin_6)//GPIOA->BSRR |= GPIO_BSRR_BS4;//  
#define CS2_ON    GPIOA->BSRR |= GPIO_BSRR_BR_12; //GPIOA->BSRR |= GPIO_BSRR_BR4;// //
#define CS2_OFF   GPIOA->BSRR |= GPIO_BSRR_BS_12;//GPIOA->BSRR |= GPIO_BSRR_BS4;//  
#define LATCH_ON  GPIOB->BSRR |= GPIO_BSRR_BR_1;
#define LATCH_OFF  GPIOB->BSRR |= GPIO_BSRR_BS_1;


//void SPI1_SendByte(uint8_t byte);
uint8_t SPI1_SendByte(uint8_t byte);
void  SPI1_WR_String(const char *);
uint8_t SPI1_ReadByte(uint8_t byte);
void SPI_ini(void);

#endif

