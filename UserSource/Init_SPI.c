#include "Init_SPI.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"

    
    #define SPI1_DR_8bit          (*(__IO uint8_t *)((uint32_t)&(SPI1->DR)))


uint8_t SPI1_SendByte(uint8_t data)
{
   uint32_t i; //Объявляем переменную для цикла задержки
// CS_ON;     
   SPI1_DR_8bit = data;                      
        while ((SPI1->SR & SPI_SR_BSY)); 
//   CS_OFF;
//   while (!(SPI1->SR & SPI_SR_RXNE));    
//        for(i=0; i<300; i++);  //Задержка
        return (SPI1_DR_8bit );	
   
}
//==============================================================================
void  SPI1_WR_String(const char *s)
{ int i = 0;
 while (s [i] != 0) {
	SPI1_SendByte (s[i++]);
	}
}
//==============================================================================
uint8_t SPI1_ReadByte(uint8_t byte)
{
//  while (!(SPI1->SR & SPI_SR_TXE));
  SPI1->DR=byte;
  while (!(SPI1->SR & SPI_SR_RXNE));
  return (SPI1->DR);
}

void SPI_ini(void)
{

        //PF7 DC
        // PA15 CS
 RCC->AHBENR |=RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;															 
 GPIOA->MODER &=~ GPIO_MODER_MODER7 | // MOSI
                  GPIO_MODER_MODER5;// |// SCK
//                  GPIO_MODER_MODER4 ; // DC
 GPIOB->MODER &=~ GPIO_MODER_MODER1; //DC
 GPIOA->MODER &=~ GPIO_MODER_MODER6;// | GPIO_MODER_MODER12; // NSS
// GPIOF->MODER &=~ GPIO_MODER_MODER7;
 GPIOA->MODER |=  GPIO_MODER_MODER7_1| // MODER = 10 Alternate Function
                  GPIO_MODER_MODER5_1;// |
                  //GPIO_MODER_MODER4_0 ; //DC out
 GPIOB->MODER |= GPIO_MODER_MODER1_0; //DC
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_0; //DC
	
 GPIOA->MODER |=  GPIO_MODER_MODER6_0;// | GPIO_MODER_MODER12_0; //   //GPIO_MODER_MODER15_0 ; // out
 GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;// | GPIO_OSPEEDER_OSPEEDR12; // high cpeed
 GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_0;// | GPIO_PUPDR_PUPDR12_0; // PUPDR2 = 01  PULLUP   PUPDR2 = 10  PULLDN 
// GPIOF->MODER |=  GPIO_MODER_MODER7_0 ;  //DC
 RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN;
// 000: fPCLK/2
//001: fPCLK/4
//010: fPCLK/8
//011: fPCLK/16
//100: fPCLK/32
//101: fPCLK/64
//110: fPCLK/128
//111: fPCLK/256               
 SPI1->CR1 |= 
			 SPI_CR1_BR_0 |     //011: fPCLK/16
			 SPI_CR1_BR_1 | 
//          SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | 
			 SPI_CR1_BR_2 | //100: fPCLK/32
			 SPI_CR1_MSTR |
			 SPI_CR1_SSI 	|				 
			 SPI_CR1_SSM 
         
//         | SPI_CR1_CPOL
//         | SPI_CR1_CPHA
						 ;	
	SPI1->CR2  = 0x700;   //  8 bit
//	SPI1->CR2 |= SPI_CR2_FRXTH;
	SPI1->CR1 |= SPI_CR1_SPE;	

}

