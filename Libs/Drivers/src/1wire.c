#include "1wire.h"


// ����� ��� ������/�������� �� 1-wire
uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//-----------------------------------------------------------------------------
// ������� ����������� ���� ���� � ������, ��� �������� ����� USART
// ow_byte - ����, ������� ���� �������������
// ow_bits - ������ �� �����, �������� �� ����� 8 ����
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// �������� �������������� - �� ����, ��� �������� ����� USART ����� ���������� ����
// ow_bits - ������ �� �����, �������� �� ����� 8 ����
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}






//-----------------------------------------------------------------------------
// ������������� USART � DMA
//-----------------------------------------------------------------------------
void OW_Init(void)
    {
			//USART1 TX A2

      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
      RCC->APB2ENR|= RCC_APB2ENR_USART1EN;
			//Remap USART D
//    SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP;
      
      GPIOA->MODER &= ~(GPIO_MODER_MODER2 );
      GPIOA->MODER |=   GPIO_MODER_MODER2_1 ; 
      GPIOA->OTYPER |= GPIO_OTYPER_OT_2; 
//       GPIOA->OSPEEDR |=   GPIO_OSPEEDR_OSPEEDR2;
       GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR2;
       GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_1;
//      GPIOB->AFR[1] |=(4<<(4*2)) ; // B10
			GPIOA->AFR[0] |=(1<<(4*2)) ; // A2   AF1 << ����� �� 4*1 ���� �����
			
      USART1->BRR =(APB1CLK+BAUD115200/2)/BAUD115200; //
      USART1->CR1 |= USART_CR1_TE |USART_CR1_RE; // 
			USART1->CR3 |=USART_CR3_HDSEL;
      //USART2->CR1 |= USART_CR1_UE; //  
			
			RCC->AHBENR |=RCC_AHBENR_DMA1EN;       //
			DMA1_Channel3->CPAR = (uint32_t) &(USART1->RDR);//RX DMA1_Channel6
			DMA1_Channel3->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel3->CNDTR = 0x08;                //

			DMA1_Channel3->CCR=0;
			DMA1_Channel3->CCR  = 
		                     
													DMA_CCR_MINC
													//DMA_CCR_CIRC|
													//DMA_CCR_DIR
                          ;
		
			DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR); // TX DMA1_Channel7
			DMA1_Channel2->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel2->CNDTR = 0x08;                //

			DMA1_Channel2->CCR=0;
			DMA1_Channel2->CCR  = 
													 
													DMA_CCR_MINC|
													//DMA_CCR_CIRC|
													DMA_CCR_DIR
													 ;
													

    }
//-----------------------------------------------------------------------------
// ������������ ����� � �������� �� ������� ��������� �� ����
//-----------------------------------------------------------------------------		
	uint8_t OW_Reset() {
	uint8_t ow_presence;
		
		USART1->CR1 &= ~USART_CR1_UE; //   USART1  	
		USART1->BRR =(APB1CLK+BAUD9600/2)/BAUD9600; //usart2
		
		USART1->CR1 |= USART_CR1_UE; //   USART1  
			
			
		USART1->ICR|=USART_ICR_TCCF;	
		USART1->TDR= 0xf0;
		
			
		 while(!(USART1->ISR & USART_ISR_TC));
		ow_presence = USART1->RDR;
			
			
		USART1->CR1 &= ~USART_CR1_UE; //   USART1  	
		USART1->BRR =(APB1CLK+BAUD115200/2)/BAUD115200; //usart2
		USART1->CR1 |= USART_CR1_UE; //   USART1  	
	
	if (ow_presence != 0xf0) {
		return OW_OK;
	  }

	return OW_NO_DEVICE;
   }
	
	 
	 
//-----------------------------------------------------------------------------
// ��������� ������� � ����� 1-wire
// sendReset - �������� RESET � ������ �������.
// 		OW_SEND_RESET ��� OW_NO_RESET
// command - ������ ����, ���������� � ����. ���� ����� ������ - ���������� OW_READ_SLOTH
// cLen - ����� ������ ������, ������� ���� ��������� � ����
// data - ���� ��������� ������, �� ������ �� ����� ��� ������
// dLen - ����� ������ ��� ������. ����������� �� ����� ���� �����
// readStart - � ������ ������� �������� �������� ������ (���������� � 0)
//		����� ������� OW_NO_READ, ����� ����� �� �������� data � dLen
//-----------------------------------------------------------------------------
	 
uint8_t OW_Send(uint8_t sendReset, char *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

    DMA1_Channel3->CNDTR = 0x08; 
		DMA1_Channel2->CNDTR = 0x08; 
		
		DMA1_Channel3->CCR  |=  DMA_CCR_EN; 
    DMA1_Channel2->CCR  |=  DMA_CCR_EN;   
		USART1->CR3|=USART_CR3_DMAR | USART_CR3_DMAT;
		
	
    while(!(DMA1->ISR & DMA_ISR_TCIF3)){};//3
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		
		DMA1_Channel3->CCR  &=  ~DMA_CCR_EN; 
    DMA1_Channel2->CCR  &=  ~DMA_CCR_EN;   
		USART1->CR3&= ~(USART_CR3_DMAR | USART_CR3_DMAT);
		

		// 
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

// ���������� ���������. ���������� ��������� ����� ���
void OW_SendBits(uint8_t num_bits) {
	 DMA1_Channel3->CPAR = (uint32_t) &(USART1->RDR);
			DMA1_Channel3->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel3->CNDTR = num_bits;                //

			DMA1_Channel3->CCR=0;
			DMA1_Channel3->CCR  = 
		                     
						DMA_CCR_MINC
						//DMA_CCR_CIRC|
						//DMA_CCR_DIR
                          ;
		
			DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR);
			DMA1_Channel2->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel2->CNDTR = num_bits;                //

			DMA1_Channel2->CCR=0;
			DMA1_Channel2->CCR  = 
													 
						DMA_CCR_MINC|
						//DMA_CCR_CIRC|
						DMA_CCR_DIR;

    DMA1_Channel3->CCR  |=  DMA_CCR_EN; 
    DMA1_Channel2->CCR  |=  DMA_CCR_EN;   
	  USART1->CR3|=USART_CR3_DMAR | USART_CR3_DMAT;
		
	
    while(!(DMA1->ISR & DMA_ISR_TCIF3)){};
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		
	  DMA1_Channel3->CCR  &=  ~DMA_CCR_EN; 
    DMA1_Channel2->CCR  &=  ~DMA_CCR_EN;   
	  USART1->CR3&= ~(USART_CR3_DMAR | USART_CR3_DMAT);

}

//-----------------------------------------------------------------------------
// ������ ������� ������������ ������������ ���� 1-wire � ���������� ���������
//   ID ��������� � ������ buf, �� 8 ���� �� ������ ����������.
// ���������� num ������������ ���������� ��������� ���������, ����� �� �����������
// �����.
//-----------------------------------------------------------------------------
uint8_t OW_Scan(uint8_t *buf, uint8_t num) {

        uint8_t found = 0;
        uint8_t *lastDevice;
        uint8_t *curDevice = buf;
        uint8_t numBit, lastCollision, currentCollision, currentSelection;

        lastCollision = 0;
        while (found < num) {
                numBit = 1;
                currentCollision = 0;

                // �������� ������� �� ����� ���������
                OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, 0, 0, OW_NO_READ);

                for (numBit = 1; numBit <= 64; numBit++) {
                        // ������ ��� ����. �������� � ���������������
                        OW_toBits(OW_READ_SLOT, ow_buf);
                        OW_SendBits(2);

                        if (ow_buf[0] == OW_R_1) {
                                if (ow_buf[1] == OW_R_1) {
                                        // ��� �������, ���-�� ���������� � ����������� �����
                                        return found;
                                } else {
                                        // 10 - �� ������ ����� ������ 1
                                        currentSelection = 1;
                                }
                        } else {
                                if (ow_buf[1] == OW_R_1) {
                                        // 01 - �� ������ ����� ������ 0
                                        currentSelection = 0;
                                } else {
                                        // 00 - ��������
                                        if (numBit < lastCollision) {
                                                // ���� �� ������, �� ����� �� ��������
                                                if (lastDevice[(numBit - 1) >> 3]
                                                                & 1 << ((numBit - 1) & 0x07)) {
                                                        // (numBit-1)>>3 - ����� �����
                                                        // (numBit-1)&0x07 - ����� ���� � �����
                                                        currentSelection = 1;

                                                        // ���� ����� �� ������ �����, ���������� ����� ����
                                                        if (currentCollision < numBit) {
                                                                currentCollision = numBit;
                                                        }
                                                } else {
                                                        currentSelection = 0;
                                                }
                                        } else {
                                                if (numBit == lastCollision) {
                                                        currentSelection = 0;
                                                } else {
                                                        // ���� �� ������ �����
                                                        currentSelection = 1;

                                                        // ���� ����� �� ������ �����, ���������� ����� ����
                                                        if (currentCollision < numBit) {
                                                                currentCollision = numBit;
                                                        }
                                                }
                                        }
                                }
                        }

                        if (currentSelection == 1) {
                                curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
                                OW_toBits(0x01, ow_buf);
                        } else {
                                curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
                                OW_toBits(0x00, ow_buf);
                        }
                        OW_SendBits(1);
                }
                found++;
                lastDevice = curDevice;
                curDevice += 8;
                if (currentCollision == 0)
                        return found;

                lastCollision = currentCollision;
        }

        return found;
}


