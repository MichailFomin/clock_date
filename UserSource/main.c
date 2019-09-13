/*
A2 - UART_TX DS18B20
A5 - CLK
A7 - DATA
B1 - LATCH
*/

#include "stm32f0xx.h" 
#include "Init_I2C.h"
//#include "SSD1306_I2C.h"
//#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include <math.h>
//#include "1wire.h"
#include "stm32f0xx_gpio.h"
#include "Init_SPI.h"
//#include "SSD1306_SPI.h"
#include "SSD1351.h"
#include "1wire.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_flash.h"
//#include "stm32f0xx_adc.h"
//#include "stm32f0xx_i2c.h"

#include "fonts.h"
//#include "fonts_Pepsi.h"
#include "stm32f0xx_misc.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"



//#define MCP23017_I2C_ADDR1        0x20
#define V_REF 3.34
#define MINUS 64
#define PLUS 128

//#define EEPROM_OWN_ADDRESS (0x50)
//#define ssd1306_I2C_TIMEOUT					5000

#define DS3231_I2C_TIMEOUT					1000
uint16_t I2C_Timout_DS3231 = 0;
uint8_t Timeout_DS3231 = 0;

#define DS3231_addr     0x68 //0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB

//#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_minutes  0x01
#define DS3231_hours    0x02
#define DS3231_day      0x03
#define DS3231_date     0x04
#define DS3231_month    0x05
#define DS3231_year     0x06
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB


// All DS3231 registers
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t alarm1_secconds;
	uint8_t alarm1_minutes;
	uint8_t alarm1_hours;
	uint8_t alarm1_day;
	uint8_t alarm1_date;
	uint8_t alarm2_minutes;
	uint8_t alarm2_hours;
	uint8_t alarm2_day;
	uint8_t alarm2_date;
	uint8_t control;
	uint8_t status;
	uint8_t aging;
	uint8_t msb_temp;
	uint8_t lsb_temp;
} DS3231_registers_TypeDef;

// DS3231 date
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_of_week;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} DS3231_date_TypeDef;

// Human Readable Format date
typedef struct {
	uint8_t  Seconds;
	uint8_t  Minutes;
	uint8_t  Hours;
	uint8_t  Day;
	uint8_t  Month;
	uint16_t Year;
	uint8_t  DOW;
} HRF_date_TypeDef;

HRF_date_TypeDef date;

// Структура для Меню
//typedef struct {
struct Settings {
	uint8_t  	Imp;
	float  		m_sens; // сколько метров за 1 оборот датчика скорости
	uint16_t  TAH1;
	uint16_t  TAH2;
	uint16_t  TAH3;
	uint8_t  	Light_mode;
	uint16_t  Speed_max;
	uint8_t  	SP_Test_On_Off;
	uint16_t	speed_test_time;
	uint16_t	sp_test_delay_ms;
  float 		SP_Scale; //масштабирование шкалы для узменения макс угла отклонения стрелки
  float 		Frq_display_SP; // частота отображения скорости на спидометре, на дисплее;
	uint8_t		Tah_imp;
  uint16_t  TAH_max;
	uint8_t  	TAH_Test_On_Off;
	uint16_t	Tah_test_time;
	uint16_t	Tah_test_delay_ms;
	
	float 		TAH_Scale; //масштабирование шкалы для узменения макс угла отклонения стрелки
  float 		Frq_display_TAH; // частота отображения оборотов на тахометре, на дисплее;
	uint8_t 	Display_Num; // кол-во дисплеев
	float 		Callibrate_V1;
	uint8_t		Acceleration_60;
	uint8_t 	Acceleration_100;
	uint16_t	Acceleration_X;
	uint8_t		Temp_sensor;
	uint8_t		Presure_sensor;
	uint8_t		Fuel_sensor;
	uint16_t	Soft_light_time;
   uint16_t generate_SP;
   uint16_t generate_TAH;
   uint16_t LED_max;
   uint16_t LED_min;
   uint16_t Strelki_time;
	
};   

//Menu_TypeDef Settings;
struct Settings Set_Menu;
struct Settings *pointerSettings = &Set_Menu;

uint16_t    sec,
            msec,
						disp,
            msec20,
            frame,frame_max,
            ACP[6],
            min,
            hour,
            graph_msec,
            timer_DS,
            timer_button1,
            timer_button_UP_temp,
            timer_button_UP,
            timer_button_DN_temp,
            timer_button_DN,
            timer_button,
            i_ssd,
            
            tim_eep
            ;
         
uint8_t     
            ADC_calibrate,
				i_cal,
				i2c_count,
				bit_sec,
            temp_HC = 0,
            s = 1,
						HC1,
						HC2,
            minus1,
            nul,
            Disp01,
            Disp1,
            Disp2,
            Disp3,
            Disp4,
            Disp5,
            Disp6,
            Disp7,
            Dig[10] = {	1+2+4+8+16+32, 
						16+32,
						1+4+8+32+64,
						1+8+16+32+64,
						2+16+32+64,
						1+2+8+16+64,
						1+2+4+8+16+64,
						1+16+32,
						127,
						123
						},
            clear,
            GRAPH_Buff[128],
            Grid[128],
            g,g_t,h,
            i[6],
            buffer[20],
            mesure,
            ROM[16],
            temp_m,
            temp_h,
            sw_state_m_UP,
            sw_state_m_DN,
            sw_state_h_UP,
            sw_state_h_DN,
            set_time[7],
            buf[2],
            OW_err,
            temp2,
            temp1,
            temp3,
            temp4,
            enter_level_menu,
            start_timer_button_UP,
            start_timer_button_DN,
            sw_state,
            sw_state2,
            start_timer_button1,
            enter_menu,
            screen = 1,
            start_timer_button,
            Out,
            sw_state3,
            
            bit = 0,
            out_A = 0xFF
            ;
char        str[25],str2[25],str3[25],str4[25],str5[25],str6[25],str7[25],charset,number = 0,txt[80];

float       Temp1,
            Temp2,
            R2,
            ADC_sum[6],
            ADC_RMS[6],
            ADC_temp[6],
            ADC_data[6],
            ADC_chanel[6],
            ADC_chanel_avr_tmp[6],
            ADC_chanel_avr[6],
            mV_chanel[6],
            TEMP_V[6],
            TOK[6],
            VOLT[2],
            
            
            T_sr[2],
            T[2]
            
//            
            ;

uint32_t    num_sample = 0,
            max_num_sample = 0,
            rms_sample = 0,
            max_rms_sample = 0;
//uint16_t I2C_Timout = 0;
//uint8_t Timeout = 0;


void Delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for (; nCount!=0; nCount--);
}

void Delay_us(uint32_t us)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000000)*us;
        for (; nCount!=0; nCount--);
}




void DS3231_write(uint16_t address, uint8_t data)
{
   
   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((1+1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=address;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = data; //*(buf++);	// Отправляю данныеCount++
	}
   

//    //стоп
//   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
//	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
//	// Очищаю флаги - необходимо для дальнейшей работы шины
//	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
//	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
//	// Если есть ошибки на шине - очищаю флаги
//	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
//	{
//		I2C1->ICR |= I2C_ICR_ARLOCF;
//		I2C1->ICR |= I2C_ICR_BERRCF;
//	}
   //-------------------------------------
	
//	I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | (DS3231_addr); 
//		
//			/* Check Tx empty */
//	  while (!(I2C1->ISR & I2C_ISR_TXE) );
//	
//	 I2C1->TXDR =(uint8_t) (address); /* Byte to send */
//	 I2C1->CR2 |= I2C_CR2_START; /* Go */
//		
//      //while ((!(I2C1->ISR & I2C_ISR_TXIS) )
//		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
//      {
//         I2C_Timout_DS3231++;
//         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//      };
//      Timeout_DS3231 = 0;
//      I2C_Timout_DS3231 = 0;;
////		I2C1->TXDR = (uint8_t)(address &0x00FF); /* Byte to send */
////	
////		//while (!(I2C1->ISR & I2C_ISR_TXIS) );
////		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout == 0) )
////      {
////         I2C_Timout++;
////         if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
////      };
////      Timeout = 0;
////      I2C_Timout = 0;
//      
//		I2C1->TXDR = data ; /* Byte to send */
	 	
}

void DS3231_Init(uint16_t address, uint8_t data)
{
	
	I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | (DS3231_addr<<1); 
		
			/* Check Tx empty */
	  while (!(I2C1->ISR & I2C_ISR_TXE) );
	 I2C1->TXDR =(uint8_t)(address);
//	 I2C1->TXDR =(uint8_t)(address &0x00FF);; /* Byte to send */
	 I2C1->CR2 |= I2C_CR2_START; /* Go */
		
      //while ((!(I2C1->ISR & I2C_ISR_TXIS) )
		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
      {
         I2C_Timout_DS3231++;
         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
      };
      Timeout_DS3231 = 0;
      I2C_Timout_DS3231 = 0;;
		I2C1->TXDR = 0x7c; /* Byte to send */
      
		//while (!(I2C1->ISR & I2C_ISR_TXIS) );
		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
      {
         I2C_Timout_DS3231++;
         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
      };
      Timeout_DS3231 = 0;
      I2C_Timout_DS3231 = 0;
      
//		I2C1->TXDR = 0x08 ; /* Byte to send */
      
      //стоп
//   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
//	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
//	// Очищаю флаги - необходимо для дальнейшей работы шины
//	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
//	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
//	// Если есть ошибки на шине - очищаю флаги
//	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
//	{
//		I2C1->ICR |= I2C_ICR_ARLOCF;
//		I2C1->ICR |= I2C_ICR_BERRCF;
//	}
	 	
}

void DS3231_Read(uint16_t address,uint8_t* buff,uint8_t nbytes)
{
   
   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;  //авто-СТОП отключаем
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
      // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
		
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = address;	// Отправляю адрес регистра
	}
	 i2c_count = 0;
   // Повторный старт
   I2C1->CR2 |= I2C_CR2_RD_WRN;	// Режим приёма
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((nbytes)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
      // Принимаем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, приём прекращаем.
		buffer[i2c_count] = I2C1->RXDR;
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_RXNE) 
		{
			buffer[i2c_count] = I2C1->RXDR; //*(buff++) = I2C1->RXDR;	// Принимаю данныеCount++
			i2c_count++;
		}
	}
//   //стоп
   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
   
   
	//---------------------------------
//	uint8_t index=0;
//	I2C1->CR2 =  (1<<16) | (DS3231_addr);//пишем адрес устройства 
////	 while (!(I2C1->ISR & I2C_ISR_TXE) );
//   while ((!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout_DS3231 == 0) )
//   {
//      I2C_Timout_DS3231++;
//      if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//   };
//   Timeout_DS3231 = 0;
//   I2C_Timout_DS3231 = 0;
//   I2C1->CR2 |= I2C_CR2_START; /* Go */
//	I2C1->TXDR = (uint8_t) (address); /* Byte to send (address>>8); */
//	
////		while (!(I2C1->ISR & I2C_ISR_TXIS) );
//   while ((!(I2C1->ISR & I2C_ISR_TC) ) && (Timeout_DS3231 == 0) ) //I2C_ISR_TXIS
//   {
//      I2C_Timout_DS3231++;
//      if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//   };
//   Timeout_DS3231 = 0;
//   I2C_Timout_DS3231 = 0;
//	 I2C1->CR2 =  //I2C_CR2_AUTOEND |
//            		 ((nbytes)<<16) | 
//		            (DS3231_addr) |
//		             I2C_CR2_RD_WRN |
//		             I2C_CR2_NACK; 
//	 I2C1->CR2 |= I2C_CR2_START; /* Go */
//		for(index=0;index<=nbytes-1;index++)
//	 { 
////	 while (!(I2C1->ISR & I2C_ISR_RXNE) ){};
//       while ((!(I2C1->ISR & I2C_ISR_RXNE) ) && (Timeout_DS3231 == 0) )
//       {
//          I2C_Timout_DS3231++;
//          if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//       };
//       Timeout_DS3231 = 0;
//       I2C_Timout_DS3231 = 0;
//       buf[index]  = I2C1->RXDR ;
//	
//	 }
//    
//    //стоп
//   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
////	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
//	// Очищаю флаги - необходимо для дальнейшей работы шины
//	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
//	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
//	// Если есть ошибки на шине - очищаю флаги
//	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
//	{
//		I2C1->ICR |= I2C_ICR_ARLOCF;
//		I2C1->ICR |= I2C_ICR_BERRCF;
//	}
    
}





void DS3231_WriteDate(uint8_t* buf, uint8_t nbytes)
{
   uint8_t temp,i;
	if (nbytes) //шлем и секунды
	{
		for (i = 0;i <= 6;i++)
    {
      temp = (buf[i]/10) << 4 | (buf[i]%10);
      DS3231_write(i, temp);
    }
	}
	else // то шлем без секунд
	{
		for (i = 1;i <= 6;i++)
    {
      temp = (buf[i]/10) << 4 | (buf[i]%10);
      DS3231_write(i, temp);
    }
	}
		
   
}

void DS3231_ReadDate(uint8_t* buff) { //HRF_date_TypeDef* hrf_date
//	DS3231_date_TypeDef raw_date;
//   DS3231_registers_TypeDef raw_date;
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
   DS3231_Read(0,buff,18);//тут біло 19 вместо 18 и сбивало DS18B20 ROM-code
//	DS3231_ReadDateRAW(&raw_date);

	date.Seconds = (buffer[0] >> 4) * 10 + (buffer[0] & 0x0f);
	date.Minutes = (buffer[1] >> 4) * 10 + (buffer[1] & 0x0f);
  date.Hours   = (buffer[2] >> 4) * 10 + (buffer[2] & 0x0f);
	date.DOW     = (buffer[3] >> 4) * 10 + (buffer[3] & 0x0f);
  date.Day     = (buffer[4] >> 4) * 10 + (buffer[4] & 0x0f);
	date.Month   = (buffer[5] >> 4) * 10 + (buffer[5] & 0x0f);
	date.Year    = (buffer[6] >> 4) * 10 + (buffer[6] & 0x0f) + 2000;
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
//	hrf_date->Hours   = (raw_date.hours   >> 4) * 10 + (raw_date.hours   & 0x0f);
//	hrf_date->Day     = (raw_date.date    >> 4) * 10 + (raw_date.date    & 0x0f);
//	hrf_date->Month   = (raw_date.month   >> 4) * 10 + (raw_date.month   & 0x0f);
//	hrf_date->Year    = (raw_date.year    >> 4) * 10 + (raw_date.year    & 0x0f) + 2000;
//	hrf_date->DOW     = raw_date.day_of_week;
}



void SetSysClockToHSI48(void)
{
   uint32_t StartUpCounter = 0, HSIStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
//  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSIStatus = RCC->CR & RCC_CR_HSIRDY;
    StartUpCounter++;  
  } while((HSIStatus == 0) && (StartUpCounter != HSI_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
  {
    HSIStatus = (uint32_t)0x01;
  }
  else
  {
    HSIStatus = (uint32_t)0x00;
  }  

  if (HSIStatus == (uint32_t)0x01)
  {

    /* Enable Prefetch Buffer */
    //FLASH->ACR |= FLASH_ACR_PRFTBE;
		FLASH_PrefetchBufferCmd(ENABLE);
		
    /* Flash 0 wait state */
//    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
//    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;    
FLASH->ACR |= (uint32_t)FLASH_Latency_1;
//	FLASH_SetLatency(FLASH_Latency_1);
//    /* HCLK = SYSCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
//      
//    /* PCLK2 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
//    
//    /* PCLK1 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
    
		/*  PLL configuration:  = (HSI / 2) * 12 = 48 MHz */
//		RCC_PREDIV1Config(RCC_PREDIV1_Div1);
//		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12); //RCC_PLLMul_12
/*  PLL configuration:  = (HSE) * 6 = 48 MHz */
		RCC_PREDIV1Config(RCC_PLLSource_HSI_Div2);
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12); //RCC_PLLMul_12

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}
void Init_PORT(void)
{
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_GPIOB, ENABLE);
	//Input BUTTON
//00: Input mode (reset state)
//01: General purpose output mode
//10: Alternate function mode
//11: Analog mode   
   GPIOA->MODER &= ~GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4;
//   0: Output push-pull (reset state)
//   1: Output open-drain
   GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1  | GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_4;
   GPIOA->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1 | GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_4;
   //Output open-drain
//   00: No pull-up, pull-down
//   01: Pull-up
//   10: Pull-down
//   11: Reserved
   GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4;
   //   01: Pull-up               
   GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0;
   //x0: Low speed
//01: Medium speed
//11: High speed
//reset OSPEEDR

//OUT HC595 LATCH
   GPIOB->MODER |= (GPIO_MODER_MODER1_0);
   GPIOB->OTYPER &= ~GPIO_OTYPER_OT_1;
   GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR1);
   GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPDR1);
	 GPIOB->PUPDR |= ( GPIO_PUPDR_PUPDR1_0);

}


void Init_OUT(void)
{
	//OUT HC595 LATCH
   GPIOA->MODER |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0);
   GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7);
   GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR7);
   GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR7);
	 GPIOA->PUPDR |= ( GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR7_0);

	
	//OUT HC595 LATCH
   GPIOB->MODER |= (GPIO_MODER_MODER1_0);
   GPIOB->OTYPER &= ~GPIO_OTYPER_OT_1;
   GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR1);
   GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPDR1);
	 GPIOB->PUPDR |= ( GPIO_PUPDR_PUPDR1_0);

}



void HC595(uint8_t N_Disp,uint8_t Dot)
{
   CS_ON;
   //SPI1_SendByte(temp_HC);
//   SPI1_SendByte(0);//4+2+1
   SPI1_SendByte(0);//6+2 numbers
   SPI1_SendByte(0);//segments
   LATCH_ON;
   LATCH_OFF;
   CS_OFF;
   
   
   CS_ON;
	
	switch(N_Disp)
   {
      case 1://temper
//         SPI1_SendByte(0);//4+2+1
//         if ( (T_sr[0] < 10) && (T_sr[0] > -10) ) SPI1_SendByte(128); else SPI1_SendByte(64);//6+2 numbers
				 if (T_sr[0] >= 1) {SPI1_SendByte(32 | MINUS | PLUS);}
				 else {if (T_sr[0] <= -1) SPI1_SendByte(32 | MINUS);}
				 
				 if ((T_sr[0] < 1) && (T_sr[0] > -1)) SPI1_SendByte(32);
				
         SPI1_SendByte(Dig[Disp1]);
         break;
      case 2://temper
//         SPI1_SendByte(0);//4+2+1
         if ( (T_sr[0] < 10) && (T_sr[0] > -10) ) SPI1_SendByte(0); else SPI1_SendByte(16);//6+2 numbers
         SPI1_SendByte(Dig[Disp2]);
         break;
      case 3://sec
//         SPI1_SendByte(0);//4+2+1
         SPI1_SendByte(8);//6+2 numbers     if (date.Seconds % 2) SPI1_SendByte(0); else 
         SPI1_SendByte(Dig[Disp6] | Dot);
         break;
      case 4://sec
//         SPI1_SendByte(0);//4+2+1
					SPI1_SendByte(4);
//         if (date.Seconds < 10) SPI1_SendByte(0); else SPI1_SendByte(8); //6+2 numbers      8
         SPI1_SendByte(Dig[Disp7] | Dot);
         break;
      case 5://minut
//         SPI1_SendByte(0);//4+2+1
         SPI1_SendByte(2);//6+2 numbers
         SPI1_SendByte(Dig[Disp4] );
         break;
      case 6://minut
//         SPI1_SendByte(0);//4+2+1
//         if (nul) 
//         {
////            if (minus1) 
////            {
//               if (date.Minutes < 10) SPI1_SendByte(0);else SPI1_SendByte(32);
////            }
////            else
////            {
////               
////            }
//             
//         }
//         else
//            if (minus1) 
//            {
//               if (date.Minutes < 10) SPI1_SendByte(64); else  SPI1_SendByte(32+64);
//            } 
//            else 
//               if (date.Minutes < 10) SPI1_SendByte(128); else SPI1_SendByte(32+128);//6+2 numbers
				SPI1_SendByte(1);
         SPI1_SendByte(Dig[Disp5] );
         break;
      
   }
	
 /*  switch(N_Disp)
   {
      case 1:
//         SPI1_SendByte(0);//4+2+1
         if ( (T_sr[0] < 10) && (T_sr[0] > -10) ) SPI1_SendByte(0); else SPI1_SendByte(1);//6+2 numbers
         SPI1_SendByte(Dig[Disp2] | Dot);
         break;
      case 2:
//         SPI1_SendByte(0);//4+2+1
         SPI1_SendByte(2);//6+2 numbers
         SPI1_SendByte(Dig[Disp1] | Dot);
         break;
      case 3:
//         SPI1_SendByte(0);//4+2+1
         SPI1_SendByte(4);//6+2 numbers     if (date.Seconds % 2) SPI1_SendByte(0); else 
         SPI1_SendByte(Dig[Disp6] );
         break;
      case 4:
//         SPI1_SendByte(0);//4+2+1
         if (date.Seconds < 10) SPI1_SendByte(0); else SPI1_SendByte(8); //6+2 numbers      8
         SPI1_SendByte(Dig[Disp7] );
         break;
      case 5:
//         SPI1_SendByte(0);//4+2+1
         SPI1_SendByte(16);//6+2 numbers
         SPI1_SendByte(Dig[Disp4] );
         break;
      case 6:
//         SPI1_SendByte(0);//4+2+1
         if (nul) 
         {
//            if (minus1) 
//            {
               if (date.Minutes < 10) SPI1_SendByte(0);else SPI1_SendByte(32);
//            }
//            else
//            {
//               
//            }
             
         }
         else
            if (minus1) 
            {
               if (date.Minutes < 10) SPI1_SendByte(64); else  SPI1_SendByte(32+64);
            } 
            else 
               if (date.Minutes < 10) SPI1_SendByte(128); else SPI1_SendByte(32+128);//6+2 numbers
         SPI1_SendByte(Dig[Disp5] );
         break;
      
   }*/
//   SPI1_SendByte(Dig[Seg]);
   LATCH_ON;
   LATCH_OFF;
   CS_OFF;
   
   
   
//   CS_ON;
//   //SPI1_SendByte(temp_HC);
//   SPI1_SendByte(0);//4+2+1
//   SPI1_SendByte(1);//6+2 numbers
//   SPI1_SendByte(Dig[0]);//segments
//   LATCH_ON;
//   LATCH_OFF;
//   CS_OFF;
//   
//   CS_ON;
//   //SPI1_SendByte(temp_HC);
//   SPI1_SendByte(0);//4+2+1
//   SPI1_SendByte(0);//6+2 numbers
//   SPI1_SendByte(0);//segments
//   LATCH_ON;
//   LATCH_OFF;
//   CS_OFF;
//   
//   CS_ON;
//   //SPI1_SendByte(temp_HC);
//   SPI1_SendByte(0);//4+2+1
//   SPI1_SendByte(4);//6+2 numbers
//   SPI1_SendByte(Dig[1]);//segments
//   LATCH_ON;
//   LATCH_OFF;
//   CS_OFF;
   
}

void SysTick_Handler (void) 
{
   msec++;
   graph_msec++;
   disp++;
   msec20++;
   
      
      
   if (msec == 1000)
   {
      msec = 0;
      frame_max = frame;
      frame = 0;
      sec++;
//      bit ^= 1;
      
      if (sec == 60)
      {
         sec = 0;
//         min++;
//         if (min == 60)
//         {
//            min = 0;
//            hour++;
//            if (hour == 999) hour = 0;
//         }
      }
   }
   
   if (graph_msec == 500)
   {
      graph_msec = 0;
      bit_sec ^= 1;
//		 T_sr[1] -= 0.1;
		 
      
   }
//	 s++;
//   if ( (s > 6) || (s < 1) ) s = 1;
	 //частота индикации цыфр
	 if (disp > 1)
	 {
		  s++;
			if ( (s > 6) || (s < 1) ) s = 1;
		  disp = 0;
	 }
	 if (bit_sec) HC595(s,0); else HC595(s,128);
//   if (date.Seconds % 2) HC595(s,0); else HC595(s,128);
   
   
   //Для датчика температуры
   if (mesure == 1) timer_DS++;
	 if (timer_DS == 950)
   {
		 
      mesure = 2;
      timer_DS = 0;
   }	
   //////////*******//////
   
   if (msec20 == 100) 
   {
//      Math_ADC(0);
//      Math_ADC(1);
//			Math_ADC(2);
//      Math_ADC(3);
//			Math_ADC(4);
//      Math_ADC(5);
      msec20 = 0;
//      Flag_math = 1;
      max_rms_sample = rms_sample;
      rms_sample = 0;
		 DS3231_ReadDate(buffer);
   }
   
   
   
   
//   if (enter_level_menu) 
//	{
		//чтобы при зажатой кнопке ВВЕРХ увеличивалось значение автоматически
//		if ( (start_timer_button_UP) && (timer_button_UP_temp < 1200) )  timer_button_UP_temp++;
//		if (timer_button_UP_temp == 1200)
//		{
//			
//			timer_button_UP++;
//			if (timer_button_UP == 50)
//			{
//				timer_button_UP = 0;
//				sw_state ^=1;
//			}
//			
//			
//		}
//		
//		///////////////
//		//чтобы при зажатой кнопке ВНИЗ увеличивалось значение автоматически
//		if ( (start_timer_button_DN) && (timer_button_DN_temp < 1200) )  timer_button_DN_temp++;
//		if (timer_button_DN_temp == 1200)
//		{
//			timer_button_DN++;
//			if (timer_button_DN == 50)
//			{
//				timer_button_DN = 0;
//				sw_state2 ^=1;
//			}
//			
//			
//		}
		
//	} // для зажатой кнопки
   
//   if (start_timer_button1) timer_button1++; //для входа в меню таймер 3 сек
//   if (timer_button1 >= 3000) 
//   {
////		  SSD1306_Fill(SSD1306_COLOR_BLACK);
//      enter_menu = 1;
//      timer_button1 = 0;
//      start_timer_button1 = 0;
//      screen = 1;
//      
//      SSD1306_Fill_1(SSD1306_COLOR_BLACK);
////      SSD1306_UpdateScreen_My();
////      
////      sprintf(str, "%i Impulse",screen);
////      sprintf(str2, "%i",pointerSettings->Imp);
////      SSD1306_GotoXY(9, 2);
////      SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
////      SSD1306_GotoXY(9, 20);
////      SSD1306_Puts(str2, &Font_7x10, SSD1306_COLOR_WHITE);
//      
////            SSD1306_Fill2(SSD1306_COLOR_BLACK);
////      SSD1306_Fill(SSD1306_COLOR_BLACK);
////            SSD1306_UpdateScreen_My();
////            SSD1306_UpdateScreen_My2();
//         
//   }
   
   
}







//                      0         12         128         12
void Display_float(float Number1, uint8_t Number2, uint8_t Number3) //Ф-ция для разложения десятичного цисла/**/
{
  uint8_t  Num7,Num6,Num5,Num4,Num3,Num2, Num1,Num01;
  Num7=0;
   Num6=0;
  Num5=0;
  Num4=0;
  Num3=0;
  Num2=0;
  Num1=0;
  Num01=0;
   if ( (Number1 > -1) && (Number1 < 1) ) nul = 1; else nul = 0;
  
  if (Number1<0) minus1=1; else minus1=0;  //определяем отрицательное или положительное число
  //switch (minus)
  //{
	  //case 0:
	  //if (Number1>=10000) num_disp=5; else
	  //if ((Number1>=1000)&&(Number1<10000)) num_disp=4; else
	  //if ((Number1>=100)&&(Number1<1000)) num_disp=3; else
	  //if ((Number1>=10)&&(Number1<100)) num_disp=2; else
	  //if ((Number1>=0)&&(Number1<10)) num_disp=1;
	  //break;
	  //case 1:
	  //if (Number1<=-10000) num_disp=6; else
	  //if ((Number1<=-1000)&&(Number1>-10000)) num_disp=5; else
	  //if ((Number1<=-100)&&(Number1>-1000)) num_disp=4; else
	  //if ((Number1<=-10)&&(Number1>-100)) num_disp=3; else
	  //if ((Number1<=0)&&(Number1>-10)) num_disp=2; else num_disp=1;
	  //break;
  //};
  
  if (minus1==0)
  {
//		while (Number1 >= 1000) //1000
//		 {
//		 Number1 -= 1000;  
//		  Num4++; 
//		 }
  
		 while (Number1 >= 100) //100
		 {
		  Number1 -= 100;  
		  Num3++; 
		  }
  
		  while (Number1 >= 10) //10
		  {
		 Number1 -= 10;  
		 Num2++; 
		 }
  
		 while (Number1 >= 1) //1
		 {
		 Number1 -= 1;  
		  Num1++; 
		 }
  
		 while (Number1 >= 0.1) //1
		 {
		 Number1 -= 0.1;  
		 Num01++; 
		 }
  }
  
  if (minus1==1)
  {
//	  while ((Number1 <= -100)&&(Number1>-1000)) //100
//	  {
//		  Number1 += 100;
//		  Num3++;
//	  }
	  
	  while ((Number1 <= -10)&&(Number1>-100)) //10
	  {
		  Number1 += 10;
		  Num2++;
	  }
	  
	  while ((Number1 <= -1)&&(Number1>-10)) //1
	  {
		  Number1 += 1;
		  Num1++;
	  }
	  
	  while ((Number1 < 0)&&(Number1>-1)) //1
	  {
		  Number1 += 0.1;
		  Num01++;
	  }
  }	
  
  //for CLOCK
      while (Number2 >= 10) //10
		  {
		 Number2 -= 10;  
		 Num5++; 
		 }
  
		 while (Number2 >= 1) //1
		 {
		 Number2 -= 1;  
		  Num4++; 
		 }
       
       while (Number3 >= 10) //10
		  {
		 Number3 -= 10;  
		 Num7++; 
		 }
  
		 while (Number3 >= 1) //1
		 {
		 Number3 -= 1;  
		  Num6++; 
		 }
    
    
 // Num1 = Number1; //остаток
  Disp7 = Num7;
  Disp6 = Num6;
       
  Disp5 = Num5;
  Disp4 = Num4;
       
  Disp3 = Num3;
  Disp2 = Num2;
  Disp1 = Num1;
  Disp01= Num01;  
}; 






void Button_m_UP(void)
{
   if ( ( (GPIOA->IDR & GPIO_IDR_0)) && (sw_state_m_UP) ) // Отпустили
   {
//      Button_UP_press = 0;
      sw_state_m_UP = 0;
   }
   
   if ( ( !(GPIOA->IDR & GPIO_IDR_0)) && (!sw_state_m_UP) ) // Нажали
   {
      temp_m = date.Minutes;
//      if (temp_m < 59) temp_m++;
		  temp_m++;
		  if (temp_m > 59) temp_m = 0;
      set_time[0] = 0; // sec
      set_time[1] = temp_m; // minutes
      set_time[2] = date.Hours; // hours
      set_time[3] = date.DOW; // day
      set_time[4] = date.Day; // date
      set_time[5] = date.Month; //month
      set_time[6] = date.Year - 2000; // year

      DS3231_WriteDate(set_time,1);
//         DS3231_write(DS3231_minutes, temp_m);
//      Button_UP_press = 1;
      sw_state_m_UP = 1;
   }
}

void Button_m_DN(void)
{
   if ( ( (GPIOA->IDR & GPIO_IDR_1)) && (sw_state_m_DN) ) // Отпустили
   {
//      Button_DN_press = 0;
      sw_state_m_DN = 0;
   }
   
   if ( ( !(GPIOA->IDR & GPIO_IDR_1)) && (!sw_state_m_DN) ) // Нажали
   {
      temp_m = date.Minutes;
      temp_m--;
		  if (temp_m > 59 ) temp_m = 59;
      set_time[0] = 0; // sec
      set_time[1] = temp_m; // minutes
      set_time[2] = date.Hours; // hours
      set_time[3] = date.DOW; // day
      set_time[4] = date.Day; // date
      set_time[5] = date.Month; //month
      set_time[6] = date.Year - 2000; // year

      DS3231_WriteDate(set_time,1);
//         DS3231_write(DS3231_minutes, temp_m);
//      Button_DN_press = 1;
      sw_state_m_DN = 1;
   }
}


void Button_h_UP(void)
{
   if ( ( (GPIOA->IDR & GPIO_IDR_3)) && (sw_state_h_UP) ) // Отпустили
   {
//      Button_UP_press = 0;
      sw_state_h_UP = 0;
   }
   
   if ( ( !(GPIOA->IDR & GPIO_IDR_3)) && (!sw_state_h_UP) ) // Нажали
   {
      
      temp_h = date.Hours;
      temp_h += 1;
		  if (temp_h > 23) temp_h = 0;
//      set_time[0] = 0; // sec
      set_time[1] = date.Minutes; // minutes
      set_time[2] = temp_h; // hours
      set_time[3] = date.DOW; // day
      set_time[4] = date.Day; // date
      set_time[5] = date.Month; //month
      set_time[6] = date.Year - 2000; // year

      DS3231_WriteDate(set_time,0);
//         DS3231_write(DS3231_hours, temp_h);
//      Button_UP_press = 1;
      sw_state_h_UP = 1;
   }
//   Delay_ms(10);
}

void Button_h_DN(void)
{
   if ( ( (GPIOA->IDR & GPIO_IDR_4)) && (sw_state_h_DN) ) // Отпустили
   {
//      Button_DN_press = 0;
      sw_state_h_DN = 0;
   }
   
   if ( ( !(GPIOA->IDR & GPIO_IDR_4)) && (!sw_state_h_DN) ) // Нажали
   {
      temp_h = date.Hours;
      temp_h--;
		 if (temp_h > 23 ) temp_h = 23;
//      set_time[0] = 0; // sec
      set_time[1] = date.Minutes; // minutes
      set_time[2] = temp_h; // hours
      set_time[3] = date.DOW; // day
      set_time[4] = date.Day; // date
      set_time[5] = date.Month; //month
      set_time[6] = date.Year - 2000; // year

      DS3231_WriteDate(set_time,0);
//         DS3231_write(DS3231_hours, temp_h);
//      Button_DN_press = 1;
      sw_state_h_DN = 1;
   }
}



int main(void)
{
   RCC_DeInit();
   SetSysClockToHSI48();
	SPI_ini();
	Init_PORT();
	
//   OW_Init();
// 	number = OW_Scan( ROM, 1);//ROM - буффер куда пишеться весь SCRATCHPAD от всех датчиков. 2 - это два буффера
//   Delay_ms(10);
//   Init_Port_ADC();
//   DMA_ini_ADC1();
//   ADC_init();
   Init_I2C();
   // на 40 сек отстают от ноутбука
//set_time[0] = 0; // sec
//set_time[1] = 2; // minutes
//set_time[2] = 19; // hours
//set_time[3] = 7; // day
//set_time[4] = 28; // date
//set_time[5] = 10; //month
//set_time[6] = 18; // year

//DS3231_WriteDate(set_time,1);
//DS3231_write(DS3231_seconds, 15);
//DS3231_write(DS3231_minutes, 8);
   DS3231_Init(DS3231_control,0);
//MCP23017_send(0, 0);//in A
//   MCP23017_send(1, 255);//out B
//   Init_Parameters();
//   Init_SSD1306(SSD1306_COLOR_BLACK);
//   Delay_ms(10);

   
//   MCP23017_send(0x14, 0);
////   MCP23017_send(0x15, 255);//B 33. v
//   Init_Parameters();
//   Init_PORT();
//   InitTimer3();
//   InitTimer14();
//   GPIOA->BSRR |= GPIO_BSRR_BR_2; // SP 
//   GPIOA->BSRR |= GPIO_BSRR_BR_3; // TAH
//   GPIOA->BSRR |= GPIO_BSRR_BR_5;
   
//   Init_SSD1306(SSD1306_COLOR_BLACK);

    OW_Init();
    number = OW_Scan( ROM, 1);//ROM - буффер куда пишеться весь SCRATCHPAD от всех датчиков. 2 - это два буффера
    
//Init_OUT();
    
    SysTick_Config(SystemCoreClock/1000); //1 ms
	 
//	 SSD1351_Init();
	 
//	 SSD1351_WriteData(&SSD1351_Buffer[LCDWIDTH * 8], sizeof(SSD1351_Buffer));
//   LCD1_init();
//	 CS_ON;
//   SSD1306_SPI_Fill(SSD1306_SPI_COLOR_WHITE);
//	 SSD1306_SPI_UpdateScreen_My();
//   CS_OFF;
	 
//   VOLT[0] = mV_chanel[2] * 11;
//   Delay_ms(100);
//   EEPROM_16bit_write(60,950); // LED_max
//   EEPROM_16bit_write(62,250); // LED_min
//   Save_Parameters_Frst_Init_EE();

//	SSD1351_DrawPixel(5, 10, 255, 0, 0);
//	SSD1351_DrawPixel(4, 20, 0, 255, 0);
//	SSD1351_DrawPixel(3, 30, 0, 0, 255);
//	SSD1351_DrawPixel(2, 40, 127, 127, 0);
//	SSD1351_DrawPixel(1, 50, 0, 127, 127);
//  SSD1351_DrawPixel(0, 0, 127, 0, 127);
//	Delay_ms(3000);
//	SSD1351_WriteChar(30, 30, 48, &Font_11x18, 135, 35, 35, 255, 0, 0);
//	Delay_ms(3000);
//	sprintf(str,  "Temp");
//	SSD1351_WriteString(0, 50, str, &Font_11x18, 250, 0, 0, 0, 0, 0);
//	Delay_ms(3000);

//	SSD1351_FillScreen(0,0,0);
//	Delay_ms(3000);
	HC1 = 1;
	HC2 = Dig[1];
	
   T_sr[1] = 2;
   while(1) 
   {
      
      Display_float(T_sr[0],date.Hours,date.Minutes); // T_sr[0]
		 
//	 for (temp_HC = 0; temp_HC < 10; temp_HC++)	 
//		 {
//			 
//			 
//			 
//			 
////		HC2 = Dig[temp_HC]; 
////		 
//////	 SPI1_SendByte(255);//4+2+1
//		 temp_HC++;
//		 if (temp_HC > 9) temp_HC = 0;
//   SPI1_SendByte(HC1);//6+2 numbers
//   SPI1_SendByte(Dig[temp_HC]);//segments
//   LATCH_ON;
//   LATCH_OFF;		
//		 Delay_ms(1000);
//		 }
		 
//		 GPIOA->BSRR |= GPIO_BSRR_BS_7;//DATA
//								
//for (HC1 = 0; HC1 < 16; HC1++)
//{		 
//		 GPIOA->BSRR |= GPIO_BSRR_BS_5;//CLK
//		 LATCH_OFF;
//   Delay_ms(500);  
//		 
//	 GPIOA->BSRR |= GPIO_BSRR_BR_5;
//	 LATCH_ON;
//	 Delay_ms(500);
//}

//GPIOA->BSRR |= GPIO_BSRR_BR_7;//DATA
//								
//for (HC1 = 0; HC1 < 16; HC1++)
//{		 
//		 GPIOA->BSRR |= GPIO_BSRR_BS_5;//CLK
//		 LATCH_OFF;
//   Delay_ms(500);  
//		 
//	 GPIOA->BSRR |= GPIO_BSRR_BR_5;
//	 LATCH_ON;
//	 Delay_ms(500);
//}
//		 Delay_ms(500);
//   LATCH_OFF;
//		 Delay_ms(500);
		 
//		HC595(1, Disp2);
//      Delay_ms(2);
//      HC595(2, Disp1);
//      Delay_ms(2);
      
//      HC595(3, Disp6);
//      Delay_ms(2);
//      HC595(4, Disp7);
//      Delay_ms(2);
//      
//      HC595(5, Disp4);
//      Delay_ms(2);
//      HC595(6, Disp5);
//      Delay_ms(2);
//       Delay_ms(500);
//       temp_HC = temp_HC << 1;
//      if (temp_HC == 0) temp_HC = 1;

		 
		 
//		 sprintf(str, "Display 1");
//		 CS_ON;     
//			SSD1306_SPI_Fill(SSD1306_SPI_COLOR_BLACK);
//      SSD1306_GotoXY_SPI(3, 4);
//			SSD1306_Puts_SPI(str, &Font_7x10, SSD1306_SPI_COLOR_WHITE);
//		 SSD1306_SPI_UpdateScreen_My();
//		 CS_OFF;  
		 
		 
      //график 1/x^2
//      R2 = (2 * mV_chanel[0])/(3.3 - mV_chanel[0]);
//      Temperature();
//		DS3231_ReadDate(buffer);
      if (mesure == 0) 
   {
		  
      OW_Send(OW_SEND_RESET, "\xcc\x44", 2,  NULL,  NULL,  OW_NO_READ);
      mesure = 1;
//		  DS3231_ReadDate(buffer);
   }
////   Delay_ms(800);
   if (mesure == 2)
   {
      sprintf(txt, "\x55%c%c%c%c%c%c%c%c\xbe\xff\xff",ROM[0],ROM[1],ROM[2],ROM[3],ROM[4],ROM[5],ROM[6],ROM[7]);
      OW_Send(OW_SEND_RESET, txt, 12,  buf,  2,  10); 
      if (buf[1] == 0xFF) // minus
      {
         temp2 = ( ((buf[1]&0x07)<<4)|(buf[0]>>4));
         temp1=(buf[0]&0x0F);	
         temp1=((temp1<<1)+(temp1<<3));	
         temp1=(temp1>>4);
         T_sr[0] = (temp2 + (temp1 * 0.1)) - 127;
      }
      else
      {
         temp2 = ( ((buf[1]&0x07)<<4)|(buf[0]>>4));
         temp1=(buf[0]&0x0F);	
         temp1=((temp1<<1)+(temp1<<3));	
         temp1=(temp1>>4);
         T_sr[0] = (temp2 + (temp1 * 0.1));
      }
      
//		 //второй датчик
//      sprintf(txt, "\x55%c%c%c%c%c%c%c%c\xbe\xff\xff",ROM[8],ROM[9],ROM[10],ROM[11],ROM[12],ROM[13],ROM[14],ROM[15]);
//      OW_Send(OW_SEND_RESET, txt, 12,  buf,  2,  10); 
//      
//      temp4 = ( ((buf[1]&0x07)<<4)|(buf[0]>>4));
//      temp3=(buf[0]&0x0F);	
//      temp3=((temp3<<1)+(temp3<<3));	
//      temp3=(temp3>>4);
//      T_sr[1] = (temp4 + (temp3 * 0.1));
//////      DS3231_Init(DS3231_control,0);
//////      DS3231_ReadDate(buffer);
//////      DS3231_write(DS3231_control, 38); // для оцифровки температуры
      mesure = 0;

   }   
   Button_m_UP();
   Button_m_DN();
   Button_h_UP();
   Button_h_DN();
   
//   sprintf(str,  "sec = %2i",sec);
//		 SSD1351_WriteString(0, 0, str, &Font_7x10, 0, 127, 0, 0, 0, 0);
//		 sprintf(str2,  "FPS = %3i",frame_max);
//		 SSD1351_WriteString(0, 12, str2, &Font_7x10, 0, 0, 127, 0, 0, 0);
//		 sprintf(str3,  "RMS_20=%4i",max_rms_sample);
//		 SSD1351_WriteString(0, 24, str3, &Font_7x10, 127, 0, 0, 0, 0, 0);
//       sprintf(str4,  "DS=%1i OW_Status = %i",number,OW_err);//T1 = %2.1f    ,T_sr[0]
//		 SSD1351_WriteString(0, 36, str4, &Font_7x10, 0, 80, 255, 0, 0, 0);
//       sprintf(str5,  "t1=%2.1f t2=%2.1f",T_sr[0],T_sr[1]);//T1 = %2.1f    ,T_sr[0]
//		 SSD1351_WriteString(0, 48, str5, &Font_7x10, 0, 80, 255, 0, 0, 0);
//       sprintf(txt, "%02x%02x%02x%02x%02x%02x%02x%02x  %02x%02x%02x%02x%02x%02x%02x%02x  ",ROM[0],ROM[1],ROM[2],ROM[3],ROM[4],ROM[5],ROM[6],ROM[7],ROM[8],ROM[9],ROM[10],ROM[11],ROM[12],ROM[13],ROM[14],ROM[15]);
//		 SSD1351_WriteString(0, 60, txt, &Font_7x10, 0, 80, 255, 0, 0, 0);
//       sprintf(str6, "%02u:%02u.%02u    T=%2u",date.Hours,date.Minutes,date.Seconds,(buffer[17] - 8));
//		 SSD1351_WriteString(0, 84, str6, &Font_7x10, 255, 80, 255, 0, 0, 0);
//       sprintf(str7, "%02u-%02u-%04u  %2u  ",date.Day,date.Month,date.Year,date.DOW);
//		 SSD1351_WriteString(0, 96, str7, &Font_7x10, 255, 80, 255, 0, 0, 0);
		 frame++;
   
	 
//      TIM14->CCR1 = 200;
//      sec++;
      
      
//      if (bit == 0) {MCP23017_send(0x14, out_A);} // out B 0 v
//      else {MCP23017_send(0x14, ~out_A);} // OUT B 3.3 V
      
//      Button_UP();
//      Button_DN();
//      Button_Menu(); 
//      No_Menu();
//      TOK[0] = (mV_chanel[1] - 1.66) * 20;
//      if ( (TOK[0] < 0.02) && (TOK[0] > -0.02) ) TOK[0] = 0;
//      VOLT[0] = mV_chanel[2] * 11;
//         No_Menu();
//         SSD1306_UpdateScreen_My();
//      if (mV_chanel[1] < 1.672) mV_chanel[1] = 1.6701;
//      sprintf(str,  "%2.1fV %2.2fA ",VOLT[0],TOK[0]);//frame_max
//      sprintf(str2, "%2.2fA %2.2f ",((mV_chanel[1] - 1.67) * 20),(1/(R2*R2)));
//      sprintf(str3, "%2.2f %2.2f %i ",mV_chanel[0], T[0], bit);
//      SSD1306_Fill_1(SSD1306_COLOR_BLACK);
//      sprintf(str4, "sampl 20ms = %i ",max_rms_sample);
//      sprintf(str5, "TrueRMS Volt/Amp  ");
//      if (clear == 1) 
//      {
//         SSD1306_Fill_1(SSD1306_COLOR_BLACK); clear = 0;
//      }

//      if ( (g_t < 127)  ) //&& (g_t > 0)
//      {
//         SSD1306_DrawPixel_1((g_t), (64-GRAPH_Buff[g_t-1]), SSD1306_COLOR_WHITE);
//         Draw_HLine(0,18,128,18);
//         Draw_HLine(0,40,128,40);
//         Draw_HLine(0,63,128,63);
//         Draw_VLine(0,18,0,63);
//         Draw_VLine(63,18,63,63);
//         Draw_VLine(127,18,127,63);
//         
//      }
//      else
//      {
//         SSD1306_Fill_1(SSD1306_COLOR_BLACK);
//         Draw_HLine(0,18,128,18);
//         Draw_HLine(0,40,128,40);
//         Draw_HLine(0,63,128,63);
//         Draw_VLine(0,18,0,63);
//         Draw_VLine(63,18,63,63);
//         Draw_VLine(127,18,127,63);
//         frame++;
//         for (g = 0;g<128;g++)
//         {
//            SSD1306_DrawPixel_1(g, (64-GRAPH_Buff[g]), SSD1306_COLOR_WHITE);
//         }
//         
//      }
//      SSD1306_GotoXY_1(0, 0);
//      SSD1306_Puts_1(str, &Font_11x18, SSD1306_COLOR_WHITE);
//      SSD1306_GotoXY_1(0, 20);
//      SSD1306_Puts_1(str2, &Font_10x12, SSD1306_COLOR_WHITE);
//      SSD1306_GotoXY_1(0, 40);
//      SSD1306_Puts_1(str3, &Font_11x18, SSD1306_COLOR_WHITE);
//      SSD1306_GotoXY_1(0, 36);
//      SSD1306_Puts_1(str4, &Font_7x10, SSD1306_COLOR_WHITE);
//      SSD1306_GotoXY_1(0, 52);
//      SSD1306_Puts_1(str5, &Font_7x10, SSD1306_COLOR_WHITE);
//      for (g = 0;g<128;g++)
      
         
            
      
      
      
//      SSD1306_UpdateScreen_My_1();
      
      
   }

}
