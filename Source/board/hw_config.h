/**
  ******************************************************************************
  * @file    stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
void TimingDelay_Decrement(void);
void InitPins(void);
void RCC_Configuration(void);
void init_button(void);
void nvic_tacho(void);


//#include "stm32f10x_adc.h"
//#include "stm32f10x_bkp.h"
//#include "stm32f10x_can.h"
//#include "stm32f10x_cec.h"
//#include "stm32f10x_crc.h"
//#include "stm32f10x_dac.h"
//#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h" 
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
//#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h" 
#include "stm32f10x_i2c.h"
//#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h" 
//#include "stm32f10x_rtc.h"
//#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
//#include "stm32f10x_wwdg.h"
#include "misc.h"   /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//мои переменные для мигания раз в секунду
extern	uint32_t	astmsi;
extern	uint32_t	astmsc;
extern	uint32_t	astmsd;	


//назначим выводам органы управления
//выход контроля тактовой частоты MCO PA8
//альтернативная функция
#define MCO_GPIO_PORT		GPIOA
#define MCO_GPIO_PIN    GPIO_Pin_9
#define MCO_GPIO_CLK    RCC_AHBPeriph_GPIOA

//АЦП канал 0 канал 1 канал 2
#define	ADC1_GPIO_PIN0				GPIO_Pin_0	//AKB_in
#define	ADC1_GPIO_PIN1				GPIO_Pin_1	//Temp
#define	ADC1_GPIO_PIN2				GPIO_Pin_2	//Fuel
#define ADC1_GPIO_PORT 				GPIOA
#define ADC1_PORT_CLK   			RCC_AHBPeriph_GPIOA

//вход AKPP_bit0
//PC0
#define AKPP_bit0_GPIO_PIN			GPIO_Pin_0	
#define AKPP_bit0_GPIO_PORT 		GPIOC
#define AKPP_bit0_PORT_CLK   		RCC_AHBPeriph_GPIOC

//вход AKPP_bit1
//PC1
#define AKPP_bit1_GPIO_PIN			GPIO_Pin_1	
#define AKPP_bit1_GPIO_PORT 		GPIOC
#define AKPP_bit1_PORT_CLK   		RCC_AHBPeriph_GPIOC

//вход AKPP_bit2
//PC2
#define AKPP_bit2_GPIO_PIN			GPIO_Pin_2	
#define AKPP_bit2_GPIO_PORT 		GPIOC
#define AKPP_bit2_PORT_CLK   		RCC_AHBPeriph_GPIOC

//вход микропереключателя SW_LED5.1
//PB1
#define SW_LED5_1_GPIO_PIN			GPIO_Pin_1	
#define SW_LED5_1_GPIO_PORT 		GPIOB
#define SW_LED5_1_PORT_CLK   		RCC_AHBPeriph_GPIOB

//вход микропереключателя SW_LED5.2
//PB2
#define SW_LED5_2_GPIO_PIN			GPIO_Pin_2	
#define SW_LED5_2_GPIO_PORT 		GPIOB
#define SW_LED5_2_PORT_CLK   		RCC_AHBPeriph_GPIOB

//вход микропереключателя SW_LED5.3
//PB14
#define SW_LED5_3_GPIO_PIN			GPIO_Pin_14	
#define SW_LED5_3_GPIO_PORT 		GPIOB
#define SW_LED5_3_PORT_CLK   		RCC_AHBPeriph_GPIOB

//вход микропереключателя SW_LED5.4
//PB12
#define SW_LED5_4_GPIO_PIN			GPIO_Pin_12	
#define SW_LED5_4_GPIO_PORT 		GPIOB
#define SW_LED5_4_PORT_CLK   		RCC_AHBPeriph_GPIOB

//вход микропереключателя SW_LED5.5
//PB13
#define SW_LED5_5_GPIO_PIN			GPIO_Pin_13	
#define SW_LED5_5_GPIO_PORT 		GPIOB
#define SW_LED5_5_PORT_CLK   		RCC_AHBPeriph_GPIOB

//вход кнопки SB1 (CHECK)
//PC6
#define SB1_GPIO_PIN				GPIO_Pin_6	
#define SB1_GPIO_PORT 			GPIOC
#define SB1_PORT_CLK   		  RCC_AHBPeriph_GPIOC

//вход кнопки SB2 (O/D)
//PC7
#define SB2_GPIO_PIN				GPIO_Pin_7	
#define SB2_GPIO_PORT 			GPIOC
#define SB2_PORT_CLK   		  RCC_AHBPeriph_GPIOC

//вход кнопки SB3 (OIL)
//PC8
#define SB3_GPIO_PIN				GPIO_Pin_8	
#define SB3_GPIO_PORT 			GPIOC
#define SB3_PORT_CLK   		  RCC_AHBPeriph_GPIOC

//вход внешнего прерывания Tacho
//PC9
#define Tacho_GPIO_PIN				GPIO_Pin_9
#define Tacho_GPIO_PORT 			GPIOC
#define Tacho_PORT_CLK   			RCC_AHBPeriph_GPIOC

//вход внешнего прерывания Speed
//PC10
#define Speed_GPIO_PIN				GPIO_Pin_10
#define Speed_GPIO_PORT 			GPIOC
#define Speed_PORT_CLK   			RCC_AHBPeriph_GPIOC

//вход внешнего прерывания Fors
//PB0
#define Fors_GPIO_PIN					GPIO_Pin_0
#define Fors_GPIO_PORT 				GPIOB
#define Fors_PORT_CLK   			RCC_AHBPeriph_GPIOB

//выход управления Starter
//PC11
#define	Starter_GPIO_PIN		GPIO_Pin_11
#define Starter_GPIO_PORT 	GPIOC
#define Starter_PORT_CLK    RCC_AHBPeriph_GPIOC

//выход управления Cruise
//Cruise On/Off 
//PA4
#define	Cruise_1_GPIO_PIN		GPIO_Pin_4
#define Cruise_1_GPIO_PORT 	GPIOA
#define Cruise_1_PORT_CLK   RCC_AHBPeriph_GPIOA

//Cruise Res/Acc
//PA5
#define	Cruise_2_GPIO_PIN		GPIO_Pin_5
#define Cruise_2_GPIO_PORT 	GPIOA
#define Cruise_2_PORT_CLK  	RCC_AHBPeriph_GPIOA

//Cruise Set/Coast
//PA6
#define	Cruise_3_GPIO_PIN		GPIO_Pin_6
#define Cruise_3_GPIO_PORT 	GPIOA
#define Cruise_3_PORT_CLK  	RCC_AHBPeriph_GPIOA

//Cruise Cancel
//PA7
#define	Cruise_4_GPIO_PIN		GPIO_Pin_7
#define Cruise_4_GPIO_PORT	GPIOA
#define Cruise_4_PORT_CLK  	RCC_AHBPeriph_GPIOA

//MPX-Tx (input signal)
//PB10
#define	MPX_Tx_GPIO_PIN		GPIO_Pin_10
#define MPX_Tx_GPIO_PORT	GPIOB
#define MPX_Tx_PORT_CLK  	RCC_AHBPeriph_GPIOB

//MPX-Rx (output signal)
//PB11
#define	MPX_Rx_GPIO_PIN		GPIO_Pin_11
#define MPX_Rx_GPIO_PORT	GPIOB
#define MPX_Rx_PORT_CLK  	RCC_AHBPeriph_GPIOB
