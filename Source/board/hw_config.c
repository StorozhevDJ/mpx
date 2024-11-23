//#include "stm32H107.h"
#include "hw_config.h"
//#include "can_init.h"

/*USART_TypeDef* COM_USART[COMn] = {EVAL_COM1}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};*/

//константы системного таймера
#define	SYSTICK_MAXCOUNT		0xFFFFFF
#define	SYSTICK_ENABLE			0x0
#define	SYSTICK_TICKINT		0x1
#define	SYSTICK_CLKSOURCE		0x2

int	ticks;
extern 	char 		string[];
extern	int			TimingDelay;


DMA_InitTypeDef   sEEDMA_InitStructure; 

//это добавлено	
	GPIO_InitTypeDef   			GPIO_InitStructure;

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
/*void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO clock
  RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

  if (COM == COM1)
  {
    // Enable the USART2 Pins Software Remapping
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }

  // Configure USART Tx as alternate function push-pull
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  // Configure USART Rx as input floating
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  // USART configuration
  USART_Init(COM_USART[COM], USART_InitStruct);
    
  // Enable USART
  USART_Cmd(COM_USART[COM], ENABLE);
}*/



void RCC_Configuration(void)
	{  
//	настроим тактовую систему 
//	заданы следующие параметры:
//	тактирование от внешннего кварца 16МГц, тактовую частоту процессора зададим 72Мгц
//	ПРОПИШЕМ В ФАИЛЕ 	STM32F10X.H
//	И ТАМ ВСЕ НАСТРОИТСЯ,	но не работает....
//	система тактирования замудренная, и в файлах по умолчанию настроена на 25Mhz
//	нам надо сделать по другому:
//	OTG only 8 DIV 16 PLL2=8  PLLON x9 /3 PLL3OFF 
//XT=16
RCC_DeInit();
RCC_HSEConfig(RCC_HSE_ON); 	// включаем внешний генератор HSE 16MHz
RCC_WaitForHSEStartUp(); 		// ждем пока включится

/* Enable Prefetch Buffer */
// разрешаем буфер предварительного чтения флеша
FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
/* Flash 2 wait state */
//для частоты от 48MHz до 72MHz требуется два такта ожидания чтения влеша 
FLASH_SetLatency(FLASH_Latency_2);

/* HCLK = SYSCLK */
RCC_HCLKConfig(RCC_SYSCLK_Div1);
/* PCLK1 = HCLK / 2 = 36MHZ но частота для таймеров  * 2 в данном случае */
RCC_PCLK1Config(RCC_HCLK_Div2);	//зададим частоту шины APB1 и ее переферии
//RCC_PCLK1Config(RCC_HCLK_Div4);	//зададим частоту шины APB1 и ее переферии
/* PCLK2 = HCLK / 2 = 36MHZ но частота для таймеров  * 2 в данном случае */
RCC_PCLK2Config(RCC_HCLK_Div2); //зададим частоту шины APB2 и ее переферии
//RCC_PCLK2Config(RCC_HCLK_Div4);	//зададим частоту шины APB1 и ее переферии
//зададим частоту ADC (максимум 14МГЦ)
/* PCLK2 / 4 = 9 MHZ */
RCC_ADCCLKConfig(RCC_PCLK2_Div4);
//зададим частоту для USB 48MHz как 72/1.5
//RCC_USBCLKConfig (RCC_USBCLKSource_PLLCLK_1Div5);

// Configure PLLs ********************************************************
// PPL2 configuration: PLL2CLK = (HSE / 16) * 8 = 8 MHz
RCC_PREDIV2Config(RCC_PREDIV2_Div16);
RCC_PLL2Config(RCC_PLL2Mul_8);

// Enable PLL2
RCC_PLL2Cmd(ENABLE);
// Wait till PLL2 is ready
while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET){}

// PPL1 configuration: PLLCLK = (PLL2 ) * 9 = 72 MHz
RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div1);
RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);					

// команда запустить тактирование от PLL
RCC_PLLCmd(ENABLE);
	
/* Wait till PLL is ready */
while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
/* Select PLL as system clock source */
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
/* Wait till PLL is used as system clock source */
while (RCC_GetSYSCLKSource() != 0x08){}
	
//включем тактирование модулей GPIO, ADC, TIM1

RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA |
					RCC_APB2Periph_GPIOB |
					RCC_APB2Periph_GPIOC |
					RCC_APB2Periph_AFIO	 |
					RCC_APB2Periph_ADC1	 ,
					ENABLE); 

						
//-----------------------------------------------------------------------------------------
//настроим SysTickTimer(STK)
//системная частота 72000000 делится AHB прескалером на 1 и получаем частоту HCLK	72000000
//далее она делится на 1 или 8 прескалером SYSTICK	и поступает на системный таймер ядра
//зададим прескалер = 8
//настроим на 1000 прерываний в секунду
//системный таймер, вызывающий прерывания через определённое количество тактов генератора
//счётчик делителя тогда равен (9000000/1000)=9000
//и считать будет от 9000 до 0 -> прерывание
ticks=9000;
SysTick->LOAD  = (ticks & SYSTICK_MAXCOUNT) - 1;						//загрузим с маской 
NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);	//зададим приоритет
// выбираем источник синхронизации частоту процессора, разрешаем прерывание при достижении 0, разрешаем работу
SysTick->CTRL = (0 << SYSTICK_CLKSOURCE) | (1<<SYSTICK_ENABLE) | (1<<SYSTICK_TICKINT);//частота AHB/8
}



void	nvic_tacho(void)
{
		NVIC_InitTypeDef 				NVIC_InitStructure;	// контроллера прерываний
		EXTI_InitTypeDef   			EXTI_InitStructure;
//назначим прерывание для сигнала Тахометра по отрицательному фронту
//PC9 может быть подключен к линии прерывания EXTI9
//Назначаем порт и вывод источника прерывания
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
		//выбираем линию для прерывания  EXTI
		EXTI_InitStructure.EXTI_Line = EXTI_Line9;
		//выбираем режим что прерывание 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		//выбираем по отрицательному фронту
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		//разрешаем прерывание
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		//инициализируем
    EXTI_Init(&EXTI_InitStructure);

		//configure NVIC
    //select NVIC channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    //set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    //set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
    //enable IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}



void init_button(void)
	{
//настроим вход bit0 bit1 bit2 селектора АКПП
//bit0
		GPIO_InitStructure.GPIO_Pin = AKPP_bit0_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(AKPP_bit0_GPIO_PORT, &GPIO_InitStructure);

//bit1
		GPIO_InitStructure.GPIO_Pin = AKPP_bit1_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(AKPP_bit1_GPIO_PORT, &GPIO_InitStructure);

//bit2
		GPIO_InitStructure.GPIO_Pin = AKPP_bit2_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(AKPP_bit2_GPIO_PORT, &GPIO_InitStructure);

//настроим вход Check
  	GPIO_InitStructure.GPIO_Pin = SB1_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SB1_GPIO_PORT, &GPIO_InitStructure);
		
//настроим вход O/D
  	GPIO_InitStructure.GPIO_Pin = SB2_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SB2_GPIO_PORT, &GPIO_InitStructure);
		
		
//настроим вход OIL
  	GPIO_InitStructure.GPIO_Pin = SB3_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SB3_GPIO_PORT, &GPIO_InitStructure);

//настроим вход кнопки SW_LED5.1
  	GPIO_InitStructure.GPIO_Pin = SW_LED5_1_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SW_LED5_1_GPIO_PORT, &GPIO_InitStructure);
		
//настроим вход кнопки SW_LED5.2
  	GPIO_InitStructure.GPIO_Pin = SW_LED5_2_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SW_LED5_2_GPIO_PORT, &GPIO_InitStructure);

//настроим вход кнопки SW_LED5.3
  	GPIO_InitStructure.GPIO_Pin = SW_LED5_3_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SW_LED5_3_GPIO_PORT, &GPIO_InitStructure);

//настроим вход кнопки SW_LED5.4
  	GPIO_InitStructure.GPIO_Pin = SW_LED5_4_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SW_LED5_4_GPIO_PORT, &GPIO_InitStructure);

//настроим вход кнопки SW_LED5.5
  	GPIO_InitStructure.GPIO_Pin = SW_LED5_5_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//in & pull up
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(SW_LED5_5_GPIO_PORT, &GPIO_InitStructure);
}



void InitPins(){

//	GPIO_InitTypeDef   			GPIO_InitStructure;

//PC9
//Сигнал Тахометра
//Настроим вход внешнего прерывания
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = Tacho_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
  	GPIO_Init(Tacho_GPIO_PORT, &GPIO_InitStructure);
		
	
//PA0 PA1 PA2
//Каналы Fuel, АКБ, Температуры ОЖ
//Настроим вход АЦП
  	GPIO_InitStructure.GPIO_Pin = ADC1_GPIO_PIN0 | ADC1_GPIO_PIN1 | ADC1_GPIO_PIN2;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(ADC1_GPIO_PORT, &GPIO_InitStructure);
		GPIO_WriteBit(ADC1_GPIO_PORT, ADC1_GPIO_PIN0 |ADC1_GPIO_PIN1 | ADC1_GPIO_PIN2 , Bit_RESET);

//PC11
//Пин управления реле стартером
  	GPIO_InitStructure.GPIO_Pin = Starter_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(Starter_GPIO_PORT, &GPIO_InitStructure);
		
//PA4
//Пин управления круиз - on-off cruise
  	GPIO_InitStructure.GPIO_Pin = Cruise_1_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(Cruise_1_GPIO_PORT, &GPIO_InitStructure);

//PA5
//Пин управления круиз - Resume/Accel
  	GPIO_InitStructure.GPIO_Pin = Cruise_2_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(Cruise_2_GPIO_PORT, &GPIO_InitStructure);

//PA6
//Пин управления круиз - Set/Coast
  	GPIO_InitStructure.GPIO_Pin = Cruise_3_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(Cruise_3_GPIO_PORT, &GPIO_InitStructure);

//PA7
//Пин управления круиз - Cancel
  	GPIO_InitStructure.GPIO_Pin = Cruise_4_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
  	GPIO_Init(Cruise_4_GPIO_PORT, &GPIO_InitStructure);
		
//PB10 
//MPX-Tx (input signal)
  	GPIO_InitStructure.GPIO_Pin = MPX_Tx_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
  	GPIO_Init(MPX_Tx_GPIO_PORT, &GPIO_InitStructure);

//PB11
//MPX-Rx (output signal)
  	GPIO_InitStructure.GPIO_Pin = MPX_Rx_GPIO_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
  	GPIO_Init(MPX_Rx_GPIO_PORT, &GPIO_InitStructure);


}



//-------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------									
//вызывается из прерывания системного таймера, настроенного на 1000 прерываний в секунду
void TimingDelay_Decrement(void)
{
if (TimingDelay) TimingDelay--;

astmsi++;							//инкрементирруем переменную счета миллисекунд
	if(astmsi==(astmsc-1))
	{
		astmsd^=0x01;			//инвертируем бит 0x01
		astmsi=0;
	}
}
//-------------------------------------------------------------------------------------------------------
