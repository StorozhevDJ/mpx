/*******************************************************************************
* 
* MPX BEAN bus driver for STM32F1xx
* 
*/


#include "mpx.h"
#include "utils.h"
#include "crc.h"



#define FREQUENCY_MHZ   	72
#define MPX_TIMEOUT     	(FREQUENCY_MHZ * 2000)
//#define MPX_IDLE        	(FREQUENCY_MHZ * 1500)
#define MPX_MSEC_TIMEOUT	(FREQUENCY_MHZ * 1000)
#undef  MPX_AUTOBAUD    



struct mpx_bus  bus;
struct mpx_bean inbean;

unsigned char   outbean_bits[128];
unsigned char   outbean_start_with_bit;
unsigned char   outbean_bit;
unsigned char   outbean_active;
unsigned char   outbean_index;
unsigned char   outbean_count;
unsigned char   outbean_rtr;
unsigned char   outbean_pause;

unsigned long   bit_coef;
unsigned short  bit_time;
unsigned int    bit_latch;
signed char     bits;

unsigned int    times[128];
unsigned char   tbits[128];



//------------------------------------------------------------------------------
TSmpx mpx_packet;		//Received data in MPX packet struct
TEMPXstate mpx_state;

uint16_t mpx_timer_rx[MPX_BUFF_LEN_MAX];
uint16_t mpx_timer_tx[MPX_BUFF_LEN_MAX];
uint16_t mpx_timer_ptr_rx=0;
uint16_t mpx_timer_ptr_tx=0;
uint16_t mpx_timer_last=0;

uint16_t bit_length=1500;
uint16_t mpx_timer_cnt_tx=0;



//Private fucntion
//Parse received stream
void MPXParseTimerArray(void);



//Pointer to callback function MPX RX Handler (optional use)
void (* MPX_RX_Handler) (TSmpx data);	



//Синхронизация
void mpx_initial_falling_front(void);
void mpx_reset_timeout(void);
void mpx_setup_measure_bit(void);
void mpx_measure_bit_begin(void);
void mpx_measure_bit_end(void);
void mpx_start_transmit(void);
void mpx_setup_lost_sync(void);
//Прием
void mpx_start_bit(void);
void mpx_stream(void);
void mpx_sof(void);
void mpx_pri_ml(void);
void mpx_dstid(void);
void mpx_msgid(void);
void mpx_data(void);
void mpx_crc(void);
void mpx_eom(void);
//Передача
void mpx_send_idle(void);
void mpx_send_stream(void);
void mpx_dummy(void);

unsigned char inbox_bean(struct mpx_bean*bean);




/*****************************************************************************/
/* Инициализация */               
/*****************************************************************************/
void mpx_init(void)
{
/*outbean_active = 0;
bus.mpx_active = 0;
bus.lost_count = 0;
//1 - FALLING
//0 - RISING
bus.front = MPX_INITIAL_FRONT;
bus.mpx_sync_handler = mpx_dummy;//mpx_initial_falling_front;
bus.mpx_timeout_handler = mpx_dummy;
bus.lost_count = 0;
bus.mpx_state = MPX_STATE_IDLE;
bus.msec = 0;*/
  

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//PORT B clock enable

//Init GPIOB.11 RX_MPX (MCU output)
GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_InitStruct.GPIO_Pin=MPXTX_PIN;
GPIO_Init(MPX_PORT, &GPIO_InitStruct);
//Init GPIOB.10 TX_MPX (MCU input)
GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;
GPIO_InitStruct.GPIO_Pin=MPXRX_PIN;
GPIO_Init(MPX_PORT, &GPIO_InitStruct);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//Alternate function (TIM2 remap) clock enable
GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);		//TIM2 remap enable


//TIM2_CH3 MPX_RX Init (bit lenght measure)
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2 clock enable

TIM_TimeBaseInitTypeDef timer_base;
TIM_TimeBaseStructInit(&timer_base);
timer_base.TIM_Prescaler = 2;//72 - 1;//72MHz/72=1kHz=1us
timer_base.TIM_ClockDivision = TIM_CKD_DIV4;//72MHz
TIM_TimeBaseInit(TIM2, &timer_base);

// Capture mode setup:
TIM_ICInitTypeDef timer_ic;
timer_ic.TIM_Channel = TIM_Channel_3;
timer_ic.TIM_ICPolarity = TIM_ICPolarity_Falling;
timer_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
timer_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
timer_ic.TIM_ICFilter = 0;
TIM_ICInit(TIM2, &timer_ic);

// Выбираем источник для триггера: вход 1 (PA6)
TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
// По событию от триггера счётчик будет сбрасываться.
TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
// Включаем события от триггера
TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

// Разрешаем таймеру генерировать прерывание по захвату
TIM_ClearITPendingBit (TIM2, TIM_IT_CC3);
TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
// TIM2 Interrupt enable
NVIC_EnableIRQ(TIM2_IRQn);
// Timer enable
TIM_Cmd(TIM2, ENABLE);

TIM_OCInitTypeDef  TIM_OCInitStructure;
// always initialise local variables before use
TIM_OCStructInit (&TIM_OCInitStructure);
// just use basic Output Compare Mode
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
// set the initial match interval for CC1
TIM_OCInitStructure.TIM_Pulse = 0xffff;
TIM_OC4Init (TIM2, &TIM_OCInitStructure);

TIM_ClearITPendingBit (TIM2, TIM_IT_CC4);
// put the counter into a known state
TIM_SetCounter (TIM2, 0);
// Disable the interrupt for CC4 (enable to start transmit)
TIM_ITConfig (TIM2, TIM_IT_CC4, DISABLE);



//TIM3 Init (timeout timer)
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//TIM3 clock enable

TIM_TimeBaseStructInit(&timer_base);
timer_base.TIM_Prescaler = 2;
timer_base.TIM_ClockDivision = TIM_CKD_DIV1;//
timer_base.TIM_Period=1600*6;//		//Over 6 bits length timeout
TIM_TimeBaseInit(TIM3, &timer_base);

//Timer update flag reset
TIM_ClearFlag(TIM3, TIM_FLAG_Update);
// Разрешаем таймеру генерировать прерывание
TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
// TIM3 Interrupt enable
NVIC_EnableIRQ(TIM3_IRQn);
// Timer enable
TIM_Cmd(TIM3, ENABLE);

/*bus.bit_coef[0]=105*1;//One bit lenght
bus.bit_coef[1]=105*2;//Two bit lenght
bus.bit_coef[2]=105*3;
bus.bit_coef[3]=105*4;
bus.bit_coef[4]=105*5;
bus.bit_coef[5]=105*6;*/
}



/*******************************************************************************
* Measuring pulse length timer
* (capture mode)
*/
void TIM2_IRQHandler(void)
{
if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)//Receive
	{
	//Reset timer capture interrupt flag
	TIM2->SR=~TIM_IT_CC3;
	int capture = TIM_GetCapture3(TIM2);
	mpx_reset_timeout();
	mpx_timer_rx[mpx_timer_ptr_rx]=(capture>mpx_timer_last)?(capture-mpx_timer_last):(capture-mpx_timer_last+65535);
	mpx_timer_last=capture;
	if (++mpx_timer_ptr_rx>=MPX_BUFF_LEN_MAX) mpx_timer_ptr_rx=0;
	TIM2->CCER ^= TIM_CCER_CC3P;//invert edge polarity
	if (mpx_state!=MPX_RECEIVE)
		{
		mpx_state=MPX_RECEIVE;
		TIM_Cmd(TIM3, ENABLE);	//Enable timeout timer
		}
	//bus.mpx_sync_handler();
	}
    //Handle over-capture here 
if (TIM_GetITStatus(TIM2, TIM_FLAG_CC3OF) != RESET)
	{
	TIM2->SR=~TIM_FLAG_CC3OF;//TIM_ClearFlag(TIM2, TIM_FLAG_CC3OF);
	}
}



/*******************************************************************************
* Timeout timer
*/
void TIM3_IRQHandler(void)
{
if (mpx_state==MPX_TRANSMIT)
	{
	MPX_PORT->ODR^=MPXTX_PIN;		//change output state
	//TIM3->CNT=0;
	TIM3->ARR=mpx_timer_tx[mpx_timer_ptr_tx++];//Set time length
	if (mpx_timer_ptr_tx>mpx_timer_cnt_tx)//If all bits are transmitted, 
		{
		MPX_PORT->BRR=MPXTX_PIN;		//Output to high level
		TIM_Cmd(TIM3, DISABLE);		//then transmit/timeout interrupt disable
		TIM_ClearITPendingBit (TIM2, TIM_IT_CC3);
		TIM_ITConfig (TIM2, TIM_IT_CC3, ENABLE);// Enable the interrupt for CC3 (recieve)
		mpx_state=MPX_IDLE;
		}
	}
else if (mpx_state==MPX_RECEIVE)
	{
	//bus.mpx_timeout_handler();
	
	mpx_timer_rx[mpx_timer_ptr_rx]=65535;
	TIM_Cmd(TIM3, DISABLE);
	TIM2->CCER |= TIM_CCER_CC3P;
	/*if (mpx.sync==true) *///MPX_ParsePacketTest();
	//bus.front = MPX_INITIAL_FRONT;
	//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
	if (mpx_timer_ptr_rx>15) MPXParseTimerArray();
	else mpx_state=MPX_IDLE;
	mpx_timer_ptr_rx=0;
	}

TIM_ClearFlag(TIM3, TIM_FLAG_Update);
}



/*******************************************************************************
* Parse the received stream from the array timer
*/
void MPXParseTimerArray(void)
{
//Find minimum length of bit in array
uint16_t bit_length_tmp=65535;
for (uint16_t i=1; i<mpx_timer_ptr_rx; i++) if (bit_length_tmp>mpx_timer_rx[i]) bit_length_tmp=mpx_timer_rx[i];
uint16_t min=bit_length_tmp;//-bit_length_tmp/50;//Min = val -2%

//Convert of the length bit array in of the count bit array
for (uint16_t i=0; i<mpx_timer_ptr_rx; i++)
	{
	uint8_t tmp = (mpx_timer_rx[i]+min/2)/min;
	if (tmp==6) bit_length_tmp=mpx_timer_rx[i]/6;
	mpx_timer_rx[i]=tmp;
	}

//Clear Staffing bits
uint8_t bits_block_count=0;
uint8_t bits_block[MPX_BUFF_LEN_MAX];
clearmemory(bits_block, MPX_BUFF_LEN_MAX);
for (uint16_t i=1; i<mpx_timer_ptr_rx; i++)
	{
	if ((mpx_timer_rx[i-1]==5)&&(mpx_timer_rx[i]==1)) bits_block[bits_block_count-1]+=mpx_timer_rx[++i];
	else if ((mpx_timer_rx[i-1]==5)&&(mpx_timer_rx[i]> 1)) bits_block[bits_block_count++]=mpx_timer_rx[i]-1;//This is correct???
	else bits_block[bits_block_count++]=mpx_timer_rx[i];
	}

//Convert bits array to bytes array
uint8_t bytes_array[18]={0};
uint8_t byte_num=0;
uint8_t bit_num=0x80;
uint8_t bit_polarity=((bits_block[0]&0x01)==1)?0x00:0x01;	//initial bit polarity

for (uint8_t i=1; i<bits_block_count; i++)	//parse all bits blocks, exclude SOF
	{
	for (uint8_t j=0; j<bits_block[i]; j++)	//parse all bits from current bits block
		{
		if (bit_num==0)
			{
			bit_num=0x80;
			if (++byte_num>17) break;	//If the packet length is overflow
			bytes_array[byte_num]=0;
			}
		if (bit_polarity) bytes_array[byte_num]|=bit_num;
		bit_num>>=1;
		}
	bit_polarity^=0x01;
	}

//Check and copy received data in to MPX data struct
TSmpx * mpx_packet_ptr;
mpx_packet_ptr = (TSmpx *) bytes_array;

//if (bytes_array[mpx_packet_ptr->ML+2]==0x7e)	//If "End Of Frame" received
	{
	if (bytes_array[mpx_packet_ptr->ML+1]!=crc8(bytes_array, mpx_packet_ptr->ML+1))//If crc is not Ok -> return
		{
		mpx_state=MPX_IDLE;
		return;
		}
	copymemory((unsigned char *) &mpx_packet, bytes_array, mpx_packet_ptr->ML+1);
	mpx_packet.crc=bytes_array[mpx_packet_ptr->ML+1];
	mpx_state=MPX_RECEIVED;		//Set the end of received packet status
	bit_length=bit_length_tmp;
	if (MPX_RX_Handler!=0) MPX_RX_Handler(mpx_packet);
	}
//else mpx_state=MPX_IDLE;
}



void MPXStartTransmit(TSmpx *data)
{
//Wait to receive complette
//while (mpx_state!=MPX_RECEIVED);
//Calc CRC
data->crc=crc8((unsigned char *) data, data->ML+1);
data->data[data->ML-2]=data->crc;
//Convert data struct to bits array and add Staffing bits
uint8_t bits_array[MPX_BUFF_LEN_MAX];
clearmemory(bits_array, MPX_BUFF_LEN_MAX);
uint8_t bits_ptr=0;
uint8_t *data_ptr=(uint8_t *)data;
bool cur_bit, last_bit=(data_ptr[0]&0x80)?true:false;

for (uint8_t i=0; i<data->ML+2; i++)		//Convert all bytes
	{
	for (uint8_t b=0x80; b!=0; b=b>>1)		//Convert all bits from byte
		{
		cur_bit=(data_ptr[i]&b)?true:false;//Read current bit
		if (cur_bit==last_bit) bits_array[bits_ptr]++;
		else
			{
			bits_array[++bits_ptr]++;	//Bits changed
			last_bit=cur_bit;
			}
		if (bits_array[bits_ptr]>=5)
			{
			bits_array[++bits_ptr]=1;	//Add Staffing bits
			last_bit=(bool)!cur_bit;
			}
		}
	}
//Add EOM byte
if (cur_bit==false) bits_array[bits_ptr]++;
else bits_array[++bits_ptr]++;
bits_array[++bits_ptr]=6;	//EOM
bits_array[++bits_ptr]=6;	//Timeout
//Convert bits array to timer array
for (uint8_t i=0; i<=bits_ptr; i++) mpx_timer_tx[i]=bits_array[i]*bit_length;
//Wait to receive complette
while (mpx_state==MPX_RECEIVE);

mpx_timer_cnt_tx=bits_ptr;
mpx_timer_ptr_tx=0;

mpx_state=MPX_TRANSMIT;

TIM_ITConfig (TIM2, TIM_IT_CC3, DISABLE);//Disable receive

TIM_SetAutoreload(TIM3, bit_length);
MPX_PORT->BSRR=MPXTX_PIN;		//change output state to high (send SOF bit)
TIM_Cmd(TIM3, ENABLE);

/*TIM_ClearITPendingBit (TIM2, TIM_IT_CC4);
// put the counter into a known state (start bit)
TIM2->CCR4+=bit_length;
MPX_PORT->BSRR=MPXTX_PIN;		//change output state to high (send SOF bit)
// Enable the interrupt for CC4 (enable to start transmit)
TIM_ITConfig (TIM2, TIM_IT_CC4, ENABLE);
// Disable the interrupt for CC3 (disable recieve)
TIM_ITConfig (TIM2, TIM_IT_CC3, DISABLE);*/
}




/**************************** END of my source code ***************************/



void MPX_ParsePacketTest(void)
{
bus.mpx_bit_handler = mpx_sof; 
for (uint8_t i=0; i<mpx_timer_ptr_rx; i++)
	{
	bit_coef = mpx_timer_rx[i];

  //If received bits is short than "noise_setpoint", then this is noise
  if( (unsigned int)bit_time < (unsigned int)bus.noise_setpoint ){
    return;
  }
  bus.front = (~bus.front)&1;
  

//Calculate bit
/*  bit_coef = ((unsigned long)bit_time * 100) / bus.startbit_length;
  bus.bit_latch = bit_latch;*/
  
  bits=-bus.bit_stuff;
  bus.bit_stuff=0;
//Calculate the number of bits
  if( bit_coef < bus.bit_coef[0] ){
    bits += 1;
  }else if( bit_coef < bus.bit_coef[1] ){
    bits += 2;
  }else if( bit_coef < bus.bit_coef[2] ){
    bits += 3;    
  }else if( bit_coef < bus.bit_coef[3] ){
    bits += 4;
  }else if( bit_coef < bus.bit_coef[4] ){
    bits += 5;
    bus.bit_stuff=1;
  }else if( bit_coef < bus.bit_coef[5] ){
    bits += 6;    
  }
//If not received correct bit
  if( !bits) {
    bus.bit = (~bus.bit)&1;
    return;
  }
//Parse received bits
  while( bits-- ){
    bus.mpx_bit_handler();
  }
//If received EOM byte
  if( bus.eom ){
     bus.bit_stuff=0;
  }
  
  bus.bit = (~bus.bit)&1;
 
  //If the end of the package is received
  if( bus.end ){
    if( inbox_bean(&inbean) ){
      bus.lost_count++;
      if( bus.lost_count > 20 ){
        bus.lost_count = 0;
//1 - FALLING
//0 - RISING
        bus.front = MPX_INITIAL_FRONT;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity

        mpx_initial_falling_front();
	   return;
      }
    }else{
      bus.lost_count = 0;      
    }
    mpx_start_transmit();
  }else{
  }
	}
}



/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* Синхронизация */
/* Waiting timeout, finding end of packet */
/*****************************************************************************/
/*****************************************************************************/
void mpx_initial_falling_front(void){
  bus.mpx_state = MPX_STATE_RX_SYNC;
  bus.bit_time_stat_ctr = 0;
  bus.startbit_length = 65535;
  bus.mpx_sync_handler = mpx_reset_timeout;		//Reset timeout counter when receiving each bit
  bus.mpx_timeout_handler = mpx_setup_measure_bit;//waiting timeout, for change mpx_sync_handler to mpx_measure_bit_begin

//Timeout timer enable
TIM3->CR1 |= TIM_CR1_CEN;//TIM_Cmd(TIM3, ENABLE);
}



/**
* Reset timeout counter
*/
void mpx_reset_timeout(void){
/*if( !bus.mpx_active ){
	bus.mpx_timeout_handler = mpx_dummy;
	bus.mpx_sync_handler = mpx_dummy;
	}*/
TIM3->CR1&=~TIM_CR1_CEN;	//Switch off timer
TIM3->CNT = 0;			//Reset timeout counter
TIM3->CR1|=TIM_CR1_CEN;	//Start the timer
}



/**
* Preparing to measure length of first bit in packet
*/
void mpx_setup_measure_bit(void){
  bus.front = (~bus.front)&1;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
  
  bus.mpx_sync_handler = mpx_measure_bit_begin;
  bus.mpx_timeout_handler = mpx_dummy;

}



/**
* Begin to measure length of bit
*/
void mpx_measure_bit_begin(void){
  bus.front = (~bus.front)&1;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity

bus.bit_latch = TIM2->CCR3;	//Read timer capture value for first edge

  bus.mpx_sync_handler = mpx_measure_bit_end;
}



/**
* End measure length of bit. Verification is correct length.
*/
void mpx_measure_bit_end(void){
  unsigned int  bit;
  
  bus.front = (~bus.front)&1;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity

bit = TIM2->CCR3;		//Read timer capture value for second edge
  bit -= bus.bit_latch;	//Calculate length of bit 
  if( bit < bus.startbit_length ){//Find bit with minimal length
    bus.startbit_length = bit;
  }
    
  mpx_reset_timeout();
  
  //Check, while count < MPX_BIT_TIME_RETRIES
  if( bus.bit_time_stat_ctr++ >= MPX_BIT_TIME_RETRIES ){
    bus.noise_setpoint = ((unsigned long)bus.startbit_length * 70) / (unsigned long)100;
    //1 - FALLING
    //0 - RISING
    bus.front = MPX_INITIAL_FRONT;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
    //If measured length of bit is not correct
    if( ((unsigned int)bus.startbit_length < (unsigned int)1000) || ((unsigned int)bus.startbit_length > (unsigned int)2500)){
      mpx_initial_falling_front();
    }else{
	//Preparing to start transmitting
      bus.mpx_sync_handler = mpx_reset_timeout;		//Reset timeout if bit received
      bus.mpx_timeout_handler = mpx_start_transmit;	//start transmit after timeout
    }
    
  }else{
    bus.mpx_sync_handler = mpx_measure_bit_begin;
  }
    
}  



/**
* 
*/
void mpx_start_transmit(void){
  clearmemory((unsigned char*)&inbean,sizeof(struct mpx_bean));

  if( bus.mpx_active ){
    bus.mpx_state = MPX_STATE_RX_SYNCHED;
  
    bus.mpx_timeout_handler = mpx_send_idle;
    bus.mpx_sync_handler = mpx_start_bit;
    //1 - FALLING
    //0 - RISING
    bus.front = (~MPX_INITIAL_FRONT) & 1;
    
//                 CEN
//    TIM3->CR1 &= ~(1<<0);//Timeout Counter disable


/*    TIM3->ARRH = (unsigned char)(MPX_IDLE >> 8);
    TIM3->ARRL = (unsigned char)(MPX_IDLE & 255);*/
//TIM2->ARR=(uint16_t)MPX_IDLE;

//                CEN
//    TIM3->CR1 |= (1<<0);//Counter enable


//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
//                CEN
    //TIM1->CR1 |= (1<<0);//Counter enable
  }else{
    bus.mpx_state = MPX_STATE_IDLE;
    bus.mpx_timeout_handler = mpx_dummy;
    bus.mpx_sync_handler = mpx_dummy;
	
  }
}



void mpx_setup_lost_sync(void){
  bus.lost_count++;
  if( bus.lost_count > 20 ){
    bus.lost_count = 0;
  //1 - FALLING
  //0 - RISING
    bus.front = MPX_INITIAL_FRONT;
//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
    
    mpx_initial_falling_front();
  }else{  
    mpx_start_transmit();
  }
}



/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* Прием */               
/* Preparing to receive, initialize data to default values */
/*****************************************************************************/
/*****************************************************************************/
void mpx_start_bit(void){
  mpx_reset_timeout();
  
bus.bit_latch = TIM2->CCR3;


//Set initial values
  //1 - FALLING
  //0 - RISING
  bus.front = (MPX_INITIAL_FRONT);
  bus.mpx_sync_handler = mpx_stream;
  bus.mpx_timeout_handler = mpx_setup_lost_sync;  
  bus.mpx_bit_handler = mpx_sof;  
  bus.receive_bits = 1;
  bus.byte_index=0;
  bus.bit = 1;   
  bus.bit_stuff =0;
  bus.end = 0;
  bus.eom = 0;
  
//Timeout timer enable
TIM3->CR1 |= TIM_CR1_CEN;

//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
}


/**
* Receive streaming data
*/
#ifdef DEBUG
uint16_t bit_array[1024];
uint16_t  bit_ptr=0;
#endif
void mpx_stream(void){
  mpx_reset_timeout();

bit_time = TIM2->CCR3;
  bit_latch = bit_time;
  if (bit_time>bus.bit_latch) bit_time -= bus.bit_latch;
  else bit_time -= bus.bit_latch+0x10000;

//If received bits is short than "noise_setpoint", then this is noise
  if( (unsigned int)bit_time < (unsigned int)bus.noise_setpoint ){
    return;
  }
  bus.front = (~bus.front)&1;
  
//bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity
  

//Calculate bit
  bit_coef = ((unsigned long)bit_time * 100) / bus.startbit_length;
  bus.bit_latch = bit_latch;
#ifdef DEBUG
bit_array[bit_ptr++]=bit_coef;
if (bit_ptr>=1024) bit_ptr=0;
#endif
  
  bits=-bus.bit_stuff;
  bus.bit_stuff=0;
//Calculate the number of bits
  if( bit_coef < bus.bit_coef[0] ){
    bits += 1;
  }else if( bit_coef < bus.bit_coef[1] ){
    bits += 2;
  }else if( bit_coef < bus.bit_coef[2] ){
    bits += 3;    
  }else if( bit_coef < bus.bit_coef[3] ){
    bits += 4;
  }else if( bit_coef < bus.bit_coef[4] ){
    bits += 5;
    bus.bit_stuff=1;
  }else if( bit_coef < bus.bit_coef[5] ){
    bits += 6;    
  }
//If not received correct bit
  if( !bits) {
    bus.bit = (~bus.bit)&1;
    return;
  }
//Parse received bits
  while( bits-- ){
    bus.mpx_bit_handler();
  }
//If received EOM byte
  if( bus.eom ){
     bus.bit_stuff=0;
  }
  
  bus.bit = (~bus.bit)&1;
 
  //If the end of the package is received
  if( bus.end ){
  //if (MPX_CalcCRC(inbean)==inbean.crc)
    if( inbox_bean(&inbean) ){
      bus.lost_count++;
      if( bus.lost_count > 20 ){
        bus.lost_count = 0;
//1 - FALLING
//0 - RISING
        bus.front = MPX_INITIAL_FRONT;

bus.front?(TIM2->CCER |= TIM_CCER_CC3P):(TIM2->CCER &=~ TIM_CCER_CC3P);	//change capture channel 3 edge polarity

        mpx_initial_falling_front();
	   return;
      }
    }else{
      bus.lost_count = 0;      
    }
    //mpx_start_transmit();
  }else{
  }
}



void mpx_sof(void){
  bus.receive_bits--;
  bus.receive_bits = 8;  
  bus.mpx_bit_handler = mpx_pri_ml;
}



void mpx_pri_ml(void){
  inbean.pri_ml<<=1;
  inbean.pri_ml|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.receive_bits = 8;    
    bus.receive_bytes = inbean.pri_ml & 0x0f;
    
    if( bus.receive_bytes < 3 ){
      mpx_setup_lost_sync();
    }else if( bus.receive_bytes > 11 ){
      mpx_setup_lost_sync();
    }else{
      bus.mpx_bit_handler = mpx_dstid;
    }
  }
}



void mpx_dstid(void){
  inbean.dstid<<=1;
  inbean.dstid|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.receive_bits = 8;
    bus.receive_bytes--;
    bus.mpx_bit_handler = mpx_msgid;
  }  
}



void mpx_msgid(void){
  inbean.msgid<<=1;
  inbean.msgid|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.receive_bits = 8;
    bus.receive_bytes--;
    bus.mpx_bit_handler = mpx_data;
  }  
}



void mpx_data(void){
  inbean.data[bus.byte_index]<<=1;
  inbean.data[bus.byte_index]|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.receive_bits = 8;
    bus.receive_bytes--;
    if( bus.receive_bytes==0 ){
      bus.mpx_bit_handler = mpx_crc;
    }else{
      bus.byte_index++;
      bus.mpx_bit_handler = mpx_data;
    }
  }  
}



void mpx_crc(void){
  inbean.crc<<=1;
  inbean.crc|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.receive_bits = 7;
    bus.mpx_bit_handler = mpx_eom;
    bus.eom = 1;
  }  
}



void mpx_eom(void){
  inbean.eom<<=1;
  inbean.eom|=bus.bit;
  
  bus.receive_bits--;
  if( bus.receive_bits==0 ){
    bus.end = 1;
  }  
}



/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* Передача */               
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void mpx_send_silent(void){
    mpx_start_transmit();
}



void mpx_send_idle(void){
  if( outbean_active ) {
    bus.mpx_state = MPX_STATE_TX;

    //GPIOC->ODR |= (1<<5);
    MPX_PORT->BSRR=MPXTX_PIN;	//MPX out to high

TIM3->ARR=(uint16_t) bus.txbit_length;
  
    outbean_bit = outbean_start_with_bit;
    outbean_index =0;
    bus.mpx_sync_handler = mpx_dummy;
    bus.mpx_timeout_handler = mpx_send_stream;
    
  }else{
    mpx_reset_timeout();
    /*ничего*/
  }
}



void mpx_send_stream(void){
  unsigned int	silent_bit;

  if(outbean_index>=outbean_count){
  MPX_PORT->BRR=MPXTX_PIN;	//MPX out to low level
    
    if(outbean_rtr ){
	 outbean_rtr--;
	if( !outbean_rtr ){
	      outbean_active = 0;
     }
    }

    bus.mpx_timeout_handler = mpx_send_silent;
    bus.mpx_sync_handler = mpx_dummy;
//                 CEN
    //TIM3->CR1 &= ~(1<<0);
TIM3->CR1&=~TIM_CR1_CEN;	//Switch off timer

    silent_bit = bus.txbit_length * 5;
TIM2->ARR=(uint16_t)silent_bit;
//                CEN
    //TIM3->CR1 |= (1<<0);
TIM3->CR1|=TIM_CR1_CEN;	//Start the timeout timer

  }else{
    if( outbean_bit ){
    MPX_PORT->BSRR=MPXTX_PIN;	//MPX out to high
    }else{
    MPX_PORT->BRR=MPXTX_PIN;	//MPX out to low
    }
    outbean_bit = (~outbean_bit)&1;
  
    bit_time = (unsigned long)bus.txbit_length * (unsigned long)outbean_bits[outbean_index];

TIM3->ARR=(uint16_t) bit_time;
    
    outbean_index++;
  }
  
}



void mpx_dummy(void){
}



unsigned char MPX_CalcCRC(struct mpx_bean data)
{
unsigned char buf[14];
buf[0]=data.pri_ml;
buf[1]=data.dstid;
buf[2]=data.msgid;
uint8_t len = (data.pri_ml&0x0f)-2;//Data length (without DST-ID & MES-ID)
if (len<=11) for (uint8_t i=0; i<len; i++) buf[3+i]=data.data[i];
else return 0;
//return crc8(buf, len+3);
//return crc8_8540(buf, len+3);
}