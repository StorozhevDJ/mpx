
#include "MPX\mpx.h"







void send_str(TSmpx *data);


//Test receive MPX
TSmpx testbean[100];
uint16_t ptr=0;
//If MPX packet received
void MPXTest(TSmpx bean)
{
testbean[ptr]=bean;
if (++ptr>=100) ptr=0;
}



void UART_TXTestInit(void)
{
RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

//GPIO Tx
GPIO_InitTypeDef port;
GPIO_StructInit(&port);
port.GPIO_Mode = GPIO_Mode_AF_PP;
port.GPIO_Pin = GPIO_Pin_12;
port.GPIO_Speed = GPIO_Speed_2MHz;
GPIO_Init(GPIOC, &port);

//Init UART
USART_InitTypeDef uart_struct;
uart_struct.USART_BaudRate            = 172800;
uart_struct.USART_WordLength          = USART_WordLength_8b;
uart_struct.USART_StopBits            = USART_StopBits_1;
uart_struct.USART_Parity              = USART_Parity_No ;
uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
uart_struct.USART_Mode                = USART_Mode_Tx;
USART_Init(UART5, &uart_struct);
//Enable UART
USART_Cmd(UART5, ENABLE);
}



//Send MPX data struct to UART
void send_str(TSmpx *data) {
uint8_t *ptr=(uint8_t *)data;

for (uint8_t i=0; i<sizeof(TSmpx); i++)
	{
	uint8_t ch_h=*ptr>>4;
	uint8_t ch_l=*ptr&0x0f;
	ptr++;
	while(!(UART5->SR & USART_SR_TC));
	UART5->DR=ch_h<10?(ch_h|0x30):(ch_h+0x37);
	while(!(UART5->SR & USART_SR_TC));
	UART5->DR=ch_l<10?(ch_l|0x30):(ch_l+0x37);
	while(!(UART5->SR & USART_SR_TC));
	UART5->DR=' ';
	}
while(!(UART5->SR & USART_SR_TC));
UART5->DR='\r';
while(!(UART5->SR & USART_SR_TC));
UART5->DR='\n';
}



int main()
{
MPX_RX_Handler=&MPXTest;
mpx_init();

UART_TXTestInit();

__enable_interrupt();

//Waiting to receive packet (for sync)
while (ptr==0);


TSmpx data={
.ML=8,
.PRI=2,
.DES_ID=0xFE,
.MES_ID=0x7F,
//.data={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
.data={0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
};

for (uint8_t i = 0; i<4; i+=2)
	{
	for (char j=1; j>0; j<<=1)
		{
		data.data[i]=j;
		MPXStartTransmit(&data);
		for (uint32_t t=0; t<5000000; t++) __no_operation();
		}
	}

data.MES_ID=0x2C;
for (uint8_t i = 0x80; i<255; i++)
	{
	data.data[0]=i;
	MPXStartTransmit(&data);
	for (uint32_t t=0; t<100000; t++) __no_operation();
	}

data.MES_ID=0x40;
for (uint8_t i = 1; i>0; i<<=1)
	{
	data.data[0]=i;
	MPXStartTransmit(&data);
	for (uint32_t t=0; t<5000000; t++) __no_operation();
	}


while(1)
	{
	if (ptr) send_str(&testbean[--ptr]);
	}
}
