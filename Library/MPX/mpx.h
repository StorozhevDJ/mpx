#ifndef MPX_H
#define MPX_H


#define DEBUG


#define MPX_PORT	GPIOB
#define MPXRX_PIN	GPIO_Pin_10	//MPX TX line, MCU input pin
#define MPXTX_PIN	GPIO_Pin_11	//MPX RX line, MCU output pin

#define MPX_BUFF_LEN_MAX	160



/**
Old code
*/
#define MPX_BIT_TIME_RETRIES    40

//Задаем фрон синхронизации, переход из активнеого состоянии линии в idle
//1 - FALLING
//0 - RISING
#define MPX_INITIAL_FRONT       1

#define MPX_STATE_RX_SYNC	0
#define MPX_STATE_RX_SYNCHED	1
#define MPX_STATE_TX		2
#define MPX_STATE_IDLE		3

struct mpx_bus{
  void (*mpx_sync_handler)(void);
  void (*mpx_timeout_handler)(void);
  void (*mpx_bit_handler)(void);
  
  unsigned int  startbit_length;
  unsigned int  bit_latch;
  unsigned int  noise_setpoint;
  unsigned int  txbit_length;

  unsigned int  bit_coef[6];

  unsigned char bit_time_stat_ctr;
  unsigned char receive_bits;
  unsigned char receive_bytes;
  unsigned char byte_index; 
  unsigned char bit;
  unsigned char front;
  unsigned char eom;
  unsigned char end;
  unsigned char lost_count;
  unsigned char bit_stuff;
  unsigned char mpx_active;
  unsigned char mpx_state;
  unsigned char msec;
};

struct mpx_bean{
  unsigned char pri_ml;
  unsigned char dstid;
  unsigned char msgid;
  unsigned char data[9];
  unsigned char crc;
  unsigned char eom;
  unsigned char rsp;
  unsigned char eof;
};
/*
End Old code
**/



#pragma pack(push, 1)
typedef struct
	{
	uint8_t ML:4;	//Message length
	uint8_t PRI:4;	//Priority
	uint8_t DES_ID;//Destance ID
	uint8_t MES_ID;//Message ID
	uint8_t data[11];
	uint8_t crc;
	} TSmpx;
#pragma pack(pop)


typedef enum
	{
	MPX_IDLE=0,
	MPX_RECEIVE,
	MPX_RECEIVED,
	MPX_TRANSMIT
	}TEMPXstate;


extern TSmpx mpx_packet;		//Received data in MPX packet struct


//Инициализация
void mpx_init(void);
//Pointer to callback function MPX RX Handler (optional use)
extern void (* MPX_RX_Handler) (TSmpx data);
//Check is MPX data received
bool IsMPXReceived(void);
//Start transmit
void MPXStartTransmit(TSmpx *data);


void mpx_dummy(void);
void mpx_initial_falling_front(void);

unsigned char MPX_CalcCRC(struct mpx_bean data);



#endif//MPX_H



/**
MyLog (bit lenght is ~0.1ms)
100101000111110110011111011000100000100000100000100000100000100000100000100000100001110100101111110000000
100101000111110110011111011000100000100000100000100000100000100000100000100000100001110100101111110000000000

100100011111011110110101111000001001111001001111110000000000000
1 21  3    51   41 2111   4    51 2   4 21 2     5 

1001001011111011101000001110110000010000011000001100000101101000111111

Parse
1   0010 1000 11111110 01111111 00010000 00000000 00000000 00000000 00000000 00000000 11101001 01111110 00  000000 0
SOF PRI  ML   DST-ID   MES-ID   Data0    Data1    Data2    Data3    Data4    Data5    CRC      EOM      RSP EOF    IFS
01  02   08   FE       7F       10       00       00       00       00       00       E9       7E       00  00     00

1   0010 0011 11111110 11010111 10000000 11110010 01111110 00  000000 0   000
SOF PRI  ML   DEST-ID  MES-ID   Data0    CRC      EOM      RSP EOF    IFS 
01  02   03   FE       D7       80       F2       7E       00  00     00

1   0010 0101 11111110 10000011 01100000 00000100 00010000 00110100 01111110 00  000000 0
SOF PRI  ML   DEST-ID  MES-ID   Data0    Data1    Data2    CRC      EOM      RSP EOF    IFS
01  02   05   FE       83       60       04       10       34       7E       00  00     00
*/


