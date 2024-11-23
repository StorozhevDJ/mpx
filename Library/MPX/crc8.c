//==============================================================================
//
//==============================================================================
//#define CRC8POLY    0x18              // 0X18 = X^8 + X^5 + X^4 + X^0

//==============================================================================
//
//==============================================================================
int CRC8_Calc(char poly, char * data, unsigned int col_bytes)
{ unsigned int   i;
  unsigned char  crc, bit_counter, b, feedback_bit;

  crc = 0x00; // #define CRC8INIT    0x00

  for(i = 0; i != col_bytes; i++)
  { b = data[i];
    bit_counter = 8;
    do 
    { feedback_bit = (crc ^ b) & 0x01;	
      if(feedback_bit == 0x01) { crc = crc ^ poly; }
      crc = (crc >> 1) & 0x7F;
      if(feedback_bit == 0x01) { crc = crc | 0x80; }		
      b = b >> 1;
      bit_counter--;
    } while(bit_counter > 0);
  }
  return (int) crc;
}


#define CRC8INIT    0x00
#define CRC8POLY    0x13
              //    0x18 = ( 0x130 >> 1 ) & 0x7F
unsigned short crc8_8540( unsigned char *data_in, unsigned char bytes )
{
unsigned char crc = CRC8INIT;
unsigned short bit_cnt;
register unsigned char data;
register unsigned char feedback_bit;

    while( bytes-- )
    {   data = *data_in++;

        for( bit_cnt=8; bit_cnt; bit_cnt-- )
        {
            feedback_bit = (data^crc) & 0x01;
            crc  >>= 1;
            data >>= 1;
            if( feedback_bit )
                crc ^= ((CRC8POLY >> 1)|0x80);
        }
    }
    return( crc );
}

/*#define CRC8INIT    0x00
#define CRC8POLY    0x18              //    X^8+X^5+X^4+X^0
              //    0x18 = ( 0x130 >> 1 ) & 0x7F
bint crc8_8540( BYTE *data_in, bint bytes )
{
BYTE crc = CRC8INIT;
bint bit_cnt;
register BYTE data;
register BYTE feedback_bit;

    while( bytes-- )
    {   data = *data_in++;

        for( bit_cnt=8; bit_cnt; bit_cnt-- )
        {
            feedback_bit = (data^crc) & 0x01;
            crc  >>= 1;
            data >>= 1;
            if( feedback_bit )
                crc ^= ((CRC8POLY >> 1)|0x80);
        }
    }
    return( crc );
}*/