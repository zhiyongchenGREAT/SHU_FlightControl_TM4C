
#ifndef UART_DATA_H

#define UART_DATA_H

#define UART_DATA_PACKET_INT             15
#define UART_DATA_PACKET_CHAR            UART_DATA_PACKET_INT*2
#define UART_DATA_PACKET                 UART_DATA_PACKET_CHAR
union UART_Buff
{
  uint8  Data_char[UART_DATA_PACKET_CHAR];
  int16  Data_int[UART_DATA_PACKET_INT];
};
typedef union UART_Buff UART_Buff;



#endif  //NRFDATA_H