
#ifndef NRF_DATA_H

#define NRF_DATA_H


#define NRF_DATA_PACKET_INT 12
#define NRF_DATA_PACKET_CHAR NRF_DATA_PACKET_INT*2
#define NRF_DATA_PACKET NRF_DATA_PACKET_CHAR
union NrfBuff
{
  uint8  Data_char[NRF_DATA_PACKET_CHAR];                                       //Data_char[24]
  int16  Data_int[NRF_DATA_PACKET_INT];                                         //int16 index 0~11 in union NrfBuff(Data_int[12])
};
typedef union NrfBuff NrfBuff;



#endif  //NRFDATA_H