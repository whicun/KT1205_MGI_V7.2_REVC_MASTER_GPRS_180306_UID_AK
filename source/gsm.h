/*
 * rs_485.h
 *
 *  Created on: Feb 3, 2016
 *      Author: kripya
 */

#ifndef KT1205_MGI_DMI_V3_1_170314_GSM_COMMUNOICATION_RS_485_H_
#define KT1205_MGI_DMI_V3_1_170314_GSM_COMMUNOICATION_RS_485_H_

#define MAX_FRAME_DATA_SIZE 256//10			//NO OF 16 BIT DATAS // multiples of two 8 bit
#define BUFFER_SIZE         150				        //Should be more than 2x size of data size

#define Device_ID 0x01					 //DEVICE ID
#define FRAME_HEADER 0xAABB				//COMM FRAME DETECT HEADER

//#define INITIALIZATION_1 0xAT+S
void transmit_dat(char nDATA, char Tx_DEVICE_ADDRESS);
void tx_gsm(void);
void clear_Rx_frame(void);
void clear_Tx_frame(void);

char makedigit (int *number, int base);
char *makestring (int number);
char *makestring1 (long int number1);

char scia_xmit(char a);
char scia_msg(char *msg);

unsigned int CRC16( char * pucFrame, char len );

__interrupt void sciaTxFifoIsr(void);
__interrupt void sciaRxFifoIsr(void);

void GPRS_SCI_Init(void);

struct BYTE
{
	char Lo_byte:8;
	char Hi_byte:8;
};

union DATA_BIT16
{
	unsigned int BIT_16;
	struct BYTE byte ;
};

#endif /* KT1205_MGI_DMI_V3_1_170314_GSM_COMMUNOICATION_RS_485_H_ */
