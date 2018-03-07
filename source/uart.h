/*
 * plc.h
 *
 *  Created on: Mar 2, 2015
 *      Author: admin
 */

#ifndef UART_H_
#define UART_H_

void Copy_InverterData(void);
void CRC_calc(void);
void Mains_Write_portConfig(void);
void Mains_Read_PortConfig(void);
void PLC_sendComplete(void);
void cr_data_16bit_buf(void);
void sci_fifo_rx_1bit_int(void);
void sci_fifo_tx_1bit_int(void);
void sci_fifo_tx_16bit_int(void);
void sci_fifo_rx_16bit_int(void);
void ST7540_PinConfig(void);
interrupt void cpu_timer0_isr(void);
void oi_uart(void);
void power_start (void);

interrupt void sciTxFifoIsr(void);
interrupt void sciRxFifoIsr(void);

interrupt void GPRSTxFifoIsr(void);
void scia_fifo_init(void);

unsigned int crc16( unsigned char *, unsigned char);
unsigned int check_received_data(void);
/*
void spi_fifo_tx_1bit_int(void);
void spi_fifo_tx_16bit_int(void);
void spi_fifo_rx_1bit_int(void);
void spi_fifo_rx_16bit_int(void);
*/

// PLC UNUSED VARIABLES BUT USEFUL FOR FUTURE
extern unsigned int PLC_MODE,send_ack ;
extern volatile unsigned int send_received_val,prev_Rcvd_Data ;

//13b69f
//extern unsigned int CR_data1,CR_data2;
//extern unsigned int tx_data[35],tx_data0,tx_data1,tx_data2, tx_data3;
//extern unsigned int CR_Read,Cr_data_receive,isr_count ,Rcvd_Data[5] ,Header_detect ,a_check ,a_count , Read_16bit_data , Data_count;
////extern volatile unsigned int CR_dataread_temp2,End_of_data, pos,Cr_write_failure , data_reception_successful ,CRC_Val ,rx_crc_val ;
////extern unsigned int CR_dataread_temp, CR_datawrite,  CR_dataread_temp1;
////extern volatile unsigned long int CR_dataread;
//extern volatile unsigned int  Tx_Sent ,Rx_Received,Cr_attempt ,frame_Received_count,rx_data_correct;
//extern unsigned char tx_data_temp[10];
//extern volatile unsigned int  rx_receiving ;

//18022015
extern volatile unsigned int  CR_Init , PLC_Ready, Frame_Size,Check_Frame_Size ,Frame_Received ;

/*Baki*/

extern volatile unsigned int Copy_Data_Flag;

extern volatile unsigned int CRC_calc_over;

extern unsigned int Tx_sent_Success;
extern volatile unsigned int Tx_Init ;

#define CR_Write    0
#define CR_read	    1
#define Mains_Write	2
#define Mains_Read	3

//24022015
extern volatile unsigned int PLC_ReInit_counter;

////////////02-01-2015


//volatile unsigned int s_Counter = 0,Timeout_flag = 0;

//26022015

extern volatile unsigned int  Tx_timeout,Rx_timeout,Tx_timeout_enable,Rx_timeout_enable;

extern volatile unsigned int PLC_ACKNOWLEDGEMENT_RECEIVED ,PLC_RX_RECEIVED , PLC_TX_BEGIN  , PLC_CR_INIT  ;

#endif /* PLC_H_ */
