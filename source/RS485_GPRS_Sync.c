//Note : GPIO 11 is replaced with GPIO17 FOR sync pcb Testing

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "gsm.h"

#include "DSP28x_Project.h"
#define POLY 0xA001
#include "uart.h"
#include "Solar Microinverter_defines.h"
//#include "inpromation.h"

//RAMFUNCS Declaration
//#pragma CODE_SECTION(sciRxFifoIsr, "ramfuncs");
//#pragma CODE_SECTION(sciTxFifoIsr, "ramfuncs");
//#pragma CODE_SECTION(oi_uart, "ramfuncs");

#define UID 0x1A3B5C  //1719132

unsigned char crc8(unsigned char *data,int length);

void GSM_DataUpdate(void);
void Slaveid_Calc(void);


// GPRS&RS_485 Variable Declaration Starts
int p1=0,i2=0,p2=0;
char  GSM_BreakFlag=0, Read_Index=0;
char *msg, *msg1, *msg2, *msg3, *msg4, *msg5, *msg6, *msg6_1,*msg6_2, *msg7, *msg8, *msg9, *msg10, *msg11, *msg12, *msg_reset ;
char *msg7_0, *msg7_1,*msg7_2, *msg7_3,*msg7_4, *msg7_5, *msg7_6;
char GSM_Tx_pos = 0,Tx_buffer_index = 0,Tx_buffer_index1 = 0;
char msgcount=0,GSM_SyncFlag=0, GSM_CompleteFlag=0, GSM_StartFlag;
char Tx_Frame_len = 0, Rx_Frame_len = 0;
char Tx_FRUP=0,Rx_FRUP=0,Rx_flag =0,Rx_SyncStart=0, Rx_ErrorCount=0;
char TxSlaveAddress=0x71, RxSlaveAddress=0x71, GSM_EnableFlag=0;
unsigned int  rs_485_count=0, GSM_SendUpdateFlag=0, TX_FIFO_Count=0,GSM_LOOP_COUNT=6000;
int Buffer0=0,Buffer1=0, Buffer2=0, Buffer3=0, Buffer4=0, Buffer5=0, Buffer6=0, Buffer7=0;
long int Buffer8=0;
int gsm_soft_reset_count=50, Rx_Ready=0 ,tx_pack_flag=0, tx_pack_flag1=0;
int GSM_TX_SendFlag, pos1 = 0, rx_pos = 0,TX_SendFlag = 0, tx_data_test_prev=0 , GSM_EventCount=0;
long int GSM_Repeat_Count_max=200000,GSM_Repeat_Count=0 ;
volatile unsigned int GSM_Restart_Count=23000, GSM_Restart_Count_Max=500;
unsigned int DataDelayCount=0, Tx_DataSend_Flag=0, RX_DataRcvd_Flag=0, GSM_DelayCount=2000;
unsigned int gsm_count=0;
			//Buffer Declaration:
			//unsigned char rx_data_temp[20];
			//unsigned int tx_data[35], Rcvd_Data[5];
			//unsigned int Rcvd_Data[5];
long int Rcvd_Data_Total[4]={0,0,0,0};
unsigned char Send_Tx_Data[20],Rcvd_Rx_Data[20];
unsigned char Rcvd_Rx_Data_1[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char Rcvd_Rx_Data_2[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char Rcvd_Rx_Data_3[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char Rcvd_Rx_Data_4[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char Rcvd_Rx_Data_5[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

char Tx_frame[BUFFER_SIZE],Rx_Buffer[100],Rx_Frame[100], Update_Buffer[200],Tx_buffer[200], tmp[8],tmp1[11], Slave_Run_Status[6]={0,0,0,0,0,0};
// GPRS&RS_485 Variable Declaration Starts

// Other Variables used from External files
extern float AC_inverter_out_ClaCpu;
extern float32 Pcap_power_ClaCpu;
extern int peakInverterOutputVoltage;
extern unsigned char faultState,storeFaultState, switchState, systemState, gridFlag;
extern unsigned int crc_error_couter;
extern unsigned int driveSupplyVoltage , measuredTemperature , referenceVoltage ,inverterOutputCurrent,averageRectifiedCurrent;
extern unsigned int inputVoltageAverage;
extern unsigned int Power_Limit;
extern unsigned int pvPanelVoltage , flybackCurrent1 , flybackCurrent2  ;
extern unsigned int rx_crc_check_data;
extern volatile int inputCurrentAverage,peakGridVoltage, inverterOutputVoltage,gridVoltage;
extern volatile int peakInverterOutputCurrent;

unsigned int  rx_receiving = 0;
unsigned int copy_Total_Current = 0;

unsigned int isr_count ,RS_485_Data_delay;

unsigned int ID = 0;
unsigned int Read_count = 0;
unsigned int Rx_Index=0;
unsigned int Rx_MSB = 0, Rx_LSB = 0,m=0;
unsigned int slaves_EqualFlag = 0;
unsigned int temp_rx_crc_val=0, Tx_Index=0;
unsigned int tx_data0,tx_data1,tx_data2, tx_data3;
unsigned int Tx_sent_Success;
unsigned long Total_Current = 0;
volatile int ack_rcvd_counter = 0;
volatile int controol_enable = 1;
volatile int kicker = 0;

volatile unsigned char startflag = 0;
volatile unsigned int  CR_Init , PLC_Ready, Frame_Size,Check_Frame_Size ,Frame_Received ;
volatile unsigned int  Tx_Sent ,Rx_Received,Cr_attempt ,frame_Received_count,rx_data_correct, RS485_TX_Enable=0,RS485_TXEnable_Count=0,RS485_Start=0 ;
volatile unsigned int  Tx_timeout=0,Rx_timeout=0,Tx_timeout_enable=0,Rx_timeout_enable=0,Rx_time_count=0;
volatile unsigned int all_sent= 0, all_received = 0;
volatile unsigned int Break_Rcvd_Data1_1 = 0, Break_Rcvd_Data1_2 = 0,Break_Rcvd_Data2_1 = 0, Break_Rcvd_Data2_2 = 0, Slave_Current=0, Slave_SystemState=0;
volatile unsigned int Broadcast1 =0, Broadcast2 =0, Broadcast3 =0;
volatile unsigned int Copy_Data_Flag = 1;
volatile unsigned int CR_dataread_temp2,End_of_data =30, pos=24,Cr_write_failure = 0, data_reception_successful ,CRC_Val ,rx_crc_val ,RX_CRC_Val;
volatile unsigned int CRC_calc_over = 0,CRC_calc_start=0,CRC_pos = 0,RX_CRC_calc_over = 0,RX_CRC_calc_start=0,RX_CRC_pos = 0;
volatile unsigned int loadManagerBreakup1 =0, loadManagerBreakup2 =0, loadManagerBreakup3 =0;
volatile unsigned int Master_PC;
volatile unsigned int AddBuf, crc_calc_fail, dummy_data=12;
volatile unsigned int power_star_valga = 0;
volatile unsigned int quick_start_tx = 0;
volatile unsigned int Rx_ID = 0,Tx_ID = 0x0001, faultState_Rs485=0;
volatile unsigned int slaveRunStatus = 0,slaveRunStatus1 = 0,slaveRunStatus2 = 0,slaveRunStatus3 = 0,TotalPLC =0, TotalPLC_Slaves =0,numberOfErrorSlaves =2, numberOfSlaves =2,slavestartcounter=0, SlaveStartDelay =8000, SlaveStartFlag =0;
unsigned crc = 0xFFFF;
unsigned char  Pi1, n;

extern long unsigned int my_Irms_q15,my_Irms_q15_result;


volatile unsigned int uidByte12=0x0000,uidByte34=0x0000,uid1Byte=0x0000,uid2Byte=0x0000,uid3Byte=0x0000,uid4Byte=0x0000;
volatile unsigned long int uidByteConv1=0x0000,uidByteConv=0x0000,incIndex=0;

unsigned  int UID_TxGSM[4] = {0x0000,0x0000,0x0000,0x0000},UID_TxGSM_2[4] = {0x0000,0x0000,0x0000,0x0000},UID_TxGSM_3[4] = {0x0000,0x0000,0x0000,0x0000},UID_TxGSM_4[4] = {0x0000,0x0000,0x0000,0x0000},UID_TxGSM_5[4] = {0x0000,0x0000,0x0000,0x0000};

unsigned long int lresult=0x00000000, MuidByteConv=0x00000000,S1uidByteConv=0x00000000,S2uidByteConv=0x00000000,S3uidByteConv=0x00000000,S4uidByteConv=0x00000000, S5uidByteConv=0x00000000;
unsigned int S1uidByte12=0x0000,S1uidByte34=0x0000,S2uidByte12=0x0000,S2uidByte34=0x0000,S3uidByte12=0x0000,S3uidByte34=0x0000,S4uidByte12=0x0000,S4uidByte34=0x0000,S5uidByte12=0x0000,S5uidByte34=0x0000;


char makedigit2 (unsigned long long  int *number2,unsigned long  int base2);
char *makestring2 (unsigned long long int number2);


//++++++++++++++ Master UART Data Variables Ends ++++++++++++++++++++++++++

// This function is used to convert the incoming string into equivalent character
char makedigit (int *number, int base)
{
  static char map[] = "0123456789";
  int ix;

  for (ix=0; *number >= base; ix++)
  {
      *number -= base;
  }

  return map[ix];
}

char makedigit1 (long int *number1, long int base1)
{
  static char map[] = "0123456789";
  long int ix1;

  for (ix1=0; *number1 >= base1; ix1++)
  {
      *number1 -= base1;
  }

  return map[ix1];
}

// This function is used to convert the incoming string into equivalent character
char *makestring (int number)
{
  tmp[0] = makedigit(&number, 10000);
  tmp[1] = makedigit(&number, 1000);
  tmp[2] = makedigit(&number, 100);
  tmp[3] = makedigit(&number, 10);
  tmp[4] = makedigit(&number, 1);
  tmp[5] = ',';

  return tmp;
}

char *makestring1 (long int number1)
{
  tmp[0] = makedigit1(&number1, 100000);
  tmp[1] = makedigit1(&number1, 10000);
  tmp[2] = makedigit1(&number1, 1000);
  tmp[3] = makedigit1(&number1, 100);
  tmp[4] = makedigit1(&number1, 10);
  tmp[5] = makedigit1(&number1, 1);
  tmp[6] = ',';

  return tmp;
}


char makedigit2 (unsigned long long int *number2,unsigned long  int base2)
{
  static char map[] = "0123456789";
  unsigned long int ix1;

  for (ix1=0; *number2 >= base2; ix1++)
  {
      *number2 -= base2;
  }

  return map[ix1];
}


char *makestring2 (unsigned long long int number2)
{
//  tmp1[0] = makedigit2(&number2, 1000000000);
//  tmp1[1] = makedigit2(&number2, 100000000);
//  tmp1[2] = makedigit2(&number2, 10000000);
//  tmp1[3] = makedigit2(&number2, 1000000);
//  tmp1[4] = makedigit2(&number2, 100000);
//  tmp1[5] = makedigit2(&number2, 10000);
//  tmp1[6] = makedigit2(&number2, 1000);
//  tmp1[7] = makedigit2(&number2, 100);
//  tmp1[8] = makedigit2(&number2, 10);
//  tmp1[9] = makedigit2(&number2, 1);
//
//  tmp1[10] = ',';

    // tmp1[0] = makedigit2(&number2, 1000000000);
     tmp1[0] = makedigit2(&number2, 100000000);
     tmp1[1] = makedigit2(&number2, 10000000);
     tmp1[2] = makedigit2(&number2, 1000000);
     tmp1[3] = makedigit2(&number2, 100000);
     tmp1[4] = makedigit2(&number2, 10000);
     tmp1[5] = makedigit2(&number2, 1000);
     tmp1[6] = makedigit2(&number2, 100);
     tmp1[7] = makedigit2(&number2, 10);
     tmp1[8] = makedigit2(&number2, 1);
     tmp1[9] = ',';


  return tmp1;
}




// Transmit a character from the SCI
char scia_xmit(char a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {}
    SciaRegs.SCITXBUF=a;
    return a;
}

char scia_msg(char * msg)
{
    int i;
    i = 0;

    while(msg[i] != '\0')
    {
    scia_xmit(msg[i]);
    i++;
    }
    return msg[i];
}

// Clear TX & RX Frames

void clear_Rx_frame(void)														//Clears Rx_frame
{
	char j;
	for(j = 0;j < 200; j++)
	{
		Rx_Buffer[j] = 0;
	}
}

void clear_Tx_frame(void)														//Clears Tx_frame
{
	char j;
	for(j = 0;j < BUFFER_SIZE; j++)
	{
		Tx_frame[j] = 0;
	}
}


unsigned char crc8(unsigned char *data,int length)
{
	unsigned long crc;
	int f,bit;

	crc = 0xFF;
	for ( f=0 ; f<length ; f++ ) {
	  crc ^= data[f];
	  for ( bit=0 ; bit<8 ; bit++ ) {
	   if ( (crc&0x80)!=0 ) {
		crc <<= 1;
		crc ^= 0x1D;
	   }
		else
		  crc <<= 1;
	   }
	}
	return (~crc)&0xFF;
}


// UART TX ISR for both RS_485 & GPRS Data Transfer

interrupt void sciTxFifoIsr(void)
{
//	   GpioDataRegs.GPASET.bit.GPIO11      =   1;

    EALLOW;
    IER |= 0x001;                         // Set global priority by adjusting IER
    IER &= 0x001;
    asm("       NOP");                    // Wait one cycle
    EINT;

   // GPRS Data Transfer
    if(GSM_EnableFlag==1)									// Check for GSM Enable Flag triggered from RS_485 Intercommunication Loop
    {
        Tx_timeout = 0;										//  Reset TX Timeout to 0, resposnible for holding Inter communication;
        Rx_timeout   =0;									//  Reset RX Timeout to 0, resposnible for holding Inter communication;

        if (SciaRegs.SCICTL2.bit.TXEMPTY == 1)
         {


            if(GSM_Tx_pos<=(Tx_buffer_index1))				//  Checking for GPRS Buffer Position
            {
                if (SciaRegs.SCIFFTX.bit.TXFFST == 0)
                 {
                    SciaRegs.SCITXBUF = Tx_buffer[GSM_Tx_pos++];		// Sending GPRS complete frame Data to intermediate Buffer
                 }
            }

              else
              {
            	   tx_pack_flag						=	1;

//                   if(GSM_BreakFlag != 0)
                   {
                       GSM_DelayCount        		=   0;
                   }
                   SciaRegs.SCIFFTX.bit.TXFFINT 	= 	0;
                   SciaRegs.SCIFFTX.bit.SCIFFENA   	= 	0;

                   GSM_CompleteFlag 			= 	1;					// Setting GPRS Complete Operation
                   GSM_TX_SendFlag				=	0;					// Disabling GPRS TX Operation
                   GSM_Tx_pos 					= 	0;					// Reset GPRS Pos to Zero

              }

          }

        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    }

    // For RS 485 Data Transfer
    else
    {
		if(pos1 < 21)												// RS_485 Frame packet Intermediate transfer
		{
			if (SciaRegs.SCICTL2.bit.TXEMPTY == 1)
			{
				if(pos1 == 0)
				{
					GpioDataRegs.GPASET.bit.GPIO17        =   1;
				  SciaRegs.SCICTL1.bit.TXWAKE   = 1;
				  SciaRegs.SCITXBUF             = TxSlaveAddress;	  // Sending Slave Address to Buffer for address match
				  pos1++;
				  SciaRegs.SCIFFTX.bit.TXFFINT  = 0;
				  DataDelayCount = 0;
				  RS_485_Data_delay = 1;
				 }
				 else
				{
				  SciaRegs.SCITXBUF      = Send_Tx_Data[pos1-1];		// Buffer update  after address match with rs_485 frame packet
				  pos1++;
				}
			 }
		}
	   else
	   {
		   pos1++;
		   // RS_485 Transmission Complete
		   RS485_TX_Enable                      =   1;

		   // RS_485 Gating Delay for Disabling GPIO17, Serial Buffer End
		   RS485_TXEnable_Count                 =   0;

		   // CLEARING DataRcvd mode
		   RX_DataRcvd_Flag                     =   0;

		   //Enabling SLEEP Mode for RS485 Data Receive detecting slave address
		   SciaRegs.SCICTL1.bit.SLEEP           =   1;
		   SciaRegs.SCIFFRX.bit.RXFFIENA        =   0;
		   SciaRegs.SCIFFTX.bit.TXFFINT         =   0;
		   SciaRegs.SCIFFTX.bit.SCIFFENA        =   0;
		   SciaRegs.SCIFFTX.bit.TXFFIENA        =   0;
	   }
     }
	   SciaRegs.SCIFFTX.bit.SCIFFENA        =   0;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;							// Interrupt clear
    PieCtrlRegs.PIEACK.all |= 0x100;      							// Issue PIE ACK
    DINT;


    //     GpioDataRegs.GPACLEAR.bit.GPIO11      =   1;
}


// UART RX ISR for RS_485 Data Receive from Slaves

interrupt void sciRxFifoIsr(void)
{
        // Added for RS_485 Communication
        Rcvd_Rx_Data[Read_count]= SciaRegs.SCIRXBUF.all & 0x00FF;     // Read data

        EALLOW;
        IER |= 0x001;                         // Set global priority by adjusting IER
        IER &= 0x001;
        asm("       NOP");                    // Wait one cycle
        EINT;
        // checking Buffer from Slave data frame completed
        if (Read_count ==19)
        {
            RX_DataRcvd_Flag                =   1;					// Setting flag for Validating RS_485 Rx Data
            Read_count 						= 	0;					// Resetting Rx frame count to zero
            SciaRegs.SCICTL1.bit.SLEEP      =   1;					// Enabling Sleep Mode
            SciaRegs.SCIFFRX.bit.RXFFIENA   =   0;
            Rx_time_count = 0;
        }
        Read_count++;												// Incrementing Rx frame count
        isr_count++;												// Debug Purpose
    										// Debug Purpose

    //  *******  Clear the interrupt and acknowledge it so that next time from the same group interrupt will come ******
        SciaRegs.SCIFFRX.bit.RXFFOVRCLR =1;  						// Clear Overflow flag
        SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;  						// Clear Interrupt flag
        PieCtrlRegs.PIEACK.all|=0x100;       						// Issue PIE ack
        DINT;

}

// UART Function Initialization

void scia_fifo_init()
{
    MuidByteConv = UID;

    uidByte12=UID & 0xFFFF;
    uid1Byte=(uidByte12)&0x00FF;
    uid2Byte= (uidByte12>>8) & 0x00FF;

    uidByteConv=UID & 0xFFFF0000;
    uidByteConv1= uidByteConv>>8;
    uidByte34= uidByteConv1>>8;
    uid3Byte= (uidByte34)&0x00FF;
    uid4Byte= (uidByte34>>8) & 0x00FF;



    //
    //#define UIDBYTE12 (UID & 0xFFFF)  //LSB 16 Bit
    //
    //#define UID1BYTE ( UIDBYTE12&0x00FF )
    //#define UID2BYTE1 (UIDBYTE12>>8)
    //#define UID2BYTE (UID2BYTE1 & 0x00FF)
    //
    //
    ////MSB 16 Bit
    //#define UIDBYTECONV (UID & 0xFFFF0000)
    //#define UIDBYTECONV1  ( UIDBYTECONV>>8 )
    //
    //#define UIDBYTE34 ( UIDBYTECONV1>>8 )
    //#define UID3BYTE ( UIDBYTE34 &0x00FF )
    //#define UID4BYTE1 ((UIDBYTE34>>8))
    //#define UID4BYTE ( UID4BYTE1 & 0x00FF)



	TxSlaveAddress=0x71;						// Initializing TxSlaveAddress
	RxSlaveAddress=0x71;						// Initializing RxSlaveAddress

	SciaRegs.SCICCR.all             = 0x0007;     // 1 stop bit,  No loopback
												  // No parity,8 char bits,
												  // async mode, idle-line protocol
												  // 8: 1000

	SciaRegs.SCICTL1.all            = 0x0002;     // 0x0003  // enable TX, RX, internal SCICLK,
												  // Disable RX ERR, SLEEP, TXWAKE
												  // B: 1011

	SciaRegs.SCICTL2.all            = 0x0003;	  // Enabling UART in TX&RX Mode

	SciaRegs.SCICTL2.bit.TXINTENA   = 0;		  // Disabling TX INT DURING STARTUP
	SciaRegs.SCICTL2.bit.RXBKINTENA = 0;	      // Disabling RX BK INT DURING STARTUP

	SciaRegs.SCIHBAUD               = 0x0000;
	SciaRegs.SCILBAUD               = 0x00C2;     // 57600:20h//C2h:9600

	SciaRegs.SCIFFTX.all            = 0xE040;     // 0xC020;
	SciaRegs.SCIFFRX.all            = 0x0021;     // 0x0022;
	SciaRegs.SCIFFCT.all            = 0x0;

	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;				// FIFO Reset

	SciaRegs.SCICTL1.all                =   0x0021;     // 0x0023    // Relinquish SCI from Reset
}


// Function for GPRS & RS_485 Data Transfer
void oi_uart(void)
{
	faultState_Rs485= storeFaultState>>1;

	if(GSM_EnableFlag==0)
	{
		if ((SciaRegs.SCICTL2.bit.TXEMPTY == 1)&&(pos1 <= 21))  // Enabling TX pin upto 20 bit transfer
		{
			SciaRegs.SCIFFTX.bit.SCIFFENA        =   1;
			if(pos1 > 2)	RS_485_Data_delay = 0;

		}


		// Gating function for Falling edge for RS_485 Rx Enable --> GPIO17 clear
		// Delay meant for Sending last byte on the RS485 Data Frame
		if(RS485_TX_Enable==1 && RS485_TXEnable_Count>10)
		{
			RS485_TXEnable_Count=0;
			GpioDataRegs.GPACLEAR.bit.GPIO17        =   1;			// Clearing GPIO17 after Data Transfer
			RS485_TX_Enable                         =   0;
			SciaRegs.SCICTL1.bit.SLEEP              =   1;
			SciaRegs.SCICTL1.bit.SWRESET            =   0;
			SciaRegs.SCICTL1.bit.SWRESET            =   1;
			SciaRegs.SCIFFRX.bit.RXFFIENA           =   0;
		}


		//  Data Transfer to slave after setting TX-Send flag : Transfer buffer update of RS_485
				if(TX_SendFlag==1)
				{
					GpioDataRegs.GPASET.bit.GPIO17      =   1;		// Signal for RS-485 TX Enable

					TX_SendFlag 				= 	0;				// RESET FLAG TO 0	for Interlock
					Send_Tx_Data[0]=TxSlaveAddress&0x00FF;      	// Slave Address at beginning of TX Frame for WAKING from SLEEP Mode

					Send_Tx_Data[1]=pvPanelVoltage&0x00FF;        	// Input PV Voltage
					Send_Tx_Data[2]=(pvPanelVoltage>>8)&0x00FF;

					Send_Tx_Data[3]=flybackCurrent1&0x00FF;        	// Input PV Current
					Send_Tx_Data[4]=(flybackCurrent1>>8)&0x00FF;


					if(gridFlag==1)
					{
						Send_Tx_Data[5]=peakGridVoltage&0x00FF;        // Inverter Output Voltage for Grid-Tie
						Send_Tx_Data[6]=(peakGridVoltage>>8)&0x00FF;

						Send_Tx_Data[7]=peakInverterOutputCurrent&0x00FF;                  // Inverter Output current
						Send_Tx_Data[8]=(peakInverterOutputCurrent>>8)&0x00FF;
					}
					if(gridFlag==0)
					{
						Send_Tx_Data[5]=peakInverterOutputVoltage&0x00FF; 		// Inverter Output Voltage for OFF-Gird
						Send_Tx_Data[6]=(peakInverterOutputVoltage>>8)&0x00FF;

						Send_Tx_Data[7]=my_Irms_q15_result&0x00FF;    				// Inverter Output current
						Send_Tx_Data[8]=(my_Irms_q15_result>>8)&0x00FF;
					}


					Send_Tx_Data[9]=systemState&0x00FF;            			// System State
					Send_Tx_Data[10]=faultState_Rs485&0x00FF;           		 	// System Fault State

					Send_Tx_Data[11]=measuredTemperature&0x00FF;       		// Measured Temperature
					Send_Tx_Data[12]=(measuredTemperature>>8)&0x00FF;

                    Send_Tx_Data[13]=uid1Byte;
                    Send_Tx_Data[14]=uid2Byte;
                    Send_Tx_Data[15]=uid3Byte;
                    Send_Tx_Data[16]=uid4Byte;


                    Send_Tx_Data[17]= 0x00FF;
                    Send_Tx_Data[18]= 0x00FA;


					// CRC Value update
					CRC_Val = crc8(Send_Tx_Data,19);
					Send_Tx_Data[19]=CRC_Val;					// CRC Value update on frame-Higher Byte
					CRC_calc_over =0;

					Tx_DataSend_Flag			=	1;				// Delayed Operation of RS_485 Data Transfer
					DataDelayCount			=	0;				// Delayed Operation of RS_485 Data Transfer
					pos1=0;										// Resetting Buffer position to begin Slave buffer frame

					RS485_TX_Enable                     =   0;				// Resetting Enable flag to 0


					// Registers cleared for proper RS_485 Transfer: Int clear , FIFO Reset etc.,
					SciaRegs.SCIFFTX.bit.TXFFINTCLR   =   1;
					SciaRegs.SCIFFTX.bit.TXFIFOXRESET =   0;
					SciaRegs.SCIFFTX.bit.TXFIFOXRESET =   1;
					SciaRegs.SCIFFTX.bit.TXFFIENA     =   1;
					SciaRegs.SCICTL1.bit.TXENA        =   1;
					SciaRegs.SCIFFTX.bit.SCIFFENA     =   1;


				}



				//Step 2: Enable UART in WAKE Mode for RS-485 Data Transfer

				if(SciaRegs.SCIRXST.bit.RXWAKE==1)
				{
					AddBuf= SciaRegs.SCIRXBUF.all & 0x00FF;
					SciaRegs.SCIFFRX.bit.RXFIFORESET   =   0;
					SciaRegs.SCIFFRX.bit.RXFIFORESET   =   1;
					SciaRegs.SCIFFTX.bit.TXFFINTCLR    =   1;
					RxSlaveAddress                     =   AddBuf;

					Read_count=0;                          // Initialized for Buffer Index

					SciaRegs.SCICTL1.bit.SLEEP         =   0;
					SciaRegs.SCIFFRX.bit.RXFFIENA      =   1;
					SciaRegs.SCICTL2.bit.TXINTENA      =   0;
					SciaRegs.SCIFFTX.bit.SCIFFENA      =   1;
				}
				// Timeout for rx , for RS_485 data valiation


				if(RX_DataRcvd_Flag==1)
				{
					if(Rx_timeout > 400) // 20ms +
					{
						Rx_timeout   =0;



						RX_CRC_Val = crc8(Rcvd_Rx_Data,19);
						rx_crc_val  = RX_CRC_Val;

						temp_rx_crc_val  =Rcvd_Rx_Data[19]; // Temporary CRC Value computation from RX Data



						// Validating Slave Data with CRC ,
						//IF Computed CRC & Recevied CRC are equal, then slave Data will be updated to slave buffer
						if(rx_crc_val==temp_rx_crc_val)
						{
							//crc_calc_fail=0;
								// Slave 1 Data Receive
								if(RxSlaveAddress==0x71)				// Address match for Slave 1
								{
								   for(Read_Index=0;Read_Index<=19;Read_Index++)
								   {
									   Rcvd_Rx_Data_1[Read_Index] =Rcvd_Rx_Data[Read_Index];
								   }
								   Slave_Run_Status[1]=1;
								}

								// Slave 2 Data Receive
								else if(RxSlaveAddress==0x72)			// Address match for Slave 2
								{
								   for(Read_Index=0;Read_Index<=19;Read_Index++)
								   {
									   Rcvd_Rx_Data_2[Read_Index] =Rcvd_Rx_Data[Read_Index];
								   }
								   Slave_Run_Status[2]=1;
								}

								// Slave 3 Data Receive
								else if(RxSlaveAddress==0x73)			// Address match for Slave 3
								{
								   for(Read_Index=0;Read_Index<=19;Read_Index++)
								   {

									   Rcvd_Rx_Data_3[Read_Index] =Rcvd_Rx_Data[Read_Index];
								   }
								   Slave_Run_Status[3]=1;
								}

								// Slave 4 Data Receive
								else if(RxSlaveAddress==0x74)  		// Address match for Slave 4
								{
								   for(Read_Index=0;Read_Index<=19;Read_Index++)
								   {

									   Rcvd_Rx_Data_4[Read_Index] =Rcvd_Rx_Data[Read_Index];
								   }
								   Slave_Run_Status[4]=1;
								}

								// Slave 5 Data Receive
								else if(RxSlaveAddress==0x75)		   // Address match for Slave 5
								{
								   for(Read_Index=0;Read_Index<=19;Read_Index++)
								   {

									   Rcvd_Rx_Data_5[Read_Index] =Rcvd_Rx_Data[Read_Index];
								   }
								   Slave_Run_Status[5]=1;
								}

								rx_data_correct =   1;			// Debug Purpose
						}
						else
						{
							crc_calc_fail++;
						}
						//			   else rx_data_correct=0;				// Debug Purpose

						RX_CRC_calc_over = 0;






					}





				}





	}











    	// Step 1: Check for GSM Enable flag for GPRS Data Transfer on RS-485 Bus

		if(GSM_EnableFlag==1)
        {
            GpioDataRegs.GPACLEAR.bit.GPIO8 = 	1;
            GpioDataRegs.GPASET.bit.GPIO17  = 	1;
            Tx_timeout 						= 	0;
            Rx_timeout   					=	0;
        }

        //Step 2: Enable UART in WAKE Mode for RS-485 Data Transfer

		if(SciaRegs.SCIRXST.bit.RXWAKE==1)
		{
			 AddBuf= SciaRegs.SCIRXBUF.all & 0x00FF;
			 SciaRegs.SCIFFRX.bit.RXFIFORESET   =   0;
			 SciaRegs.SCIFFRX.bit.RXFIFORESET   =   1;
			 SciaRegs.SCIFFTX.bit.TXFFINTCLR    =   1;
			 RxSlaveAddress                     =   AddBuf;

			 Read_count=0;                          // Initialized for Buffer Index

			 SciaRegs.SCICTL1.bit.SLEEP         =   0;
			 SciaRegs.SCIFFRX.bit.RXFFIENA      =   1;
			 SciaRegs.SCICTL2.bit.TXINTENA      =   0;
			 SciaRegs.SCIFFTX.bit.SCIFFENA      =   1;
		}

		 //Step 3: Reset Tx-timeout & Rx_timeout to 0 w.r.t GSM_Delay Count
		if(GSM_DelayCount < 200)				//
		{
			Tx_timeout   = 0;
			Rx_timeout   = 0;
		}

		//Step 4: Conditional check for RS_485 Data transfer for Intermediate Buffer Break
		//   for sending Intermediate GPRS messages
		// Delay meant for Sending last byte on the GSM message frame
		if(GSM_DelayCount > 50 && GSM_DelayCount < 150 && GSM_BreakFlag != 0)
		{
			GSM_EnableFlag=0;
			GSM_DelayCount=155;

			 RS485_Start 						  =	  1;
//			 RS485_TX_Enable                      =   1;

			 RS485_TXEnable_Count                 =   0;
			 RX_DataRcvd_Flag                     =   0;

			 SciaRegs.SCICCR.bit.ADDRIDLE_MODE    =   1;         // Enable the Address mode for Multi Processor Communication
			 SciaRegs.SCICTL1.bit.SLEEP           =   1;
			 SciaRegs.SCIFFRX.bit.RXFFIENA        =   0;
			 SciaRegs.SCIFFTX.bit.TXFFINT         =   0;
			 SciaRegs.SCIFFTX.bit.SCIFFENA        =   1;
			 SciaRegs.SCIFFTX.bit.TXFFIENA        =   0;

			 GpioDataRegs.GPASET.bit.GPIO8           =   1;
	//		 GpioDataRegs.GPACLEAR.bit.GPIO29           =   1;
			 GpioDataRegs.GPACLEAR.bit.GPIO17            =   1;
        }

		//Step 5: Conditional check for RS_485 Data transfer for Complete GPRS Data Update
		//   Disabling GPRS Data transfer for one sequential upadte of Complete GPRS messages
		// Delay meant for Sending last byte on the GSM Last message frame
        if(GSM_DelayCount > 50 && GSM_DelayCount < 150 && GSM_CompleteFlag==1)
        {
                GSM_EnableFlag=0;
                GSM_CompleteFlag=0;
                GSM_DelayCount=155;

                 RS485_Start =1;

				 RS485_TXEnable_Count                 =   0;
				 RX_DataRcvd_Flag                     =   0;

				 SciaRegs.SCICCR.bit.ADDRIDLE_MODE    =   1;         // Enable the Address mode for Multi Processor Communication
				 SciaRegs.SCICTL1.bit.SLEEP           =   1;
				 SciaRegs.SCIFFRX.bit.RXFFIENA        =   0;
				 SciaRegs.SCIFFTX.bit.TXFFINT         =   0;
				 SciaRegs.SCIFFTX.bit.SCIFFENA        =   1;
				 SciaRegs.SCIFFTX.bit.TXFFIENA        =   0;

				 GpioDataRegs.GPASET.bit.GPIO8          	 =   1;
				 GpioDataRegs.GPACLEAR.bit.GPIO17            =   1;     //rs485 RXTX
        }

    // Step 6: Conditional check for RS_485 Data transfer
    // Trigger to re-enable GPRS Data Transfer
    if((GSM_Restart_Count > GSM_Restart_Count_Max )&& GSM_BreakFlag != 0 )
    {
      GSM_BreakFlag       =   0;
      GSM_StartFlag       =   1;
    }

    // Saturation values for all counters
    if(GSM_Restart_Count>50000)     GSM_Restart_Count   =   50000;

    if(DataDelayCount>50000)        DataDelayCount      =   50000;

    if(RS485_TXEnable_Count>500)    RS485_TXEnable_Count=   500;







// Timeout for tx, enabling transfer of RS_485 Next frame
//	   GpioDataRegs.GPASET.bit.GPIO11      =   1;
	  if(Tx_timeout >= 800)
	   {
		   Tx_timeout = 0;
		   Rx_timeout   =0;
		   Tx_sent_Success  =   0;
		   TX_SendFlag      =   1;
		   Tx_Sent          =   1;

		   TxSlaveAddress	=	TxSlaveAddress+0x1;			// Incrementing slave Address

		   if(TxSlaveAddress>0x76)TxSlaveAddress=0x71;      // TX_Repeat Process , ending slave address;

		   if(TxSlaveAddress==0x76)							// GPRS Data transfer sync on RS_485 Bus
		   {
			   if(GSM_StartFlag==1)							// Condition check for GPRS Data update
			   {
				   SciaRegs.SCICCR.bit.ADDRIDLE_MODE =0;
				   GSM_SyncFlag     =   1;					//Intermediate flag between RS_485 & GPRS BUFFER Update
				   TX_SendFlag      =   0;                  //Inter Process Communication Disable
				   GSM_StartFlag	= 	0;					//Flag for not re-entering this loop

				   GSM_TX_SendFlag	=	1;					//Trigger flag for initiating GPRS Data Transmission

				   // Registers Enabled for proper GPRS Data Transfer
				   SciaRegs.SCIFFTX.bit.TXFIFOXRESET =   1;
				   SciaRegs.SCIFFTX.bit.TXFFIENA     =   1;
				   SciaRegs.SCICTL1.bit.TXENA        =   1;
				   SciaRegs.SCIFFTX.bit.SCIFFENA     =   1;
			   }

			   TxSlaveAddress=0x71;							// Resetting to first Slave Address
			   GSM_StartFlag = 0;							// Flag for resetting GPRS Start flag
		   }
	   }










       if (Rx_time_count > 50000) Rx_time_count = 50000;
}

// Function for GPRS Frame update
void tx_gsm()
{

	p1=0,i2=0;


	//Individual GPRS Messages
    msg = "AT\r\n\0";

    msg1 = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n\0";
    msg2 = "AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n\0";
    msg3 = "AT+SAPBR=1,1\r\n\0";
    msg4 = "AT+SAPBR=2,1\r\n\0";

    msg5 = "AT+HTTPINIT\r\n\0";
    msg6 = "AT+HTTPPARA=\"CID\",1\r\n\0";

 //   msg6_1 = "AT+HTTPPARA=\"URL\",\"bala97.com/post_test2.php?\0";

    msg6_1 = "AT+HTTPPARA=\"URL\",\"\0";
    msg6_2 = "prod.kripyaems.com:5000/energystatsGet?\0";


//    msg6_2 = "203.129.255.166:5000/energystatsGet?\0";

//  msg7 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SD19,KRIPYASYS3,K13,\0";
//  msg7 = "AT+HTTPPARA=\"URL\",\"bala97.com/post_test2.php?100,SC27,KRIPYASYS3,K13,\0";

    /*

    msg7_0 = "100,SC1,0x0000,KT11,\0";  //D = MASTER = 0x0000
     msg7_1 = "100,SC2,0x0001,KT12,\0"; //D = SLAVE1 = 0x0001
     msg7_2 = "100,SC3,0x0002,KT13,\0"; //D = SLAVE2 = 0x0002
     msg7_3 = "100,SC4,0x0003,KT14,\0"; //D = SLAVE3 = 0x0003
     msg7_4 = "100,SC5,0x0004,KT15,\0"; //D = SLAVE4 = 0x0004
     msg7_5 = "100,SC6,XXX,KT16,\0";
     msg7_6 = "100,SC7,1Complete,KT17,\0";

*/




    msg7_0 = "100,SC1,1719132,\0";  //D20 = MASTER
     msg7_1 = "100,SC2,1719133,\0"; //D21 = SLAVE1
     msg7_2 = "100,SC3,12347,\0"; //D9 = SLAVE2
     msg7_3 = "100,SC4,12348,\0"; //D5 = SLAVE3
     msg7_4 = "100,SC5,12349,\0"; //D10 = = SLAVE4
     msg7_5 = "100,SC6,12350,\0";
     msg7_6 = "100,SC7,TestComplete,KT17,\0";


//    msg7_0 = "100,SC1,MMaster,K11,\0";
//    msg7_1 = "100,SC2,MSlave1,K12,\0";
//    msg7_2 = "100,SC3,MSlave2,K13,\0";
//    msg7_3 = "100,SC4,MSlave3,K14,\0";
//    msg7_4 = "100,SC5,MSlave4,K15,\0";
//    msg7_5 = "100,SC6,MSlave5,K16,\0";
//    msg7_6 = "100,SC7,MComplete,K17,\0";


//       msg7_0 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M10,M1,\0";	// Master
//
//       msg7_1 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M11,S1,\0";	// Slaves 1 to 5
//       msg7_2 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M12,S2,\0";
//       msg7_3 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M13,S3,\0";
//       msg7_4 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M14,S4,\0";
//       msg7_5 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,KS1706M14,S5,\0";
//
//       msg7_6 = "AT+HTTPPARA=\"URL\",\"203.129.255.166:5000/energystatsGet?100,SID1,PMODESYS,MS,\0";	// Complete System

    msg8 = "AT+HTTPDATA=192,2000\r\n\0";
    msg9 = "AT+HTTPACTION=1\r\n\0";
    msg10 = "AT+HTTPREAD\r\n\0";
    msg11 = "AT+HTTPTERM\r\n\0";
    msg12 = "\r\n\0";
    msg_reset = "AT+CFUN=1,1\r\n\0";

    // Repeared calling function for GPRS Message update to server
       if(GSM_SendUpdateFlag==0)
       {
           Tx_buffer_index=0;

           if (gsm_soft_reset_count > 3)						// Initializing function for Resetting GPRS MODEM
           {
               gsm_soft_reset_count = 0;
               msgcount = 16;									// GPRS Message update for msg_reset
           }

           if(msgcount==0)										// GPRS Message update for msg
           {
               while(msg[p1] != '\0')							// Data Update
               {
               Tx_frame[Tx_buffer_index]= msg[p1] ;
               Tx_buffer_index++;
               p1++;
               }
               msgcount=101;									// switching to next message, 101-100=1;
               Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }

           if(msgcount==1)										// GPRS Message update for msg 1
          {
              while(msg1[p1] != '\0')							// Data Update
              {
              Tx_frame[Tx_buffer_index]=  msg1[p1];
              Tx_buffer_index++;
              p1++;
              }
              msgcount=102;										// switching to next message
              Tx_FRUP = 1;										// Trigger for overall GPRS frame update
          }
           if(msgcount==2)										// GPRS Message update for msg 2
           {
               while(msg2[p1] != '\0')							// Data Update
               {
               Tx_frame[Tx_buffer_index]=  msg2[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=103;									// switching to next message
               Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
           if(msgcount==3)									 	// GPRS Message update for msg 3
           {
               while(msg3[p1] != '\0')							// Data Update
               {
               Tx_frame[Tx_buffer_index]=  msg3[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=104;									// switching to next message
               Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
           if(msgcount==4)										// GPRS Message update for msg 4
           {
               while(msg4[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]=  msg4[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=105;									// switching to next message
			  Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
           if(msgcount==5)										// GPRS Message update for msg 5
           {
               while(msg5[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]= msg5[p1];
               Tx_buffer_index++;
               p1++;
               }

               msgcount=106;									// switching to next message
			  Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
           if(msgcount==6)										// GPRS Message update for msg 6
           {
               while(msg6[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]=  msg6[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=107;									// switching to next message
			Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
           if(msgcount==7)										// GPRS Message update for msg 6_1
		 {
        	while(msg6_1[p1] != '\0')
			 {
			 Tx_frame[Tx_buffer_index]=  msg6_1[p1];
			 Tx_buffer_index++;
			 p1++;
			 }
			 msgcount=117;									// switching to next message
		  Tx_FRUP = 1;										// Trigger for overall GPRS frame update
		 }
           if(msgcount==17)										// GPRS Message update for msg 6_1
		 {
			while(msg6_2[p1] != '\0')
			 {
			 Tx_frame[Tx_buffer_index]=  msg6_2[p1];
			 Tx_buffer_index++;
			 p1++;
			 }
			 msgcount=108;									// switching to next message
		  Tx_FRUP = 1;										// Trigger for overall GPRS frame update
		 }
           if(msgcount==8)										// GPRS Message update for msg7_0 to msg7_6
           {
        	   GSM_LOOP_COUNT=18000;							// // Time interval between Upcoming Messages , 15000*0.1msec=1.5 Sec

               	   	   	   // Slave & Master ID Update Starts
                              if( GSM_EventCount==0)
                             {
                                 while(msg7_0[p1] != '\0')
                                 {
                                 Tx_frame[Tx_buffer_index]=  msg7_0[p1];
                                 Tx_buffer_index++;
                                 p1++;
                                 }
                             }
                              if( GSM_EventCount==1)
                              {
                                  while(msg7_1[p1] != '\0')
                                  {
                                  Tx_frame[Tx_buffer_index]=  msg7_1[p1];
                                  Tx_buffer_index++;
                                  p1++;
                                  }
                              }
                              if( GSM_EventCount==2)
                             {
                                 while(msg7_2[p1] != '\0')
                                 {
                                 Tx_frame[Tx_buffer_index]=  msg7_2[p1];
                                 Tx_buffer_index++;
                                 p1++;
                                 }
                             }
                              if( GSM_EventCount==3)
                             {
                                 while(msg7_3[p1] != '\0')
                                 {
                                 Tx_frame[Tx_buffer_index]=  msg7_3[p1];
                                 Tx_buffer_index++;
                                 p1++;
                                 }
                             }
                              if( GSM_EventCount==4)
                             {
                                 while(msg7_4[p1] != '\0')
                                 {
                                 Tx_frame[Tx_buffer_index]=  msg7_4[p1];
                                 Tx_buffer_index++;
                                 p1++;
                                 }
                             }
                              if( GSM_EventCount==5)
                            {
                                while(msg7_5[p1] != '\0')
                                {
                                Tx_frame[Tx_buffer_index]=  msg7_5[p1];
                                Tx_buffer_index++;
                                p1++;
                                }
                            }
                              if( GSM_EventCount==6)
						   {
							  while(msg7_6[p1] != '\0')
							  {
							  Tx_frame[Tx_buffer_index]=  msg7_6[p1];
							  Tx_buffer_index++;
							  p1++;
							  }
						   }
					   // Slave & Master ID Update Ends

		   msgcount=109;									// switching to next message
		   Tx_FRUP = 1;										// Trigger for overall GPRS frame update
           }
               if(msgcount==9)											// Real Time Data update  from Individual Slaves
			 {
		       GSM_LOOP_COUNT=20000;	//18000										// Time interval between Upcoming Messages , 18000*0.1msec=1.8 Sec

               i2=0;
               while(Update_Buffer[i2] != '\0')
               {
                 Tx_frame[Tx_buffer_index]= Update_Buffer[i2];
                 Tx_buffer_index++;
                 i2++;

               }
               p1=0;
               while(msg12[p1] != '\0')
              {
                 Tx_frame[Tx_buffer_index]=  msg12[p1];
                 Tx_buffer_index++;
                 p1++;
              }

               msgcount=110;									// switching to next message
				Tx_FRUP = 1;										// Trigger for overall GPRS frame update
          }

           if(msgcount==10)											// GPRS Message update for msg 8
           {
               GSM_LOOP_COUNT=30000;							// Time interval between Upcoming Messages , 30000*0.1msec=3 Sec
               while(msg8[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]=  msg8[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=111;										// switching to next message
				Tx_FRUP = 1;										// Trigger for overall GPRS frame update

           }

           if(msgcount==11)											// GPRS Message update for msg 9
           {
               while(msg9[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]=  msg9[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=112;										// switching to next message
				Tx_FRUP = 1;										// Trigger for overall GPRS frame update
               rx_pos = 0;
            }

           if(msgcount==12)											// GPRS Message update for msg 10
           {
               while(msg10[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]= msg10[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=113;									// switching to next message
			   Tx_FRUP = 1;										// Trigger for overall GPRS frame update
               GSM_LOOP_COUNT=5000;

           }

           if(msgcount==13)											// GPRS Message update for msg 11
           {
               while(msg11[p1] != '\0')
               {
               Tx_frame[Tx_buffer_index]=  msg11[p1];
               Tx_buffer_index++;
               p1++;
               }
               msgcount=14;										// switching to next message
			   Tx_FRUP = 1;										// Trigger for overall GPRS frame update

               Rx_FRUP =1;										// Triggering for overall GPRS frame update in RX Mode
               Rx_SyncStart = 1;								// Flag for Triggering RX Mode, not used
           }


           // GPRS Message update for msg Reset
           if(msgcount==16)
          {
              while(msg_reset[p1] != '\0')							// dATA Update
              {
              Tx_frame[Tx_buffer_index]=  msg_reset[p1];
              Tx_buffer_index++;
              p1++;
              }
              msgcount              =   14;						// GPRS Message frame update process ending
              gsm_count             =   0;						// Resetting gsm_count
              Tx_FRUP               =   1;						// Trigger for overall GPRS frame update
              GSM_Repeat_Count      =   0;						// Resetting Counter
              GSM_Repeat_Count_max  =   600000;					// Making Delay of 60 Sec during Initializing GPRS MODEM
              Rx_ErrorCount         =   0;						// Reset Rx_ErrorCount to 0, for not entering this loop
          }

// Overall GPRS Data Buffer Update for one complete GPRS Data Transfer for every 20 Seconds
           if( Tx_FRUP == 1)
           {
               GSM_Tx_pos			=	0;								//	Initialize TX Buffer Pos to 0
               Tx_buffer_index1 	= 	Tx_buffer_index-1;
               for (p1=0;p1<=Tx_buffer_index1;p1++)						// Overall TX GPRS Buffer Update

                {
                      Tx_buffer[p1]=Tx_frame[p1];
                }
                  Tx_FRUP = 0;
                  gsm_count=0;
                  GSM_Restart_Count =0;
                  GSM_BreakFlag = 2;
           }

           // Indication for GPRS Data Transfer to Server Completed
           GSM_SendUpdateFlag=1;
        }

       // Timing Interval check for individual GPRS Messages
       if(gsm_count>GSM_LOOP_COUNT)
       {
			   if(msgcount>100)
			   {
				   msgcount-=100;
				   GSM_SendUpdateFlag=0;
				   tx_pack_flag=0;
				   gsm_count=0;
			   }
       }

       // Timing Interval check for Complete GPRS Packet
      if(GSM_Repeat_Count>(GSM_Repeat_Count_max))
      {
          GSM_StartFlag =1;
      }

      //  Synchronizing flag for switching between RS_485 & GPRS Data Transfer
      if(GSM_SyncFlag==1)
       {
          GSM_SyncFlag 		= 	0;					// Flag reset for not re-entry this loop
          GSM_EnableFlag   	=   1;					// Triggering for GSM TX
       }


      // GPRS RX Data processing for Error Recovery from Response from GPRS MODEM: Not used
//      if(Rx_SyncStart==1)
//       {
//       if ( msgcount == 12)
//       {
//           if (Rx_FRUP ==1)
//           {
//               for (i=0;i<200;i++)
//               {
//                   Rx_Frame[i]= Rx_Buffer[i];
//               }
//               for (i=0;i<200;i++)
//               {
//                   Rx_Buffer[i]= 0;
//               }
//                   Rx_FRUP = 0;
//                   Rx_Ready=1;
//                   Rx_Index=0;
//           }
//               if(Rx_Ready==1)
//               {
//                   while((Rx_Frame[Rx_Index] != ':')&&(Rx_Index < 195))
//                   {
//                       Rx_Index++;
//                   }
//                   Rx_Index++;
//                   if (Rx_Frame[Rx_Index] == '1')
//                   {
//                    Rx_Index++;
//                    if (Rx_Frame[Rx_Index] == ',')
//                    {
//                        Rx_Index++;
//
//                        if (Rx_Frame[Rx_Index] == '2')
//                         {
//                             Rx_flag = 1;
//                             Rx_Ready = 0;
//                             Rx_SyncStart=0;
//                             Rx_ErrorCount=0;
//                         }
//                        else
//                         {
//                            Rx_flag = 2;
//                            Rx_ErrorCount++;
//                         }
//                        Rx_SyncStart=0;
//                    }
//
//                   Rx_Index++;
//               }
//               }
//               if(Rx_Index>195)
//               {
//                   Rx_Index=0;
//                   Rx_Ready=0;
//                   Rx_SyncStart=0;
//               }
//           }

//          // Soft Reset Trigger for GPRS Software Function Restart
         if(Rx_ErrorCount>2)
         {
             gsm_soft_reset_count=50;
         }
//         }


    //        Repeatedly Calling Function
                  if(GSM_Repeat_Count>GSM_Repeat_Count_max)
                  {
                      GSM_Repeat_Count=0;
             //       msgcount=5;									// for avoiding msg 0 to msg 3 transfer every time

                      msgcount=0;									// Trigger for updating message frame

                      GSM_EventCount++;								// Slave Update

                      if(GSM_EventCount ==7) GSM_EventCount = 0;	// Slave update Saturation

                      GSM_DataUpdate();      						// Function Call for Master-Slave Configuration

                      GSM_StartFlag =1;

                    if (GSM_Repeat_Count_max >300000) msgcount=0;	// Initial configuration settings for GPRS MODEM
					GSM_SendUpdateFlag=0;							// Clearing flag

					GSM_Repeat_Count_max=200000;					// Making Delay of 20 Sec for Next GPRS Message update on Server
		           }
     //       Repeatedly Calling Function

          if(GSM_TX_SendFlag==1)									// Register setting for Proper TX Data send
          {
              SciaRegs.SCIFFTX.bit.TXFIFOXRESET =   1;
              SciaRegs.SCIFFTX.bit.TXFFIENA     =   1;
              SciaRegs.SCICTL1.bit.TXENA        =   1;
              SciaRegs.SCIFFTX.bit.SCIFFENA     =   1;
          }

}

void GSM_DataUpdate( )
{
	int  i1=0, i3=0;
    Slaveid_Calc();


	 	 	 	 	 // Clearing the Slave Buffers after valid data Received
	 	 	 	 	 Slave_Run_Status[0]++;
	 	 	 	 	 if(Slave_Run_Status[0]>5)
	 	 	 	 	 {

	 	 	 	 		 for(i3=0; i3<=5; i3++)
	 	 	 	 			 {
	 	 	 	 			 Slave_Run_Status[i3]=0;
	 	 	 	 			 }
	 	 	 	 	 }


	 	 	 		 // Master Data update for GPRS Posting
	                 if(GSM_EventCount==0)
	                  {
	                       Buffer0=pvPanelVoltage;                 // Master PV Voltage
	                       Buffer1=flybackCurrent1;     			// Master PV Current


	                       if (systemState == 0 || systemState == 2)
	                       {
	                          Buffer2 = 0;
	                          Buffer3 = 0;
	                       }

	                       if (systemState == 1)   	   	   	   	   	   	   	    // Grid-tie  Update
	                       {
	                        Buffer2= peakGridVoltage;
	                        Buffer3= peakInverterOutputCurrent;
	                       }

	                       if(systemState == 3)								    // OFF-Gird  Update
	                       {
	                    	   Buffer2= peakInverterOutputVoltage;
	                    	   Buffer3= my_Irms_q15_result;
	                       }


	                        Buffer4=measuredTemperature;                       	// Master Temperature
	                        Buffer5=systemState;
	                        Buffer6=faultState_Rs485;                          		// Fault State
	                        Buffer7=GSM_EventCount;


                            makestring2(MuidByteConv);
                            for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}


	                         makestring(Buffer0);               				//PV Voltage
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer1);               //PV Current in mA
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer2);               //Output Voltage
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer3);               //Output Current in mA
							  for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							  Update_Buffer[i1++] = '0';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
							  Update_Buffer[i1++] = ',';
							  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
							  Update_Buffer[i1++] = '3';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = ',';
	                         makestring( Buffer4);               //Temperature
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring( Buffer5);               //System Status
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring( Buffer6);               //Trip Error
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         Update_Buffer[i1-1] = '\"';
	                         Update_Buffer[i1] = '\0';
	                  }

	                   // Slave 1 Data update for GPRS Posting

	                if(GSM_EventCount==1)          							// Slave 1
	                  {
	                       Buffer0=Rcvd_Rx_Data_1[2]<<8;           			// Slave 1 PV Voltage Higher Byte
	                       Buffer0|=Rcvd_Rx_Data_1[1];           			// Slave 1 PV Voltage Lower Byte

	                       Buffer1=Rcvd_Rx_Data_1[4]<<8;           			// Slave 1 PV Current Higher Byte
	                       Buffer1|=Rcvd_Rx_Data_1[3];           			// Slave 1 PV Current Lower Byte

	                       Buffer3=Rcvd_Rx_Data_1[8]<<8;           		// Slave 1 Grid Current Higher Byte
						   Buffer3|=Rcvd_Rx_Data_1[7];           		// Slave 1 Grid Current Lower Byte



	                       if (Rcvd_Rx_Data_1[9] == 1)   	   	   	   	    // Grid-tie Voltage Update
	                       {
	                    	   Buffer2=Rcvd_Rx_Data_1[6]<<8;           		// Slave 1 Grid Voltage Higher Byte
							   Buffer2|=Rcvd_Rx_Data_1[5];           		// Slave 1 Grid Voltage Lower Byte
	                       }

	                       if (Rcvd_Rx_Data_1[9] == 3)   	   	   	   	   	   	   	    // OFF-Grid Voltage Update
	                       {
	                    	   Buffer2=Rcvd_Rx_Data_1[6]<<8;               // Slave 1 Grid Voltage Higher Byte
							   Buffer2|=Rcvd_Rx_Data_1[5];           		// Slave 1 Grid Voltage Lower Byte
	                       }

						   if (Rcvd_Rx_Data_1[9] == 0 || Rcvd_Rx_Data_1[9] == 2)						// For avoiding Junk updates on Output Voltage & Current
	                       {
	                          Buffer2 = 0;
	                          Buffer3 = 0;
	                       }

	                       Buffer4=Rcvd_Rx_Data_1[12]<<8;           		// Slave 1 Temperature Higher Byte
						   Buffer4|=Rcvd_Rx_Data_1[11];           		    // Slave 1 Temperature Lower Byte

	                       Buffer5=Rcvd_Rx_Data_1[9];               		// Slave 1 System State
	                       Buffer6=Rcvd_Rx_Data_1[10];                      // Slave 1 Fault State

	                       Buffer7=GSM_EventCount;							// Slave Number update for GPRS


	                       makestring2(S1uidByteConv);
	                       for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}

	                         makestring(Buffer0);               //PV Voltage
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer1);               //PV Current in mA
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer2);               //Output Voltage
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         makestring(Buffer3);               //Output Current in mA
	                         for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                      	  Update_Buffer[i1++] = '1';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
							  Update_Buffer[i1++] = ',';
							  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
							  Update_Buffer[i1++] = '3';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = ',';
	                         makestring( Buffer4);               //Temperature
	                    	 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring( Buffer5);               //System Status
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring( Buffer6);               //Trip Error
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
	                         Update_Buffer[i1-1] = '\"';
	                         Update_Buffer[i1] = '\0';
	                  }

	                // Slave 2 Data update for GPRS Posting

					if(GSM_EventCount==2)          							// Slave 2
					  {
						   Buffer0=Rcvd_Rx_Data_2[2]<<8;           			// Slave 2 PV Voltage Higher Byte
						   Buffer0|=Rcvd_Rx_Data_2[1];           			// Slave 2 PV Voltage Lower Byte

						   Buffer1=Rcvd_Rx_Data_2[4]<<8;           			// Slave 2 PV Current Higher Byte
						   Buffer1|=Rcvd_Rx_Data_2[3];           			// Slave 2 PV Current Lower Byte

						   Buffer3=Rcvd_Rx_Data_2[8]<<8;           		// Slave 2 Grid Current Higher Byte
						   Buffer3|=Rcvd_Rx_Data_2[7];           		// Slave 2 Grid Current Lower Byte


						   if (Rcvd_Rx_Data_2[9] == 1)   	   	   	   	   	   	   	    // Grid-tie Voltage Update
						   {
							   Buffer2=Rcvd_Rx_Data_2[6]<<8;           		// Slave 1 Grid Voltage Higher Byte
							   Buffer2|=Rcvd_Rx_Data_2[5];           		// Slave 1 Grid Voltage Lower Byte
						   }

						   if (Rcvd_Rx_Data_2[9] == 3)   	   	   	   	   	   	   	    // OFF-Grid Voltage Update
						   {
							   Buffer2=Rcvd_Rx_Data_2[6]<<8;               // Slave 1 Grid Voltage Higher Byte
							   Buffer2|=Rcvd_Rx_Data_2[5];           		// Slave 1 Grid Voltage Lower Byte
						   }

						   if (Rcvd_Rx_Data_2[9] == 0 || Rcvd_Rx_Data_2[9] == 2)						// For avoiding Junk updates on Output Voltage & Current
						   {
							  Buffer2 = 0;
							  Buffer3 = 0;
						   }


						   Buffer4=Rcvd_Rx_Data_2[12]<<8;           		// Slave 2 Temperature Higher Byte
						   Buffer4|=Rcvd_Rx_Data_2[11];           		    // Slave 2 Temperature Lower Byte

						   Buffer5=Rcvd_Rx_Data_2[9];               		// Slave 2 System State
						   Buffer6=Rcvd_Rx_Data_2[10];                      // Slave 2 Fault State

						   Buffer7=GSM_EventCount;							// Slave Number update for GPRS

                           makestring2(S2uidByteConv);
                           for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}



							 makestring(Buffer0);               //PV Voltage
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring(Buffer1);               //PV Current in mA
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring(Buffer2);               //Output Voltage
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring(Buffer3);               //Output Current in mA
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							  Update_Buffer[i1++] = '2';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
							  Update_Buffer[i1++] = ',';
							  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
							  Update_Buffer[i1++] = '3';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = '0';
							  Update_Buffer[i1++] = ',';
							 makestring( Buffer4);               //Temperature
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring( Buffer5);               //System Status
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 makestring( Buffer6);               //Trip Error
							 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
							 Update_Buffer[i1-1] = '\"';
							 Update_Buffer[i1] = '\0';
					  }

					// Slave 3 Data update for GPRS Posting

				if(GSM_EventCount==3)          							// Slave 3
				  {
					   Buffer0=Rcvd_Rx_Data_3[2]<<8;           			// Slave 3 PV Voltage Higher Byte
					   Buffer0|=Rcvd_Rx_Data_3[1];           			// Slave 3 PV Voltage Lower Byte

					   Buffer1=Rcvd_Rx_Data_3[4]<<8;           			// Slave 3 PV Current Higher Byte
					   Buffer1|=Rcvd_Rx_Data_3[3];           			// Slave 3 PV Current Lower Byte

					   Buffer3=Rcvd_Rx_Data_3[8]<<8;           		// Slave 2 Grid Current Higher Byte
					   Buffer3|=Rcvd_Rx_Data_3[7];           		// Slave 2 Grid Current Lower Byte

					   if (Rcvd_Rx_Data_3[9] == 1)   	   	   	   	    // Grid-tie Voltage Update
					   {
						   Buffer2=Rcvd_Rx_Data_3[6]<<8;           		// Slave 1 Grid Voltage Higher Byte
						   Buffer2|=Rcvd_Rx_Data_3[5];           		// Slave 1 Grid Voltage Lower Byte
					   }

					   if (Rcvd_Rx_Data_3[9] == 3)   	   	   	   	   	   	   	    // OFF-Grid Voltage Update
					   {
						   Buffer2=Rcvd_Rx_Data_3[6]<<8;               // Slave 1 Grid Voltage Higher Byte
						   Buffer2|=Rcvd_Rx_Data_3[5];           		// Slave 1 Grid Voltage Lower Byte
					   }

					   if (Rcvd_Rx_Data_3[9] == 0 || Rcvd_Rx_Data_3[9] == 2)						// For avoiding Junk updates on Output Voltage & Current
					   {
						  Buffer2 = 0;
						  Buffer3 = 0;
					   }

					   Buffer4=Rcvd_Rx_Data_3[12]<<8;           		// Slave 3 Temperature Higher Byte
					   Buffer4|=Rcvd_Rx_Data_3[11];           		    // Slave 3 Temperature Lower Byte

					   Buffer5=Rcvd_Rx_Data_3[9];               		// Slave 3 System State
					   Buffer6=Rcvd_Rx_Data_3[10];                      // Slave 3 Fault State

					   Buffer7=GSM_EventCount;							// Slave Number update for GPRS



                       makestring2(S3uidByteConv);
                       for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}



						 makestring(Buffer0);               //PV Voltage
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 makestring(Buffer1);               //PV Current in mA
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 makestring(Buffer2);               //Output Voltage
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 makestring(Buffer3);               //Output Current in mA
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						  Update_Buffer[i1++] = '3';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
						  Update_Buffer[i1++] = ',';
						  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
						  Update_Buffer[i1++] = '3';
						  Update_Buffer[i1++] = '0';
						  Update_Buffer[i1++] = '0';
						  Update_Buffer[i1++] = ',';
						 makestring( Buffer4);               //Temperature
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 makestring( Buffer5);               //System Status
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 makestring( Buffer6);               //Trip Error
						 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
						 Update_Buffer[i1-1] = '\"';
						 Update_Buffer[i1] = '\0';
				  }

				// Slave 4 Data update for GPRS Posting

			if(GSM_EventCount==4)          							// Slave 4
			  {
				   Buffer0=Rcvd_Rx_Data_4[2]<<8;           			// Slave 4 PV Voltage Higher Byte
				   Buffer0|=Rcvd_Rx_Data_4[1];           			// Slave 4 PV Voltage Lower Byte

				   Buffer1=Rcvd_Rx_Data_4[4]<<8;           			// Slave 4 PV Current Higher Byte
				   Buffer1|=Rcvd_Rx_Data_4[3];           			// Slave 4 PV Current Lower Byte

				   Buffer3=Rcvd_Rx_Data_4[8]<<8;           		// Slave 2 Grid Current Higher Byte
				   Buffer3|=Rcvd_Rx_Data_4[7];           		// Slave 2 Grid Current Lower Byte


				   if (Rcvd_Rx_Data_4[9]== 1)   	   	   	   	   	   	   	    // Grid-tie Voltage Update
				   {
					   Buffer2=Rcvd_Rx_Data_4[6]<<8;           		// Slave 1 Grid Voltage Higher Byte
					   Buffer2|=Rcvd_Rx_Data_4[5];           		// Slave 1 Grid Voltage Lower Byte
				   }

				   if (Rcvd_Rx_Data_4[9] == 3)   	   	   	   	   	   	   	    // OFF-Grid Voltage Update
				   {
					   Buffer2=Rcvd_Rx_Data_4[6]<<8;               // Slave 1 Grid Voltage Higher Byte
					   Buffer2|=Rcvd_Rx_Data_4[5];           		// Slave 1 Grid Voltage Lower Byte
				   }

				   if (Rcvd_Rx_Data_4[9] == 0 || Rcvd_Rx_Data_4[9] == 2)						// For avoiding Junk updates on Output Voltage & Current
				   {
					  Buffer2 = 0;
					  Buffer3 = 0;
				   }

				   Buffer4=Rcvd_Rx_Data_4[12]<<8;           		// Slave 4 Temperature Higher Byte
				   Buffer4|=Rcvd_Rx_Data_4[11];           		    // Slave 4 Temperature Lower Byte

				   Buffer5=Rcvd_Rx_Data_4[9];               		// Slave 4 System State
				   Buffer6=Rcvd_Rx_Data_4[10];                      // Slave 4 Fault State

				   Buffer7=GSM_EventCount;							// Slave Number update for GPRS



                   makestring2(S4uidByteConv);
                   for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}


					 makestring(Buffer0);               //PV Voltage
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer1);               //PV Current in mA
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer2);               //Output Voltage
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer3);               //Output Current in mA
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					  Update_Buffer[i1++] = '4';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
					  Update_Buffer[i1++] = ',';
					  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
					  Update_Buffer[i1++] = '3';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = ',';
					 makestring( Buffer4);               //Temperature
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer5);               //System Status
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer6);               //Trip Error
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 Update_Buffer[i1-1] = '\"';
					 Update_Buffer[i1] = '\0';
			  }

			// Slave 5 Data update for GPRS Posting

			if(GSM_EventCount==5)          							// Slave 5
			  {
				   Buffer0=Rcvd_Rx_Data_5[2]<<8;           			// Slave 5 PV Voltage Higher Byte
				   Buffer0|=Rcvd_Rx_Data_5[1];           			// Slave 5 PV Voltage Lower Byte

				   Buffer1=Rcvd_Rx_Data_5[4]<<8;           			// Slave 5 PV Current Higher Byte
				   Buffer1|=Rcvd_Rx_Data_5[3];           			// Slave 5 PV Current Lower Byte

				   Buffer3=Rcvd_Rx_Data_5[8]<<8;           		// Slave 2 Grid Current Higher Byte
				   Buffer3|=Rcvd_Rx_Data_5[7];           		// Slave 2 Grid Current Lower Byte


				   if (Rcvd_Rx_Data_5[9]== 1)   	   	   	   	   	   	   	    // Grid-tie Voltage Update
				   {
					   Buffer2=Rcvd_Rx_Data_5[6]<<8;           		// Slave 1 Grid Voltage Higher Byte
					   Buffer2|=Rcvd_Rx_Data_5[5];           		// Slave 1 Grid Voltage Lower Byte
				   }

				   if (Rcvd_Rx_Data_5[9] == 3)   	   	   	   	   	   	   	    // OFF-Grid Voltage Update
				   {
					   Buffer2=Rcvd_Rx_Data_5[6]<<8;               // Slave 1 Grid Voltage Higher Byte
					   Buffer2|=Rcvd_Rx_Data_5[5];           		// Slave 1 Grid Voltage Lower Byte
				   }

				   if (Rcvd_Rx_Data_5[9] == 0 || Rcvd_Rx_Data_5[9] == 2)						// For avoiding Junk updates on Output Voltage & Current
				   {
					  Buffer2 = 0;
					  Buffer3 = 0;
				   }


				   Buffer4=Rcvd_Rx_Data_5[12]<<8;           		// Slave 5 Temperature Higher Byte
				   Buffer4|=Rcvd_Rx_Data_5[11];           		    // Slave 5 Temperature Lower Byte

				   Buffer5=Rcvd_Rx_Data_5[9];               		// Slave 5 System State
				   Buffer6=Rcvd_Rx_Data_5[10];                      // Slave 5 Fault State

				   Buffer7=GSM_EventCount;							// Slave Number update for GPRS



                   makestring2(S5uidByteConv);
                   for(i3=0;i3<=9;i3++){ Update_Buffer[i1]=tmp1[i3];i1++;}


					 makestring(Buffer0);               //PV Voltage
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer1);               //PV Current in mA
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer2);               //Output Voltage
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer3);               //Output Current in mA
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					  Update_Buffer[i1++] = '5';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
					  Update_Buffer[i1++] = ',';
					  Update_Buffer[i1++] = '0';         // System Capacity-300 watts
					  Update_Buffer[i1++] = '3';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = ',';
					 makestring( Buffer4);               //Temperature
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer5);               //System Status
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer6);               //Trip Error
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 Update_Buffer[i1-1] = '\"';
					 Update_Buffer[i1] = '\0';
			  }

 	 	 	 // Complete Parallel Mode System Data update for GPRS Posting
 	 	    if(GSM_EventCount==6)
			  {


                if (systemState == 1)   	   	   	   	   	   	   	    // Grid-tie  Update
                {
                 Buffer2= peakGridVoltage;
                 Buffer3= peakInverterOutputCurrent;
                }

                if(systemState == 3)								    // OFF-Gird  Update
                {
             	   Buffer2= peakInverterOutputVoltage;
             	   Buffer3= my_Irms_q15_result;
                }


                Rcvd_Data_Total[0]=Buffer3;
 	 	    	Buffer3=Rcvd_Rx_Data_1[8]<<8;
 	 	    	Buffer3|=Rcvd_Rx_Data_1[7];
 	 	    	Rcvd_Data_Total[0]=Rcvd_Data_Total[0]+ Buffer3;
 	 			Buffer3=Rcvd_Rx_Data_2[8]<<8;
 	 	 	 	Buffer3|=Rcvd_Rx_Data_2[7];
 	 	 	 	Rcvd_Data_Total[0]=Rcvd_Data_Total[0]+ Buffer3;
 	 			Buffer3=Rcvd_Rx_Data_3[8]<<8;
 	 	 	 	Buffer3|=Rcvd_Rx_Data_3[7];
 	 	 	 	Rcvd_Data_Total[0]=Rcvd_Data_Total[0]+ Buffer3;
 	 			Buffer3=Rcvd_Rx_Data_4[8]<<8;
 	 	 	 	Buffer3|=Rcvd_Rx_Data_4[7];
 	 	 	 	Rcvd_Data_Total[0]=Rcvd_Data_Total[0]+ Buffer3;
 	 			Buffer3=Rcvd_Rx_Data_5[8]<<8;
 	 	 	 	Buffer3|=Rcvd_Rx_Data_5[7];
 	 	 	 	Rcvd_Data_Total[0]=Rcvd_Data_Total[0]+ Buffer3;

 	 	 	 	   Buffer0=pvPanelVoltage;                  			// Master PV Voltage
				   Buffer1=flybackCurrent1;     						// Master PV Current
				   Buffer8= Rcvd_Data_Total[0];                         // Complete System grid Output Current

				   if (systemState == 0 || systemState == 2)
				   {
					  Buffer2 = 0;
					  Buffer8 = 0;
				   }

					Buffer4=measuredTemperature;       	// Master Temperature
					Buffer5=systemState;
					Buffer6=faultState_Rs485;               	// Fault State
					Buffer7=GSM_EventCount;

					 makestring(Buffer0);               //PV Voltage
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer1);               //PV Current in mA
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring(Buffer2);               //Output Voltage

					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring1(Buffer8);               //Output Current in mA
					 for(i3=0;i3<=6;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}

					  Update_Buffer[i1++] = '6';         // Dummy for Single Unit, Slave/Master Number for Parallel mode setup
					  Update_Buffer[i1++] = ',';
					  Update_Buffer[i1++] = '1';         // System Capacity-1200 watts
					  Update_Buffer[i1++] = '2';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = '0';
					  Update_Buffer[i1++] = ',';
					 makestring( Buffer4);               //Temperature
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer5);               //System Status
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 makestring( Buffer6);               //Trip Error
					 for(i3=0;i3<=5;i3++){ Update_Buffer[i1]=tmp[i3];i1++;}
					 Update_Buffer[i1-1] = '\"';
					 Update_Buffer[i1] = '\0';
			  }
}


void Slaveid_Calc(void)
{

        if(RxSlaveAddress==0x71)
        {
            for(incIndex =13;incIndex<=16;incIndex++)
            {
                UID_TxGSM[incIndex-13]= Rcvd_Rx_Data_1[incIndex];
            }
            S1uidByte12  =UID_TxGSM[1] <<8 ;
            S1uidByte12  = S1uidByte12| UID_TxGSM[0] ;

            S1uidByte34  =UID_TxGSM[3] <<8 ;
            S1uidByte34  = S1uidByte34| UID_TxGSM[2] ;

            S1uidByteConv =(S1uidByte34 & 0xFFFF);
            S1uidByteConv = (((S1uidByteConv<< 8)<<8));
            S1uidByteConv = (S1uidByteConv|S1uidByte12);




        }
        else if(RxSlaveAddress==0x72)
            {
                for(incIndex =13;incIndex<=16;incIndex++)
                {
                    UID_TxGSM_2[incIndex-13]= Rcvd_Rx_Data_2[incIndex];
                }
                S2uidByte12  =UID_TxGSM_2[1] <<8 ;
                S2uidByte12  = S2uidByte12| UID_TxGSM_2[0] ;

                S2uidByte34  =UID_TxGSM_2[3] <<8 ;
                S2uidByte34  = S2uidByte34| UID_TxGSM_2[2] ;


                S2uidByteConv =(S2uidByte34 & 0xFFFF);
                S2uidByteConv = (((S2uidByteConv<< 8)<<8));
                S2uidByteConv = (S2uidByteConv|S2uidByte12);
            }
        else if(RxSlaveAddress==0x73)
        {
            for(incIndex =13;incIndex<=16;incIndex++)
            {
                UID_TxGSM_3[incIndex-13]= Rcvd_Rx_Data_3[incIndex];
            }
            S3uidByte12  =UID_TxGSM_3[1] <<8 ;
            S3uidByte12  = S3uidByte12| UID_TxGSM_3[0] ;

            S3uidByte34  =UID_TxGSM_3[3] <<8 ;
            S3uidByte34  = S3uidByte34| UID_TxGSM_3[2] ;
           // S3uidByteConv = ((((S3uidByte12 & 0xFFFF)<< 8)<<8)|S3uidByte34);

            S3uidByteConv =(S3uidByte34 & 0xFFFF);
            S3uidByteConv = (((S3uidByteConv<< 8)<<8));
            S3uidByteConv = (S3uidByteConv|S3uidByte12);
        }

        else if(RxSlaveAddress==0x74)
        {
            for(incIndex =13;incIndex<=16;incIndex++)
            {
                UID_TxGSM_4[incIndex-13]= Rcvd_Rx_Data_4[incIndex];
            }
            S4uidByte12  =UID_TxGSM_4[1] <<8 ;
            S4uidByte12  = S4uidByte12| UID_TxGSM_4[0] ;

            S4uidByte34  =UID_TxGSM_4[3] <<8 ;
            S4uidByte34  = S4uidByte34| UID_TxGSM_4[2] ;
           // S4uidByteConv = ((((S4uidByte12 & 0xFFFF)<< 8)<<8)|S4uidByte34);
            S4uidByteConv =(S4uidByte34 & 0xFFFF);
           S4uidByteConv = (((S4uidByteConv<< 8)<<8));
           S4uidByteConv = (S4uidByteConv|S4uidByte12);


        }
        else if(RxSlaveAddress==0x75)
        {
            for(incIndex =13;incIndex<=16;incIndex++)
            {
                UID_TxGSM_5[incIndex-13]= Rcvd_Rx_Data_5[incIndex];
            }
            S5uidByte12  =UID_TxGSM_5[1] <<8 ;
            S5uidByte12  = S5uidByte12| UID_TxGSM_5[0] ;

            S5uidByte34  =UID_TxGSM_5[3] <<8 ;
            S5uidByte34  = S5uidByte34| UID_TxGSM_5[2] ;
           // S5uidByteConv = ((((S5uidByte12 & 0xFFFF)<< 8)<<8)|S5uidByte34);

            S5uidByteConv =(S5uidByte34 & 0xFFFF);
           S5uidByteConv = (((S5uidByteConv<< 8)<<8));
           S5uidByteConv = (S5uidByteConv|S5uidByte12);

        }

}


