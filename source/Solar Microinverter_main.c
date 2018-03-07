
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>
#include "Solar Microinverter_defines.h"
#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "uart.h"
#include "gsm.h"

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);
void Adc_Config(void);
void InitComparator(void);
void statemachine(void);

interrupt void adc_isr(void);
interrupt void epwm4_isr(void);
void LED_Indication(void);

void ModeChangeover(void);
#pragma CODE_SECTION(ModeChangeover, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamconstsLoadStart;
extern Uint16 RamconstsLoadEnd;
extern Uint16 RamconstsRunStart;
extern unsigned char switchState, systemState,faultState;
extern volatile unsigned char gridFlag;


volatile unsigned int counter_100us = 0, flag_10ms = 0;


unsigned char statemachineflag = 0;
int count = 0, max_count = 560, i = 0;
//int array1[560];		// , array2[560];


extern int start_current_fault_check;
volatile int current_counter = 0;
extern char acCurrentOffsetFlag;
volatile extern unsigned int acoffset_cal2;
void InitECapture(void);
interrupt void ecap1_isr(void);
void InitEPwmSyncGpio(void);

Uint32 ECAP1 = 0;
Uint32 ECAP2 = 0;
Uint32 ECAP3 = 0;
Uint32 ECAP4 = 0;
Uint32 ECAP_AVG = 0;
Uint32 frequancy_mains = 0;
Uint32 Zc1=0;

extern volatile unsigned int slaveRunStatus, slaveRunStatus1 ,slaveRunStatus2 ,slaveRunStatus3 ,TotalPLC , TotalPLC_Slaves ,numberOfErrorSlaves , numberOfSlaves ,slavestartcounter, SlaveStartDelay , SlaveStartFlag ;
extern volatile unsigned int  Tx_timeout ,Rx_timeout ,Tx_timeout_enable ,Rx_timeout_enable,Rx_time_count ;
extern unsigned int gsm_count;
extern long int GSM_Repeat_Count;
extern unsigned int GSM_DelayCount ,GSM_Restart_Count;
extern unsigned int DataDelayCount, RS485_TXEnable_Count; // added for RS_485 & GPRS DATA Communication
void variablePI (void);
extern unsigned int VariablePIFlag;



void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
   InitSysCtrl();
// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2803x_EPwm.c fileq
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
   InitTzGpio();
   InitECap1Gpio();
   InitEPwmSyncGpio();


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
   InitPieCtrl();
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
   MemCopy(&RamconstsLoadStart, &RamconstsLoadEnd, &RamconstsRunStart);
   InitFlash();
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
   InitPieVectTable();

   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT1 = &adc_isr;

   PieVectTable.EPWM4_INT = &epwm4_isr;

   PieVectTable.ECAP1_INT = &ecap1_isr;

   PieVectTable.SCIRXINTA       = &sciRxFifoIsr;            // For Basic RS_485
   PieVectTable.SCITXINTA       = &sciTxFifoIsr;
   EDIS;    // This is needed to disable write to EALLOW protected registers


//GPIO OUTPUT PINS INITIOLIZATION
   EALLOW;  // This is needed to write to EALLOW protected register

   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;     // Opto Drive signal 1 selected as Output pin
   GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     // Opto Drive signal 1 selected as Output pin

   GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;     // Opto Drive Signal 2 Selected as Output pin
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;     // Opto Drive Signal 2 Selected as Output pin


   GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;     // Led Drive Signal 2 selected as Output pin
   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;     // Led Drive Signal 2 selected as Output pin

   GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // Led Drive Signal 1 selected as Output pin
   GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;     // Led Drive Signal 1 selected as Output pin

   GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;      // Relay Driver selected as Output pin
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;     // Relay Driver selected as Output pin
   GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;



   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 1;          // PWM SHUTDOWN pin
   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0x0;
   GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
   GpioDataRegs.GPASET.bit.GPIO28 = 1;

   GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;     // Relay Driver2 selected as Output pin
   GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;     // Relay Driver2 selected as Output pin
   GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

   //GPIO INPUT PINS INITIOLIZATION

   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;      // ZCD mains selected as a input pin
   GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;      // ZCD mains selected as a input pin

   GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;     // On- Off switch selected as input pin
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;     // On- Off switch selected as input pin


   // Added for RS_485
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // ?? Debug Purpose as Output pin
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;  // GPIO18 = XCLKOUT
    // Added for RS_485


   GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;     // ?? Debug Purpose as Output pin
 //  GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // ?? Debug Purpose as Output pin
   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;     // ?? Debug Purpose as Output pin

   //4 wire system grid detection gpio configuration
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;      //  Grid Detect GPIO INPUT pin using OptoCoupler
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;      //  Grid Detect GPIO INPUT pin using OptoCoupler
    GpioDataRegs.GPASET.bit.GPIO10 = 0;      //Grid Detect GPIO INPUT pin using OptoCoupler



   // PWMSYNC  Master GPIO Configuration

   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;    // off Grid Startup Sync line
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;     // off Grid Startup Sync line

   GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // ZC Sync
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // ZC Sync

   GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
   GpioDataRegs.GPASET.bit.GPIO16 = 1;  //set Driver ic enablae pin for off grid


	 GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;     // BU test purpose
	 GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;

	 GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // RX/TX test purpose
	 GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;



   EDIS;    // This is needed to disable write to EALLOW protected registers

   InitAdc();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

  InitEPwm1Example();
  InitEPwm2Example();

/*   InitEPwm1Example_Grid();
   InitEPwm2Example_Grid();*/
   InitEPwm3Example();
   InitEPwm4Example();
   Adc_Config();
//   #ifdef HPROTECTION
   InitComparator();
//   #endif

   InitSciaGpio();
   scia_fifo_init();                    // UART Included for RS-485 Communication

    InitECapture();




   // Connect the watchdog to the WAKEINT interrupt of the PIE
   // Write to the whole SCSR register to avoid clearing WDOVERRIDE bit
	  EALLOW;
	  SysCtrlRegs.SCSR = 0x0004;//BIT1;
	  EDIS;




   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

   // zc period computation gpio 22 used to trigger XINT1
   EALLOW;


   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable INT 1.1 in the PIE
   PieCtrlRegs.PIEIER3.bit.INTx4 = 1;	// EPWM4

     // Enable eCAP INTn in the PIE: Group 3 interrupt 1-6
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;  // ECAP


   PieCtrlRegs.PIEIER9.bit.INTx1 = 1;          // Enable PIE Gropu SCI RX
   PieCtrlRegs.PIEIER9.bit.INTx2 = 1;          // Enable PIE Gropu SCI TX

   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // WATCHDOG Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx8 = 1;   // WATCHDOG Enable PIE Group 1 INT8



   EDIS;


   IER |= M_INT1;						// Enable CPU Interrupt 1
   IER |= M_INT3;
   IER |= M_INT4;
   IER |= M_INT9;

   asm(" NOP"); //Wait for PIEACK to exit the pipeline

   EINT;          						// Enable Global interrupt INTM
   ERTM;          						// Enable Global realtime interrupt DBGM

	ServiceDog();
	// Enable the watchdog
	EALLOW;
	SysCtrlRegs.WDCR = 0x002D;
	EDIS;


   for(;;)
   {

	   LED_Indication();  //need to modify based on the event change
       oi_uart();           // For RS_485 Data Transfer
       tx_gsm();                      // For GPRS Data Transfer



        if(acoffset_cal2 ==1)
        {
            ModeChangeover();
        }

        if(VariablePIFlag == 1)
        {
            variablePI ();
        }




        if(statemachineflag == 1)
        {
            //		GpioDataRegs.GPASET.bit.GPIO19 = 1;
            //		 GpioDataRegs.GPASET.bit.GPIO11      =   1;

            statemachineflag = 0;


        }

   }

}



void InitEPwm1Example()
{
	EALLOW;
   EPwm1Regs.TBPRD = FLYBACKPERIOD >> 1; //535;//;                        // Set timer period
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;//TB_COUNT_UP; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   EPwm1Regs.CMPA.half.CMPA = 0;


   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on Zero
   //EPwm1Regs.AQCTLA.bit.CAD=AQ_NO_ACTION;
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
   //   EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;





   // Active Low PWMs - Setup Deadband
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
   EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;
   EPwm1Regs.DBRED = RE_DELAY;
   EPwm1Regs.DBFED = FE_DELAY;
 //  EPwm1_DB_Direction = DB_UP;
   EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL	= 1;		// Select SOC from from CPMA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event

//   #ifdef HPROTECTION
   EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP2OUT;
   EPwm1Regs.DCTRIPSEL.bit.DCBHCOMPSEL = DC_COMP2OUT;
   EPwm1Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;
   EPwm1Regs.TZDCSEL.bit.DCBEVT1 = TZ_DCAH_HI;
   EPwm1Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT1;
   EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
   EPwm1Regs.TZSEL.bit.DCAEVT1 = 1;
   EPwm1Regs.TZSEL.bit.DCBEVT1 = 1;
//   #endif

  // EPwm1Regs.TZSEL.bit.OSHT1 = 1;
   EPwm1Regs.TZSEL.bit.OSHT2 = 1;
  // EPwm1Regs.TZSEL.bit.OSHT3 = 1;

  // What do we want the TZ1 and TZ2 to do?
  EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
  EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_HI;
  EPwm1Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
  EPwm1Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_HI;
  // Enable TZ interrupt
  EPwm1Regs.TZEINT.bit.OST = 1;
  EPwm1Regs.TZEINT.bit.DCAEVT1 = 1;
  EPwm1Regs.TZEINT.bit.DCBEVT1 = 1;
  EDIS;
}

void InitEPwm2Example()
{
   EALLOW;
   EPwm2Regs.TBPRD = FLYBACKPERIOD >> 1; //535;                        // Set timer period
   EPwm2Regs.TBPHS.half.TBPHS = FLYBACKINTERLEAVEDPHASE;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
   EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm2Regs.TBCTL.bit.PHSEN=TB_ENABLE;
   EPwm2Regs.TBCTL.bit.PHSDIR=TB_UP;

   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;//TB_COUNT_UP; // Count up
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow just to observe on the scope

   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Setup compare
   EPwm2Regs.CMPA.half.CMPA = 0;

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM2A on Zero
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
   //EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

   // Active Low complementary PWMs - setup the deadband
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
   EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm2Regs.DBCTL.bit.HALFCYCLE = 1;
   EPwm2Regs.DBRED = RE_DELAY;
   EPwm2Regs.DBFED = FE_DELAY;

//   #ifdef HPROTECTION
   EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP3OUT;
   EPwm2Regs.DCTRIPSEL.bit.DCBHCOMPSEL = DC_COMP3OUT;
   EPwm2Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;
   EPwm2Regs.TZDCSEL.bit.DCBEVT1 = TZ_DCAH_HI;
   EPwm2Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT1;
   EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
   EPwm2Regs.TZSEL.bit.DCAEVT1 = 1;
   EPwm2Regs.TZSEL.bit.DCBEVT1 = 1;
//   #endif

//  EPwm2Regs.TZSEL.bit.OSHT1 = 1;
   EPwm2Regs.TZSEL.bit.OSHT2 = 1;
//   EPwm2Regs.TZSEL.bit.OSHT3 = 1;

  // What do we want the TZ1 and TZ2 to do?
  EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
  EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_HI;
  EPwm2Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
  EPwm2Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_HI;
  // Enable TZ interrupt
  EPwm2Regs.TZEINT.bit.OST = 1;
  EPwm2Regs.TZEINT.bit.DCAEVT1 = 1;
  EPwm2Regs.TZEINT.bit.DCBEVT1 = 1;
  EDIS;
}


void InitEPwm3Example()
{
	EALLOW;
   EPwm3Regs.TBPRD = FULLBRIDGEPERIOD;                         // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
   EPwm3Regs.TBCTL.bit.PHSEN=TB_DISABLE;

   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope

   // Setup compare
   EPwm3Regs.CMPA.half.CMPA = FULLBRIDGEDUTY;
//   EPwm3Regs.CMPB = FULLBRIDGEDUTY;
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;              // Set PWM3A on Zero
   EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
   EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;              // Set PWM3A on Zero
   EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
   // EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

   // Active high complementary PWMs - Setup the deadband
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

//   #ifdef HPROTECTION
   EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;
   EPwm3Regs.DCTRIPSEL.bit.DCBHCOMPSEL = DC_COMP1OUT;
   EPwm3Regs.DCFCTL.bit.BLANKE = 0;
//   EPwm3Regs.DCFWINDOW = 0xFF;
   EPwm3Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;
   EPwm3Regs.TZDCSEL.bit.DCBEVT1 = TZ_DCAH_HI;
   EPwm3Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT1;
   EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
   EPwm3Regs.TZSEL.bit.DCAEVT1 = 1;
   EPwm3Regs.TZSEL.bit.DCBEVT1 = 1;
//   #endif

//   EPwm3Regs.TZSEL.bit.OSHT1 = 1;
   EPwm3Regs.TZSEL.bit.OSHT2 = 1;
//   EPwm3Regs.TZSEL.bit.OSHT3 = 1;

    // What do we want the TZ1 and TZ2 to do?
   EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
   EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm3Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
   EPwm3Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO;
   // Enable TZ interrupt
   EPwm3Regs.TZEINT.bit.OST = 1;
   EPwm3Regs.TZEINT.bit.DCAEVT1 = 1;
   EPwm3Regs.TZEINT.bit.DCBEVT1 = 1;
   EDIS;
}

void InitEPwm4Example()
{
	EALLOW;
   EPwm4Regs.TBPRD = ISR100US;                         // Set timer period
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                       // Clear counter
   EPwm4Regs.TBCTL.bit.PHSEN=TB_DISABLE;

   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope

   EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm4Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event
   EDIS;
}


void Adc_Config()
{
// Configure ADC
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	//ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E     = 1;	//Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	//Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL	= 8;	//setup EOC9 to trigger ADCINT1 to fire
	AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 0x1;	//set SOC0 channel select to ADCINB6
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;	//set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 0x7;
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC2CTL.bit.CHSEL 	= 0x8;
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC2CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC3CTL.bit.CHSEL 	= 0x9;
	AdcRegs.ADCSOC3CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC3CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC4CTL.bit.CHSEL 	= 0xA;
	AdcRegs.ADCSOC4CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC4CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC5CTL.bit.CHSEL 	= 0xB;
	AdcRegs.ADCSOC5CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC5CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC6CTL.bit.CHSEL 	= 0xC;
	AdcRegs.ADCSOC6CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC6CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC7CTL.bit.CHSEL 	= 0xE;
	AdcRegs.ADCSOC7CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC7CTL.bit.ACQPS 	= 6;
	AdcRegs.ADCSOC8CTL.bit.CHSEL 	= 0xF;
	AdcRegs.ADCSOC8CTL.bit.TRIGSEL 	= 5;
	AdcRegs.ADCSOC8CTL.bit.ACQPS 	= 6;
	EDIS;
}

void InitComparator()
{
	EALLOW;
	Comp1Regs.COMPCTL.bit.SYNCSEL = 0;  // Don't sync with SYSCLK
	Comp1Regs.COMPCTL.bit.CMPINV = 0;     // Output Low when true
	Comp1Regs.COMPCTL.bit.COMPSOURCE = 0;  // Use internal DAC
	Comp1Regs.COMPCTL.bit.COMPDACEN = 1; // Enable DAC
	//Comp1Regs.COMPCTL.all = 0x11;
	Comp1Regs.DACVAL.bit.DACVAL = 680;//800				    // Set DAC output to midpoint
	Comp2Regs.COMPCTL.bit.SYNCSEL = 0;  // Don't sync with SYSCLK
	Comp2Regs.COMPCTL.bit.CMPINV = 0;     // Output Low when true
	Comp2Regs.COMPCTL.bit.COMPSOURCE = 0;  // Use internal DAC
	Comp2Regs.COMPCTL.bit.COMPDACEN = 1; // Enable DAC
	//Comp2Regs.COMPCTL.all = 0x11;                  // Power up Comparator 1 locally
	Comp2Regs.DACVAL.bit.DACVAL = 500;				    // Set DAC output to midpoint
	Comp3Regs.COMPCTL.bit.SYNCSEL = 0;  // Don't sync with SYSCLK
	Comp3Regs.COMPCTL.bit.CMPINV = 0;     // Output Low when true
	Comp3Regs.COMPCTL.bit.COMPSOURCE = 0;  // Use internal DAC
	Comp3Regs.COMPCTL.bit.COMPDACEN = 1; // Enable DAC
	//Comp3Regs.COMPCTL.all = 0x11;                  // Power up Comparator 1 locally
	Comp3Regs.DACVAL.bit.DACVAL = 500;				    // Set DAC output to midpoint
	EDIS;
}


interrupt void epwm4_isr(void)
{

    EALLOW;
    IER |= 0x001;                         // Set global priority by adjusting IER
    IER &= 0x001;
    asm("       NOP");                    // Wait one cycle
    EINT;
    statemachineflag = 1;
    statemachine();

    Tx_timeout++;                   //added for rs485
    Rx_timeout++;
    Rx_time_count++;
    GSM_Restart_Count++;

    RS485_TXEnable_Count++;
    gsm_count++;
    if (gsm_count > 40000)gsm_count=40000;
    GSM_Repeat_Count++;
    GSM_DelayCount++;     // Delay for GPRS Data Transfer for Master-Slave
    if ( GSM_DelayCount > 40000)GSM_DelayCount=40000;

    // Clear INT flag for this timer
    EPwm4Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.bit.ACK3 = 1;
    DINT;
    EDIS;
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

}


interrupt void ecap1_isr(void)
{
    // GpioDataRegs.GPASET.bit.GPIO11 = 1;

     EALLOW;
     ECAP1 = ECap1Regs.CAP1;
     ECAP2 = ECap1Regs.CAP2;

     IER |= 0x001;                         // Set global priority by adjusting IER
     IER &= 0x001;
     asm("       NOP");                    // Wait one cycle
     EINT;
     ECap1Regs.ECCLR.bit.CEVT2 = 1;
     ECap1Regs.ECCLR.bit.INT = 1;
     // Acknowledge this interrupt to receive more interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    DINT;

    EDIS;
   // GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

}





void InitECapture()
{
    EALLOW;
    ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

    // Configure peripheral registers
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;      // continuos
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;        // Stop at 4 events
    ECap1Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
    ECap1Regs.ECCTL1.bit.CAP2POL = 0;          // Rising edge
    ECap1Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
    ECap1Regs.ECCTL1.bit.CAP4POL = 0;          // Rising edge
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation
    ECap1Regs.ECCTL2.bit.CAP_APWM = 0;
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;        // disable
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 0;            // continuous mode
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
    ECap1Regs.ECEINT.bit.CEVT2 = 1;            // 2 events = interrupt
    EDIS;
}






//===========================================================================
// No more.
//===========================================================================


void LED_Indication(void)
{
	if (systemState == GRIDMODE)
	{
	   LED_DRV1 = ON;
	   LED_DRV2 = ON;
	}

	else if(systemState == OFFGRIDMODE)
	{
	   LED_DRV1 = OFF;
	   LED_DRV2 = ON;
	}

	else if((systemState == SYSTEMERROR))
	{
	   //LED_DRV1 = ON;
	   LED_DRV2 = OFF;

	}


}



void ModeChangeover(void)
{
    if(gridFlag == 0)
    {
        InitEPwmSyncGpio();

    }
    else if (gridFlag==1)
    {
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;   // Configures GPIO6 for GPIO as input
        EDIS;

    }
}






//===========================================================================
// No more.
//===========================================================================



