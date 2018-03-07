/***************************************************************************************
PROJECT : MICRO GRID INVERTER - PHASE_1
CLIENT  : KRIPYA TECHNOLOGIES (INDIA) PVT LTD
****************************************************************************************
NOTES   : MAXDUTYCLAMPED changed from 24575 (75%) to 15729 (48%)
	  Added AMC_1100_SENSING, AC_VALUE2 for AMC sensing

***************************************************************************************/

#define ON 1
#define OFF 0

#define OPTO_DRV1 GpioDataRegs.GPADAT.bit.GPIO21
#define OPTO_DRV2 GpioDataRegs.GPBDAT.bit.GPIO33


#define LED_DRV1 GpioDataRegs.GPADAT.bit.GPIO31
#define LED_DRV2 GpioDataRegs.GPADAT.bit.GPIO30
//#define LED_DRV3 GpioDataRegs.GPADAT.bit.GPIO29

#define FLYBACKPERIOD 1071  		 	 					// ~56kHz switching frequency (1/56kHz/1.06ns) = 16864
#define FULLBRIDGEPERIOD (FLYBACKPERIOD >> 2)				// 228kHz switching frequency (1/228kHz/1.04ns) = 4216 
#define FLYBACKINTERLEAVEDPHASE (FLYBACKPERIOD >> 1)   	 	// 180Deg Phase-shift (flybackPeriod/2)
#define FULLBRIDGEDUTY (FULLBRIDGEPERIOD >> 1)

#define ZERO 	 0
#define ISR100US 6000

#define RE_DELAY 10
#define FE_DELAY 30

#define MAXDUTYCLAMPED 24575								// Maximum duty cycle of 75% in Q15 format (24575)

// 180 degree is equal to 32767
#define ONEHUNDREDSEVENTYFIVEDEGREE 32300			// 31800					// Degree to turn off the Full-Bridge unfolding circuit
#define NINETYDEGREE 16383


#define INVERTER_OVERVOLTAGE_LIMIT 7272  //7272=250V //7360=253V 	// Inverter Output Voltage ((264 * sqrt(2)) / turns ratio of TR1) * (gain of u6)
															// Converted into ADC counts and Q15 format	minus the offset
#define INVERTER_OVERVOLTAGE_LIMIT_HYS 7200 //7200=248V 	// Inverter Output Over Voltage Hysteresis ~3Vac RMS

#define INVERTER_UNDERVOLTAGE_LIMIT  5800//5800= 197  //5750 =195.5              //11400// Inverter Output Voltage ((210 * sqrt(2)) / turns ratio of TR1) * (gain of u6)
															// Converted into ADC counts and Q15 format minus the offset
#define INVERTER_UNDERVOLTAGE_LIMIT_HYS 5880 //5850 //4800~198V	6150//           //11700// Inverter Output Under Voltage Hysteresis ~3Vac RMS

#define INVERTER_OVERVOLTAGE_LIMIT_LEVEL 9000//9400//9650

#define INVERTER_UNDERVOLTAGE_LIMIT_LEVEL 3500//3380  //5000 ~170V


// 50Hz Operating Limits
#define INVERTERPERIOD50HZMIN 570 //568 = 50.8Hz //571 = 51Hz  							//52hz// Frequency set points for 50Hz (47Hz - 53Hz), ADC ISR rate is 17.87us
#define INVERTERPERIOD50HZMAX 553 // 551=49.2Hz //549 = 49Hz
#define INVERTERPERIODHYS 		3							// Hysteresis for Inverter frequency

// 60Hz Operating Limits
#define INVERTERPERIOD60HZMIN 461							// Frequency set points for 60Hz (59.3Hz - 60.5Hz), ADC ISR rate is 17.87us
#define INVERTERPERIOD60HZMAX 473

// PV Panel Operating Voltage Limits

#define PVPANEL_MPP_LIMIT 29150		//50V						// Max MPP Voltage ~46Vdc

#define PVPANEL_OVERVOLTAGE_LIMIT 29150 //50V //25920//25920~45v//26000	//45V					// ~52V open circuit voltage, 52 * (R74/(R74+R72))
															// Converted into ADC counts and Q15 format
#define PVPANEL_OVERVOLTAGE_LIMIT_HYS 27401 //47V //24700//24700~43v//24882	//43V				// PV Panel Over Voltage Hysteresis ~2V

#define PVPANEL_UNDERVOLTAGE_LIMIT 11077 //19V //11680//11680~20V//14550  //25V 				// Minimum Operating Voltage ~20Vdc, 18.5 * (R74/(R74+R72))
															// Converted into ADC counts and Q15 format		
#define PVPANEL_UNDERVOLTAGE_LIMIT_HYS 13325//13325~23v//12746//16222	//28V			// PV Panel Under Voltage Hysteresis ~2V

// Inverter Output Current Max Rating
#define INVERTER_OUTPUTCURRENT_MAX  16000 //14000 ~1.089A=250W	// At 210Vac full load, expected peak inverter output current ~1.6A.
															// This current * gain of 185mV/A * gain of U5, measured by the ADC 
															// and converted to Q15. Subtract off the offset 
  
// AC Current Offset Limits 
#define MINOFFSETCURRENT 12896								// Nominal ~1.6V, convert to ADC and Q15
#define MAXOFFSETCURRENT 17856		

// Maximum Output Power Limit
#define DERATINGFACTOR 4160
#define DERATINGSLOPE 25000//25673							// 1.567 in Q14 format
#define DERATINGCONSTANT 18235								// 1.113 in Q14 format

// Power De-rating Limits
#define POWERDERATINGLIMIT 		14300						// Corresponds to 24.5Vdc

// Power Decrement Factor for anti-islanding 
#define POWERDECREMENTFACTOR Q15(.02)						// This is ~2% which is an initial power de-rating

// Maximum Average Flyback Current Limit
#define MAXFLYBACKCURRENT 29000								// Max average current through flyback stage ~3V on ADC

// 12V Drive Supply Operating Limits
#define MAXDRIVEVOLTAGE 30766								// 12.5V drive supply * (R2/(R2+R1)), convert to ADC and then Q15
#define MINDRIVEVOLTAGE	27150 //27100~ 11v 28300								// 11.5V in Q15 format

// Temperature Operating Limits
#define MAXTEMPERATURE 		13500 //88 //13227=80 //12400~75							// Vout = (75C * 10mV/C) + .5V = 1.25V, Convert to ADC reading then Q15
#define MAXTEMPERATURE_HYS 	13000 //78 //12400=75 //11400~65							// 65C degrees, Convert to ADC reading and then Q15



// MPPT Definitions
#define MPPTFACTORMINIMUM 100								// Minimum MPPT Factor to start the inverter
#define MININCREMENTMPPTFACTOR 20//10//20//20	56						// Values should be small enough to avoid large voltage
#define MAXINCREMENTMPPTFACTOR 100//100//30		84					// deviations when operating at MPP
#define MINDECREMENTMPPTFACTOR 20//10//20//20   56
#define MAXDECREMENTMPPTFACTOR 100//80//30   84
#define LARGEVOLTAGEDIFFERENCE 30//8//40//24//8   22
#define PVPANEL_VOLTAGEDROP	-600//-1200//-600							// Change in PV panel average voltage due to large AC Voltage Change
#define MPPTFACTORMAXIMUM 24500//24500														// PV Voltage change of ~0.5V * (7.5/127.5), convert to ADC and Q15



/*
// Burst Mode Definitions
#define BURSTMODECOUNT			6700						// Burst Mode Count (~1 minute) - 1/(2xVac) * counter
#define BURSTMODETHRESHOLDLOW	525//1600						// Power to which burst mode is applied < ~15%
#define BURSTMODETHRESHOLDHIGH	578//2800						// Power to which burst mode is removed ~18%
*/

// Various #defines
#define ZEROCROSSCOUNT		60								// Number of zero cross events required before switching to Day Mode
#define RESTARTCOUNT		30000					//60000		// 10000 * 100us - 1s of no faults to switch to system startup
#define CRITICALFAULTCOUNT 	10000							// 20000 * 100us - 2s before trying to restart the system
#define LOADBALCOUNT		2								// Rate to execute the load balance routine in the state machine (Day mode) 1/10th main compensator
#define PVPANEL_40V			23341							// Panel Voltage 40V, used for changing CMP current reference
#define PVPANEL_30V			17500							// Panel Voltage 30V, used for changing CMP current reference


/*
// Coefficients for PI Controller
#define Ra _IQ15(0.055)		//0.14	// 0.18
#define Rsa _IQ15(0.028)		//0.02	// 0.02
*/

// Coefficients for Load Balancing 
#define MAXBALANCE _IQ15(0.01)
#define KAQ15 _IQ15(0.065)
#define KSAQ15 _IQ15(0.01)

// System State Definitions
#define SYSTEMSTARTUP 0
#define GRIDMODE 1
#define SYSTEMERROR 2
#define OFFGRIDMODE 3

// Input Switch State
#define SWITCHOFF 0
#define SWITCHON 1

// Fault State Definitions
#define NO_FAULT 0
#define PV_PANEL_VOLTAGE 1
#define INVERTER_FREQUENCY 2	
#define INVERTER_VOLTAGE 3
#define INVERTER_OVERCURRENT 4
#define FLYBACK_OVERCURRENT 5
#define TEMPERATURE 6
#define DRIVE_SUPPLY 7
#define FLYBACK_OUTPUT_VOLTAGE 8
#define SHORT_CIRCUIT 9      //REFERENCE_VOLTAGE
#define ACCURRENT_OFFSET 10
#define HARDWAREZEROCROSS 11

// State for Full Bridge Drive
#define FULLBRIDGE_Q3Q4_ACTIVE 1
#define FULLBRIDGE_INACTIVE_2ND_QUADRANT 2
#define FULLBRIDGE_Q2Q5_ACTIVE 3
#define FULLBRIDGE_INACTIVE_4TH_QUADRANT 4


// Include the following #define in the source code to run system on the bench using a DC power supply. 
// This will remove MPP-tracking so we need to define MPPTFactorMaximum ~25000 (max power) and 
// create another way for a controlled softstart

//#define BENCHTESTING

#ifdef BENCHTESTING
//#define MPPTFACTOR_BENCHTESTING 20000  //22900
#define MPPTFACTORINCREMENT 6					// Softstart increments MPPT by MPPTFACTORINCREMENT every 10ms
#define MPPTCOUNT	100							// Slow down the increment rate to -> 100 * 100us
#endif


// Include only ONE of the following #defines to use any of three arrays (array1, array2, and array3) for DMCI debug purposes
// DMCI_ISR is at a rate of 17us * (dmciCounter)
// DMCI_STATEMACHINE is at a rate of 100us
// DMCI_MPPT is at a rate of 3*inverterFrequency
 
//#define DMCI_ISR
//#define DMCI_STATEMACHINE
//#define DMCI_MPPT


/***********************************************************/
#define AMC_1100_SENSING
#define AC_VALUE1 _IQ15(1.2)
#define AC_VALUE2 _IQ15(1)  //1.5
#define AC_VALUE3 _IQ15(1.15)
/***********************************************************/

#define HPROTECTION

// Off grid defines
#define AC_VOLTAGE 	    15200    //15200:225V           //15200					// Original value for 220V :14100
#define FB_DUTY_LIMIT 	100
#define Ra_off _IQ15(0.12) 		//0.2    // 0.45     	// new 0.2   old 0.2
#define Rsa_off _IQ15(0.05)		//0.1    //0.128	    // new 0.18   old 0.1

