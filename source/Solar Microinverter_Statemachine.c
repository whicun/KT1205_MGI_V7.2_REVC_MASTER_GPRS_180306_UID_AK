/*
 * Solar Microinverter_Statemachine.c
 *
 *  Created on: Aug 14, 2014
 *      Author: Admin
 */


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Solar Microinverter_Statemachine.h"
#include "IQmathLib.h"

void inverterFrequencyCheck(void);
void ac_cal(void);
void Grid_amcbias_cal(void);
void inverterFrequencyCheck_OFFGRID(void);
void faulttrip(void);
void overcurrenttrip(void);
void Grid_PVvoltagetrip(void);
void Gridvoltagetrip(void);  // need to remove
void OffGridvoltagetrip(void);   // need to remove
void OFFGrid_PVvoltagetrip(void);
void Drivesupply(void);
void Temperaturefault(void);
void Grid_over_under_voltage_trip(void);
void PWMSTOP(void);
void variablePI (void);
void tempreg(void);


// System Fault Variables
unsigned char storeFaultState = 0, criticalFaultRestartFlag = 0;
unsigned char pvPanelOverVoltageFlag = 0, pvPanelUnderVoltageFlag = 0, overTemperatureFlag = 0, pvUnderVoltageCounter = 0;
unsigned char tempFaultCnt = 0, driveSupplyFaultCnt = 0, mpptFaultCounter = 0;
unsigned int inverterOutputOverCurrent = 0;
unsigned char faultIndicationResetCounter = 0;


// Start-up and Restart Variables
unsigned char zeroCrossDelay = 45, startFullBridgeFlag = 0;
unsigned char zeroCrossDelayNom = 30, zeroCrossDelayMin = 22, zeroCrossDelayMax = 37;
unsigned char acCurrentOffsetFlag = 0, acCurrentOffsetCounter = 0;
unsigned int  acCurrentOffset = 0, faultindicationcounter = 0;
unsigned int inverterPeriodMin = INVERTERPERIOD50HZMIN, inverterPeriodMax = INVERTERPERIOD50HZMAX;
long long unsigned int acCurrentOffsetAverage = 0,systemRestartCounter = 0;

// MPPT and Load Balance Variables
unsigned char  mpptCounter = 0, mpptStartUpFlag = 0;
unsigned int inputVoltageAverage = 0, inputCurrentAverage = 0, openCircuitVoltage = 0;
volatile unsigned int mpptFactor = MPPTFACTORMINIMUM;
int inputPower = 0, prevInputVoltageAverage = 0;

// Startup in system error, If a fault is detected
// faultstate will change during the restart counter.
unsigned char systemState = SYSTEMERROR;
unsigned char switchState = SWITCHOFF;
unsigned char faultState = NO_FAULT;

// Externally Defined Variables
extern int sfst_acVoltage;					// 220V AC refers to 8100 in Q15 (220*16384/445)
extern unsigned char avgInputDataReadyFlag, startupZeroCrossCounter;
extern unsigned char criticalFaultFlag, acCurrentOffsetFlag, ninetyDegreeDetectFlag;

extern unsigned int mpptFactorMaximum, numberofSamples;
extern unsigned int measuredTemperature, driveSupplyVoltage;
extern unsigned int inverterPeriod;
extern int peakInverterOutputVoltage, inverterOutputCurrent;
extern volatile int peakGridVoltage;


// Off grid variables
volatile unsigned char  gridFlag = 0;
unsigned int inverterOverCurrentCounter= 0;
//gridVoltageCounter = 0
extern unsigned int inverterPeriodCounter, inverterPeriodCounter_off, softstart_counter;

volatile unsigned int PV_Voltage_Tripcount = 0,PV_Voltage_Tripflag =0;
volatile unsigned long int Restart_count = 20000;
long unsigned int pv_Input_voltage_sum = 0;
volatile unsigned int pv_Input_voltage_average = 0;
extern unsigned int pvPanelVoltage;

unsigned int PV_INPUT_COUNT_OFF=0;




//extern int maxGridVoltage;
// Timer 3 Interrupt (300ms interrupt) for Fault indication



extern volatile int peakInverterOutputCurrent;
int maxInverterOutputCurrentflag = 0;
unsigned int overcurrentcount = 0;
extern unsigned int maxInverterOutputCurrent;
int deltaV = 0, prevInputPower = 0;
unsigned char mpptScaleFactor = 0;

volatile unsigned int MPPTFACTOR_BENCHTESTING = 20000;


volatile unsigned int acoffset_cal = 1,AMC_cal = 0,acoffset_cal2=1;
long unsigned int AC_Voltage_Average = 0;
long unsigned int AC_grid_voltage_average = 0;
unsigned int acoffsetcounter = 0;
Uint16 AC_grid_offset = 0;// 12700;
Uint16 AC_inv_offset = 0;
int AC_grid_voltage = 0,criticalFaultCounter=0;
int Max_Pos_grid_voltage = 0;
int Peak_Pos_grid_voltage = 0;
int Max_Neg_grid_voltage = 0;
int Peak_Neg_grid_voltage = 0;
int PeakError = 0;
Uint16 ac_counter = 0;
Uint16 faltu_counter = 0;
unsigned int dummycounter=0, dummycounter1 =0, dummycounter2=0,dummycounter3 =0,dummycounter4 =0,relaycounter1=0,relaycounter2=0;

volatile float Ra_float = 0.2, Rsa_float = 0.03; //0.03 //Ra_float = 0.055, Rsa_float = 0.028;
extern volatile int flag_10ms;
int start_current_fault_check = 0;
extern volatile unsigned int Ra, Rsa;
unsigned int VariablePIFlag = 0;

Uint32 AMCbiascounter = 0;
extern unsigned char flybacktron;
extern unsigned char flybackshutdownflag;
extern unsigned int startupZeroCrossCounter1;



extern unsigned int averageRectifiedCurrent;

unsigned int PVTripClearCounter =0;
extern unsigned char frequencyFaultCounter;
unsigned char frequencyFaultclearCounter = 0, offGridFlag=0;
extern volatile unsigned int Trip_inverterPeriod, Trip_zeroCrossDetectFlag,StatemachineCheckFlag;

extern unsigned int avgPeakGridVotlage,avgPeakGridVotlage1;
extern Uint32 frequancy_mains;

extern unsigned char zeroCrossCount;

extern unsigned char zeroCrossDetectFlag;
extern unsigned char Zcd1Flag;

extern volatile unsigned char grid_present_flag;
extern unsigned int OffGridStartupFlag;

unsigned int PWMSetFlag=0,GridVOutofRangeFlag =1,GridFOutofRangeFlag =1;

extern int gridVoltage, rectifiedGridVoltage;

#pragma CODE_SECTION(statemachine, "ramfuncs");
#pragma CODE_SECTION(variablePI, "ramfuncs");
#pragma CODE_SECTION(faulttrip, "ramfuncs");



// Timer 3 Interrupt (300ms interrupt) for Fault indication
void faultindication()
{
	static unsigned char ledCounter = 0, interruptCounter = 0;

	faultIndicationResetCounter++;
	if(ledCounter < storeFaultState)
	{
		LED_DRV1 ^= ON;										// Blink LED to indicate fault, storeFaultState is 2x to account for toggle Off
		ledCounter++;
	}
	else if (ledCounter < (storeFaultState + 3))			// Wait three interrupts (Clear indication blinking has stopped)
	{
		ledCounter++;
	}
	else
	{
		ledCounter = 0;
	}

	if ((systemState == GRIDMODE) || (systemState == OFFGRIDMODE))
	{
		interruptCounter++;
	}

	// InterruptCounter is used to diplay the last known fault for some time after
	// the system restarts (300ms * 200 ~ 1 minute)
	if((faultState == NO_FAULT) && (interruptCounter >= 200))
	{
		//Disable this interrupt if the fault is removed and the delay has passed
		ledCounter = 0;
		interruptCounter = 0;
		LED_DRV1 = OFF;
		LED_DRV1 = OFF;
		storeFaultState = 0;
	}

	if (faultIndicationResetCounter > 200)
	{
		faultIndicationResetCounter = 0;
	}


}



void statemachine()
{

//   GpioDataRegs.GPASET.bit.GPIO19 = 1;
  // GpioDataRegs.GPASET.bit.GPIO11 = 1;


    gridFlag = grid_present_flag;


      StatemachineCheckFlag=0;




	//Grid AMC automatic biasing - starts here
	AMCbiascounter++;

	if(AMCbiascounter >= 18000000)  //18000000  every 30 minutes AMC bias is calibrated
	{
		AMCbiascounter = 0;
		AMC_cal = 1;
	}

	if(pvPanelVoltage < 9000)
	{
		acoffset_cal = 1;
	}

	ac_cal();
	Grid_amcbias_cal();  // when AMC_cal is set this function will execute

	//Grid AMC automatic biasing - Ends here
	// off grid PV voltage averaging
	PV_INPUT_COUNT_OFF++;

	if(PV_INPUT_COUNT_OFF <= 100) //10ms
	{
		pv_Input_voltage_sum = pv_Input_voltage_sum + pvPanelVoltage;
	}
	else
	{
		PV_INPUT_COUNT_OFF=0;
		pv_Input_voltage_average = pv_Input_voltage_sum/100;
		pv_Input_voltage_sum=0;

	}



	faulttrip();
    tempreg();


	if ((gridFlag == 0) && (systemState == GRIDMODE))
	{
	        faultState = INVERTER_VOLTAGE;
			systemState = SYSTEMERROR;

	}

	if ((gridFlag == 1) && (systemState == OFFGRIDMODE))
	{
            faultState = INVERTER_VOLTAGE;
			systemState = SYSTEMERROR;
			GridVOutofRangeFlag =1;
			GridFOutofRangeFlag =1;
	}


	// Check the fault state, if it is not equal to No Fault then set systemState = SYSTEM_ERROR
	if (faultState != NO_FAULT)
	{
		systemState = SYSTEMERROR;
	}

	// Check the On/Off switch state
	if(GpioDataRegs.GPADAT.bit.GPIO20 == 0)
	{
		switchState = SWITCHON;
	}
	else
	{
		//put system state in systemError
		switchState = SWITCHOFF;
		systemState = SYSTEMERROR;
		LED_DRV1 = OFF;
		LED_DRV2 = OFF;
	}

    switch(systemState)
    {

	//GpioDataRegs.GPACLEAR.bit.GPIO10= 1;
        case SYSTEMSTARTUP:
		{
			if(acCurrentOffsetFlag == 0)
			{
				acCurrentOffsetAverage = acCurrentOffsetAverage + inverterOutputCurrent;

				if(acCurrentOffsetCounter >= 2047)
				{
					acCurrentOffset = (acCurrentOffsetAverage >> 11);
					inverterOutputOverCurrent = INVERTER_OUTPUTCURRENT_MAX + (16383 - acCurrentOffset);
					acCurrentOffsetFlag = 1;
					acCurrentOffsetCounter = 0;
					acCurrentOffsetAverage = 0;
					peakInverterOutputCurrent = 0;

					maxInverterOutputCurrent = 0;
					if((acCurrentOffset < MINOFFSETCURRENT) || (acCurrentOffset > MAXOFFSETCURRENT))
					{
						if(faultState == NO_FAULT)
						{
							faultState = ACCURRENT_OFFSET;
						}
					}
				}
				else
				{
					acCurrentOffsetCounter++;
				}
			}

			flybackshutdownflag = 1;
			if(gridFlag == 1)
			{

				if(avgInputDataReadyFlag == 1)
				{
					// During system startup read the PV panel Voltage (open circuit voltage). This information
					// is used to help speed up the time to find MPP
					openCircuitVoltage = inputVoltageAverage;
					avgInputDataReadyFlag = 0;
				}

				// Read AC Current offset during startup mode and verify data
				// Needs to be completed before the full-bridge is enabled during system startup


				// At system Start-up, enable the Full-Bridge circuit (@ peak of AC Cycle) before the flyback circuit is enabled
				// If this is not done, the flyback output will have high DC voltage and when the full-bridge is enabled at the
				// zero cross there is a large dv/dt and the output current will have a large glitch and also trip the flyback
				// OVP circuit

				if((startupZeroCrossCounter >= (ZEROCROSSCOUNT>>1)) && (ninetyDegreeDetectFlag == 1))
				{
					startFullBridgeFlag = 1;				// Set Flag to start full-bridge drive
				}

				// After several consecutive zero crossings switch to Day Mode
				if(startupZeroCrossCounter > ZEROCROSSCOUNT)
				{
					if ((inverterPeriod > INVERTERPERIOD60HZMIN ) && (inverterPeriod <= INVERTERPERIOD60HZMAX))
					{
						inverterPeriodMin = INVERTERPERIOD60HZMIN;
						inverterPeriodMax = INVERTERPERIOD60HZMAX;

						// Delay at the zero crossings to allow AC voltage to reach flyback voltage
						zeroCrossDelayNom = 20;
						zeroCrossDelayMax = 22;			// Delay slightly varies with AC voltage
						zeroCrossDelayMin = 18;

						zeroCrossDelay = zeroCrossDelayNom;		// Wait (17us*delay) at the zero cross before turning on flyback/full-bridge
					}
					else
					{
						inverterPeriodMin = INVERTERPERIOD50HZMIN;
						inverterPeriodMax = INVERTERPERIOD50HZMAX;

						// Delay at the zero crossings to allow AC voltage to reach flyback voltage
						zeroCrossDelayNom = 5;			// Originally 30;
						zeroCrossDelayMax = 5;			// Originally 35;			// Delay slightly varies with AC voltage
						zeroCrossDelayMin = 5;			// Originally 25;

						zeroCrossDelay = zeroCrossDelayNom;		// Wait (17us*delay) at the zero cross before turning on flyback/full-bridge
					}


					// Acknowledge this interrupt to receive more interrupts from group 2
					// Change system state to Day Mode
					if(inputVoltageAverage > 14000)
					{

						EALLOW;
						EPwm1Regs.TZCLR.all = 0x2D;
						EPwm2Regs.TZCLR.all = 0x2D;
						EPwm3Regs.TZCLR.all = 0x2D;
						EDIS;
						dummycounter++;
						//GRID
									//BClear, ASet
									//OFFGRID
									//BSet, Aclear

		                 GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
		                 GpioDataRegs.GPASET.bit.GPIO23 = 1;

						if  (dummycounter >= 50000)  //100000 10s
						{
							systemState = GRIDMODE;
						}
					}

				}
			}
			else if((gridFlag == 0) /*&& ((avgPeakGridVotlage < INVERTER_UNDERVOLTAGE_LIMIT_LEVEL)|| (avgPeakGridVotlage > INVERTER_OVERVOLTAGE_LIMIT_LEVEL))*/)
			{
				if(pv_Input_voltage_average > 14000)
				{
					dummycounter1++;
					EALLOW;
					EPwm1Regs.TZCLR.all = 0x2D;
					EPwm2Regs.TZCLR.all = 0x2D;
					EPwm3Regs.TZCLR.all = 0x2D;
					PieCtrlRegs.PIEACK.bit.ACK2 = 1;
					EDIS;


					//RevC
					GpioDataRegs.GPBSET.bit.GPIO32 = 1;  //Relay Drive1 in off grid mode
					GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;  //Relay Drive2 in off grid mode
					inverterPeriodCounter_off = 0;
					startFullBridgeFlag = 1;
					if(dummycounter1 >= 20000)
					{
						systemState = OFFGRIDMODE;
						//GpioDataRegs.GPASET.bit.GPIO8 = 1;
					}

				}
			}
		}
		break;
		case GRIDMODE:
		{
			acoffset_cal2=0;
			dummycounter4=0;
			relaycounter1=0;
			relaycounter2=0;
			//GRID
			//BClear, ASet
			//OFFGRID
			//BSet, Aclear
			inverterPeriodCounter_off=0;

			if(startupZeroCrossCounter1 >= 15)//&& (ninetyDegreeDetectFlag == 1))
			{
				flybackshutdownflag = 0;
			}
			dummycounter=0;
			dummycounter3=0;

			dummycounter2++;
			if(dummycounter2>= 30000)
			{
				dummycounter2=31005;
				GpioDataRegs.GPBSET.bit.GPIO32 = 1;
			}
			
			
			// When average input voltage and average input current are available call MPPT Routine
			if(avgInputDataReadyFlag == 1)
			{
			    VariablePIFlag = 1;
				GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;  //clear PWMshutdown
				offGridFlag=0;

				#ifndef BENCHTESTING
//				GpioDataRegs.GPASET.bit.GPIO11 = 1;
				MPPTRoutine();			// Call MPPT Routine when data is ready
//				GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
				#endif


				// Change Current Reference based on the PV panel voltage
				if(inputVoltageAverage >= PVPANEL_40V)
				{
					EALLOW;
					Comp2Regs.DACVAL.bit.DACVAL = 190;	//155		// Consider current at ~39V
					Comp3Regs.DACVAL.bit.DACVAL = 190;
					EDIS;
				}
				else if(inputVoltageAverage >= PVPANEL_30V)
				{
					EALLOW;

					Comp2Regs.DACVAL.bit.DACVAL = 250;	//171		// Consider current at ~29V
					Comp3Regs.DACVAL.bit.DACVAL = 250;
					EDIS;
				}
				else
				{
					EALLOW;

					Comp2Regs.DACVAL.bit.DACVAL = 400;	//190		// Consider current at ~20V
					Comp3Regs.DACVAL.bit.DACVAL = 400;
					EDIS;
				}


				avgInputDataReadyFlag = 0;
			}
			// Software for soft-start when using a bench supply as MPP-Tracking is removed
			#ifdef BENCHTESTING

			mpptCounter++;

			if(mpptFactor < MPPTFACTOR_BENCHTESTING)
			{
				if(mpptCounter >= MPPTCOUNT)
				{
					mpptFactor = mpptFactor + MPPTFACTORINCREMENT;
					mpptCounter = 0;
				}
			}
			else
			{
				mpptFactor = MPPTFACTOR_BENCHTESTING;
				mpptCounter = 0;
			}

			#endif

		}
		break;

		case OFFGRIDMODE:
		{
			acoffset_cal2=0;
			dummycounter1=0;
			dummycounter4=0;
			dummycounter3=0;
			offGridFlag=1;
			relaycounter1=0;
			relaycounter2=0;
			VariablePIFlag=0;

			// When average input voltage and average input current are available call MPPT Routine
			if(avgInputDataReadyFlag == 1)
			{

                // Change Current Reference based on the PV panel voltage
                if(inputVoltageAverage >= PVPANEL_40V)
                {
                    EALLOW;
                    Comp2Regs.DACVAL.bit.DACVAL = 190;  //155       // Consider current at ~39V
                    Comp3Regs.DACVAL.bit.DACVAL = 190;
                    EDIS;
                }
                else if(inputVoltageAverage >= PVPANEL_30V)
                {
                    EALLOW;
                    Comp2Regs.DACVAL.bit.DACVAL = 210;  //171       // Consider current at ~29V
                    Comp3Regs.DACVAL.bit.DACVAL = 210;
                    EDIS;
                }
                else
                {
                    EALLOW;
                    Comp2Regs.DACVAL.bit.DACVAL = 250;  //190       // Consider current at ~20V
                    Comp3Regs.DACVAL.bit.DACVAL = 250;
                    EDIS;
                }

				avgInputDataReadyFlag = 0;
			}
		}
				break;

		case SYSTEMERROR:
		{
			//GRID
			//BClear, ASet
			//OFFGRID
			//BSet, Aclear
			PWMSTOP();
            GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
			if (pvPanelVoltage < PVPANEL_UNDERVOLTAGE_LIMIT)
			{
				GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
				GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
		        pvPanelUnderVoltageFlag = 1;
			}
			else if (pvPanelVoltage > PVPANEL_OVERVOLTAGE_LIMIT)
            {
                GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
                pvPanelOverVoltageFlag = 1;
            }
			else if (pvPanelVoltage > PVPANEL_UNDERVOLTAGE_LIMIT_HYS)
			{
				if(gridFlag == 1)
				{
				    relaycounter1++;
				    if(relaycounter1 > 50000)
				    {
				        relaycounter1 = 50003;
                        relaycounter2=0;
				        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
                        GpioDataRegs.GPASET.bit.GPIO23 = 1;
				    }
				}
				else if(gridFlag == 0)
				{
				    relaycounter2++;
                    if(relaycounter2 > 500)
                    {
                        relaycounter2 = 503;
                        relaycounter1=0;
                        //RevB 4 Wire
                        GpioDataRegs.GPBSET.bit.GPIO32 = 1;  //Relay Drive1 in off grid mode
                        GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;  //Relay Drive2 in off grid mode
                        Restart_count = 20000;


                    }


				}

			}


			OPTO_DRV1 = 1;
			OPTO_DRV2 = 1;
			mpptStartUpFlag = 0;
			startFullBridgeFlag = 0;		// Reset flag to start full-bridge
			acCurrentOffset = 0;
			acCurrentOffsetFlag = 0;		// Allow system to re-calculate AC Current Offset
			mpptFactor = MPPTFACTORMINIMUM;
			inputPower = 0;
			prevInputVoltageAverage = 0;
			softstart_counter = 0;
			sfst_acVoltage = 1000;
			deltaV = 0;
			prevInputPower = 0;
			mpptScaleFactor = 0;
			start_current_fault_check = 0;
			inverterPeriodCounter_off = 0;
			dummycounter2=0;
			PWMSetFlag=0;


            if(gridFlag == 1)
            {
                dummycounter4=0;
                dummycounter3++;
                if(dummycounter3<3)
                {
                    acoffset_cal2=1;
                }
                else
                {
                    acoffset_cal2=0;
                    dummycounter3=55;
                }
            }
            else if (gridFlag==0)
            {
                dummycounter3=0;
                dummycounter4++;
                if(dummycounter4<5)
                {
                    acoffset_cal2=1;
                }
                else
                {
                    acoffset_cal2=0;
                    dummycounter4=15;
                }
            }




			// Critical Faults: AC Current, Flyback Over Voltage, and Flyback Over Current
			// Handle faults differently: Allow system to try to restart only once, if fault
			// is still present then disable PWM module.
			if(criticalFaultFlag == 1)
			{
				criticalFaultCounter++;

				// After 2s remove the critical fault and allow system to try to restart
				if(criticalFaultCounter > CRITICALFAULTCOUNT)
				{
					criticalFaultRestartFlag = 1;

					// As the fault for flyback over current is latched the PWM needs to
					// exit the latched fault mode in order to restart
						EALLOW;
						EPwm1Regs.TZCLR.all = 0x2D;
						EPwm2Regs.TZCLR.all = 0x2D;
						EPwm3Regs.TZCLR.all = 0x2D;
						// Acknowledge this interrupt to receive more interrupts from group 2
						//PieCtrlRegs.PIEACK.bit.ACK2 = 1;
						EDIS;

					criticalFaultFlag = 0;
					criticalFaultCounter = 0;
				}
			}




		//	amcBias2 = 12800;

			if ((switchState == SWITCHON) && (faultState == NO_FAULT))
			{
				// Switch to system startup after no faults have been detected for ~1s

				systemRestartCounter++;

				if(systemRestartCounter >= Restart_count) //)
				{
					systemState = SYSTEMSTARTUP;
					systemRestartCounter = 0;
				}
			}

			else if((switchState == SWITCHON) && (faultState != NO_FAULT))
			{
				// If a fault is present then diplay the fault using T3 and LED D27 on the PCB

				storeFaultState = (faultState << 1);		// x2 to account for "off" time
				// Remove the fault and allow system to try to restart
				// If the fault is ac current offset or HW Zero Cross

				if((faultState == INVERTER_VOLTAGE) && (gridFlag==0))
				{
					faultState = NO_FAULT;
					Restart_count = 20000;  //2sec

				}


				if((faultState == ACCURRENT_OFFSET) || (faultState == INVERTER_OVERCURRENT) || (faultState == FLYBACK_OVERCURRENT) || (faultState == FLYBACK_OUTPUT_VOLTAGE))
				{
					faultState = NO_FAULT;
					Restart_count = 20000;  //2sec

				}

				if(faultState == SHORT_CIRCUIT)
				{
					faultState = NO_FAULT;
					//Restart_count = 20000;  //2sec
				}
				if (faultIndicationResetCounter == 0)
				{
					LED_DRV1 = OFF;
				}

			}
			else if (switchState == SWITCHOFF)
			{
				// Reset Period Limits if switch is off
				// This is only required for testing 50/60 Hz operation
				// without having to cycle power
				inverterPeriodMin = INVERTERPERIOD50HZMIN;
				inverterPeriodMax = INVERTERPERIOD50HZMAX;

				LED_DRV1 = OFF;
				LED_DRV1 = OFF;
			}
		}
		break;
	}


	if ((storeFaultState !=0) && (switchState == SWITCHON))
	{
		faultindicationcounter++;
		if(faultindicationcounter > 3000)
		{
			faultindicationcounter = 0;
			faultindication();
		}
	}

//	EALLOW;
//    // Clear INT flag for this timer
//    EPwm4Regs.ETCLR.bit.INT = 1;
//    // Acknowledge this interrupt to receive more interrupts from group 3
//    PieCtrlRegs.PIEACK.bit.ACK3 = 1;
//    EDIS;
//
//
//    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
 //   GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

}


// This MPPT algorithim implements the Perturbation and Observation method for detecting Maximum
// Power Point. MPPT can be up to ~25000
void MPPTRoutine(void)
{
    //Store off previous inputPower
    prevInputPower = inputPower;

    // Calculate new input power and change in input voltage
	inputPower = (((long)inputVoltageAverage * (long)inputCurrentAverage) >> 15);
	deltaV = inputVoltageAverage - prevInputVoltageAverage;

    // If there is a large drop in voltage decrease mpptFactor at a faster rate
    // Example would be large AC voltage fluctuations

    if(deltaV <= PVPANEL_VOLTAGEDROP)
    {
        mpptFactor = mpptFactor - (mpptFactor >> 2);		// Reset to 75% power
    }


    // To find MPP faster, increase mpptFactor at a faster rate until the operating voltage
    // is less than the opencircuit voltage minus ~3V. Vmp and Voc should always be more
    // than 5-6V difference at any operating temperature or irradiance.
    if((inputVoltageAverage > (openCircuitVoltage - 1650)) && (mpptStartUpFlag == 0))
    {
        mpptFactor += 50;
    }
    else
    {
        mpptStartUpFlag = 1;
        criticalFaultFlag = 0;			// If system gets here without a critical fault, allow system to run as normal
        //criticalFaultRestartFlag = 0;
    }




    if (inputPower > prevInputPower)
    {
        if (deltaV < -LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor += MININCREMENTMPPTFACTOR;
        }
        else if (deltaV < 0)
        {
            mpptFactor += MAXINCREMENTMPPTFACTOR;
        }
        else if (deltaV > LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor -= (MAXDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV > 0)
        {
            mpptFactor -= (MINDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
    }
    else if (inputPower < prevInputPower)
    {
        if (deltaV < -LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor -= (MAXDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV < 0)
        {
            mpptFactor -= (MINDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV > LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor += MAXINCREMENTMPPTFACTOR;
        }
        else if (deltaV > 0)
        {
            mpptFactor += MININCREMENTMPPTFACTOR;
        }
    }

    // Saturate the MPPT limit to min and max values
    if(mpptFactor > mpptFactorMaximum)
    {
        mpptFactor = mpptFactorMaximum;
    }
    else if(mpptFactor < MPPTFACTORMINIMUM)
    {
        mpptFactor = MPPTFACTORMINIMUM;
    }

    // Store off last known input power and input voltage
    prevInputVoltageAverage = inputVoltageAverage;

}





void faulttrip(void)
{

    if(gridFlag == 1)
    {
        Grid_over_under_voltage_trip();
        Grid_PVvoltagetrip();
        overcurrenttrip();
        
        
    }
    else if(gridFlag == 0)
    {
        OFFGrid_PVvoltagetrip();
        
    }
//  ------------------------
    if (gridFlag == 1 && Trip_zeroCrossDetectFlag == 1)
    {
        EALLOW;
        inverterFrequencyCheck();
        EDIS;
    }
    else if(gridFlag == 0)
    {
        if(faultState == INVERTER_FREQUENCY)
        {
            faultState = NO_FAULT;
        }
    }
//  ---------------------------------

	Temperaturefault();
    Drivesupply();
    /*inverter overcurrent trip function is called in end of Grid_over_under_voltage_trip()
     * function for 10ms (every half cycle).
     *
     * overcurrenttrip();
     */
}

void Grid_PVvoltagetrip(void)
{
    // Check PV Panel Voltage Using the Average Input Voltage
    if((inputVoltageAverage > PVPANEL_OVERVOLTAGE_LIMIT) )
    {
        faultState = PV_PANEL_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState =SYSTEMERROR;

        pvPanelOverVoltageFlag = 1;
        PV_Voltage_Tripflag =1;
    }
    else if((inputVoltageAverage < PVPANEL_OVERVOLTAGE_LIMIT_HYS) && (pvPanelOverVoltageFlag == 1))
    {
        if(faultState == PV_PANEL_VOLTAGE)
        {
            faultState = NO_FAULT;
            PV_Voltage_Tripcount =0;
            pvPanelOverVoltageFlag = 0;
            PV_Voltage_Tripcount++;


        }
    }


    // Check PV Panel Minimum Voltage Using Average Input Voltage
    if((inputVoltageAverage < PVPANEL_UNDERVOLTAGE_LIMIT))
    {
        pvUnderVoltageCounter++;
        mpptFactor = mpptFactor - (mpptFactor >> 1);
        if(pvUnderVoltageCounter >= 10)
        {
            faultState = PV_PANEL_VOLTAGE;
            storeFaultState = (faultState << 1);        // x2 to account for "off" time
            systemState =SYSTEMERROR;

            pvPanelUnderVoltageFlag = 1;
            PV_Voltage_Tripflag =1;
            pvUnderVoltageCounter = 0;
        }
    }
    else if ((inputVoltageAverage > PVPANEL_UNDERVOLTAGE_LIMIT_HYS) && (pvPanelUnderVoltageFlag == 1))
    {
        if(faultState == PV_PANEL_VOLTAGE)
        {
            faultState = NO_FAULT;
            PV_Voltage_Tripcount =0;
            pvPanelUnderVoltageFlag = 0;
            PV_Voltage_Tripcount++;
        }
    }




// Grid Tie mode PV VOLTAGE TRIP ENDS/////////////
}

void overcurrenttrip(void)
{
    if((averageRectifiedCurrent >= INVERTER_OUTPUTCURRENT_MAX)  && (systemState == GRIDMODE)/*&& (start_current_fault_check == 1)*/) //&&  (acCurrentOffsetFlag == 1) && (systemState == GRIDMODE))//(systemState = GRIDMODE))
    {
        inverterOverCurrentCounter++;
        //if(inverterOverCurrentCounter >= 10)
        {
            if(faultState == NO_FAULT)
            {
                systemState =SYSTEMERROR;
                faultState = INVERTER_OVERCURRENT;
                storeFaultState = (faultState << 1);        // x2 to account for "off" time

                criticalFaultFlag = 1;

            }
        }
    }
    else
    {
    //  faultState = NO_FAULT;
        inverterOverCurrentCounter = 0;
    //  start_current_fault_check=0;
    }
}

void Drivesupply (void)
{
    //Check 12V Drive Supply Voltage
    if((driveSupplyVoltage > MAXDRIVEVOLTAGE) || (driveSupplyVoltage < MINDRIVEVOLTAGE))
    {
        driveSupplyFaultCnt++;

        if(driveSupplyFaultCnt > 10)
        {
            driveSupplyFaultCnt = 10;

            if(faultState == NO_FAULT)
            {

                faultState = DRIVE_SUPPLY;
                storeFaultState = (faultState << 1);        // x2 to account for "off" time

                systemState =SYSTEMERROR;

            }
        }
    }
    else
    {
        driveSupplyFaultCnt = 0;

        if(faultState == DRIVE_SUPPLY)
        {

            faultState = NO_FAULT;
           // Restart_count = 20000;  //2sec
        }
    }
}

void Temperaturefault (void)
{
    // Check Over Temperature Fault
    if((measuredTemperature >= MAXTEMPERATURE) /*&& (overTemperatureFlag == 0)*/)
    {
        tempFaultCnt++;
        mpptFactorMaximum = 18000;
        if(tempFaultCnt > 100)
        {
            tempFaultCnt = 105;

            if(faultState == NO_FAULT)
            {
                faultState = TEMPERATURE;
                storeFaultState = (faultState << 1);        // x2 to account for "off" time
                systemState =SYSTEMERROR;
                overTemperatureFlag=1;
            }
        }
    }
    else if ((measuredTemperature <= MAXTEMPERATURE_HYS) /*&& (overTemperatureFlag == 1)*/)
    {
       //overTemperatureFlag = 0;
        mpptFactorMaximum=24500;
        if(faultState == TEMPERATURE)
        {
            GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
            faultState = NO_FAULT;
            tempFaultCnt = 0;
            Restart_count = 200000;
            overTemperatureFlag=0;
        }
    }

}


void ac_cal(void)
{
    if ((acoffset_cal == 1) && (acoffset_cal2==0))
    {

        // Read AC Current offset during startup mode and verify data
        // Needs to be completed before the full-bridge is enabled during system startup

        AC_Voltage_Average = AC_Voltage_Average + (AdcResult.ADCRESULT7 << 3);
        AC_grid_voltage_average = AC_grid_voltage_average + (AdcResult.ADCRESULT7 << 3);

        if(acoffsetcounter >= 255)
        {
            acoffsetcounter = 0;
            acoffset_cal = 0;
            AC_inv_offset= (AC_Voltage_Average >> 8);
            AC_Voltage_Average = 0;

            AC_grid_offset = (AC_grid_voltage_average >> 8) ;
            AC_grid_voltage_average = 0;

        //  AC_grid_offset = 12700;
            AMC_cal = 1;

        }
        else
        {
            acoffsetcounter++;
        }

    }
}

void Grid_amcbias_cal(void)
{

    if ((AMC_cal == 1) && (Zcd1Flag == 1))
    {
        faltu_counter ++;
        AC_grid_voltage = (AdcResult.ADCRESULT7 << 3)- AC_grid_offset;
        if(AC_grid_voltage >= Max_Pos_grid_voltage)
        {
            Max_Pos_grid_voltage = AC_grid_voltage;
        }
        else if (AC_grid_voltage < Max_Neg_grid_voltage)
        {
            Max_Neg_grid_voltage = AC_grid_voltage;
        }

        if(faltu_counter == 1120)
        {
            faltu_counter = 0;
            ac_counter++;

            Peak_Pos_grid_voltage = Max_Pos_grid_voltage;
            Peak_Neg_grid_voltage = Max_Neg_grid_voltage;
            Max_Pos_grid_voltage = 0;
            Max_Neg_grid_voltage = 0;
        }


        if(ac_counter >= 10)
        {

            ac_counter = 0;
            PeakError= Peak_Pos_grid_voltage + Peak_Neg_grid_voltage;

            if (Zcd1Flag == 1)
            {
                AC_grid_offset = AC_grid_offset + (PeakError >> 1);
                //AC_grid_offset -= 128;
                AMC_cal = 0;
                PeakError=0;

            }

        }

    }


}




void Grid_over_under_voltage_trip(void)
{

    if(avgPeakGridVotlage < INVERTER_UNDERVOLTAGE_LIMIT_LEVEL)// && (gridUnderVoltageFlag == 0))
    {

        GridVOutofRangeFlag=1;
        faultState = INVERTER_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState = SYSTEMERROR;
        PWMSTOP();



    }
    else if(avgPeakGridVotlage > INVERTER_OVERVOLTAGE_LIMIT_LEVEL)
    {
        GridVOutofRangeFlag=1;
        faultState = INVERTER_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState = SYSTEMERROR;
        PWMSTOP();


    }

    else if((rectifiedGridVoltage >= INVERTER_OVERVOLTAGE_LIMIT) || (peakGridVoltage <= INVERTER_UNDERVOLTAGE_LIMIT))// && (gridUnderVoltageFlag == 0))
    {
        GridVOutofRangeFlag=1;
        faultState = INVERTER_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState = SYSTEMERROR;
        PWMSTOP();
        criticalFaultFlag = 1;


    }
    else if((avgPeakGridVotlage1 < INVERTER_OVERVOLTAGE_LIMIT_HYS) && (avgPeakGridVotlage1 > INVERTER_UNDERVOLTAGE_LIMIT_HYS))
    {
            if(GridVOutofRangeFlag == 1)
            {
                faultState = NO_FAULT;
                Restart_count = 200000;
                GridVOutofRangeFlag=0;
                offGridFlag=0;
            }

    }



}

void inverterFrequencyCheck(void)
{
    Trip_zeroCrossDetectFlag = 0;

    //if((frequancy_mains >= 5094) || (frequancy_mains <= 4906))
    if((ECap1Regs.CAP1>=1223755)||((ECap1Regs.CAP1>1156789)&&(ECap1Regs.CAP1<=1175765)))
    {
        PWMSTOP();
        frequencyFaultclearCounter = 0;
        if(faultState == NO_FAULT)
        {
            GridFOutofRangeFlag=1;
            faultState = INVERTER_FREQUENCY;
            storeFaultState = (faultState << 1);        // x2 to account for "off" time
            systemState =SYSTEMERROR;
        }
    }
    else if((ECap1Regs.CAP1<=1223755)&&(ECap1Regs.CAP1>=1175765)) //1175765//(faultState == INVERTER_FREQUENCY)
    {
        frequencyFaultclearCounter++;
        frequencyFaultCounter = 0;

        if((frequencyFaultclearCounter >= 50) && (GridFOutofRangeFlag ==1)) //500ms
        {
            GridFOutofRangeFlag=0;
            faultState = NO_FAULT;
            Restart_count = 200000; //20sec
            frequencyFaultclearCounter = 0;
        }

    }
    else
    {
        frequencyFaultCounter = 0;
        frequencyFaultclearCounter = 0;
    }

}



void OFFGrid_PVvoltagetrip(void)
{
    // Check PV Panel Voltage Using the Average Input Voltage
    //if((inputVoltageAverage > PVPANEL_OVERVOLTAGE_LIMIT) && (pvPanelOverVoltageFlag == 0))
    if(pv_Input_voltage_average > PVPANEL_OVERVOLTAGE_LIMIT)  //29150 = 50V
    {
        PV_Voltage_Tripflag = 1;
        faultState = PV_PANEL_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState =SYSTEMERROR;
        pvPanelOverVoltageFlag = 1;

    }
    else if((pv_Input_voltage_average < PVPANEL_OVERVOLTAGE_LIMIT_HYS) )   //27401 = 47V
    {
        if(faultState == PV_PANEL_VOLTAGE)
        {
            faultState = NO_FAULT;
            pvPanelOverVoltageFlag = 0;
            PV_Voltage_Tripcount =0;
            PV_Voltage_Tripcount++;

        }

    }


    // Check PV Panel Minimum Voltage Using Average Input Voltage
    if((pvPanelVoltage < PVPANEL_UNDERVOLTAGE_LIMIT))  //11096 = 19V
    {
        PV_Voltage_Tripflag = 1;
        faultState = PV_PANEL_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        systemState =SYSTEMERROR;
        pvPanelUnderVoltageFlag = 1;

    }


    else if ((pv_Input_voltage_average > PVPANEL_UNDERVOLTAGE_LIMIT_HYS))  //12264 = 21V
    {
        if(faultState == PV_PANEL_VOLTAGE)
        {
            faultState = NO_FAULT;
            pvPanelUnderVoltageFlag = 0;
            PV_Voltage_Tripcount =0;
            PV_Voltage_Tripcount ++;
        }
    }


}

void tempreg(void)
{
    if(measuredTemperature >= 13000)  //78
        {
            mpptFactorMaximum=22000;
        }
         else
        {
            mpptFactorMaximum=24500;
        }
}

void variablePI (void)
{
   // GpioDataRegs.GPASET.bit.GPIO11 = 1;
   //88uS
    if(VariablePIFlag == 1)
    {
                   if(flybacktron == 1)
                   {

                           Ra_float = -(0.00000000008 *mpptFactor*mpptFactor)+(0.000002*mpptFactor)+0.094;

                           Rsa_float = (0.00000000005 * mpptFactor * mpptFactor) - (0.000003 * mpptFactor) + 0.0542;

                   }
                   else
                   {
                       Ra_float = 0.3;
                       Rsa_float = 0.03;
                   }
                   Ra = _IQ15(Ra_float);
                   Rsa = _IQ15(Rsa_float);
                   VariablePIFlag=0;
   //88uS
    }
          //         GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

}
