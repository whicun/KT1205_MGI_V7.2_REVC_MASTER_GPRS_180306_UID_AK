#include "DSP28x_Project.h"
#include "Solar Microinverter_isr.h"
#include "Solar Microinverter_defines.h"
#include "IQmathLib.h"
#include "DSP2803x_Device.h"

// System Faults
int maxInverterOutputVoltage = 0, rectifiedInverterOutputVoltage = 0;
unsigned char criticalFaultFlag = 0;
// Flyback periods
int flyBackDuty1 = 0, flyBackDuty2 = 0;
// Compensator Variables
int rectifiedInverterOutputCurrent = 0, peakInverterOutputVoltage = 0;
long int Ioutput = 0;

unsigned char fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

// Inverter Current Reference Variables
unsigned char ninetyDegreeDetectFlag = 0;
unsigned int globalAngle = 0, sineAngle = 0, deltaAngle = 0;
unsigned int currentReferenceDynamic = 0;
unsigned int temp1, d;
// ADC Variables
unsigned int pvPanelVoltage = 0, flybackCurrent1 = 0, flybackCurrent2 = 0;
int inverterOutputVoltage = 0, inverterOutputCurrent = 0;
unsigned int driveSupplyVoltage = 0, measuredTemperature = 0;

// Flyback Current Moving Average Variables/Arrays
unsigned char currentArrayCnt = 0;
unsigned int averageFlybackCurrent1 = 0, averageFlybackCurrent2 = 0;
unsigned int flybackCurrent1Array[8] = {0,0,0,0,0,0,0,0};
unsigned int flybackCurrent2Array[8] = {0,0,0,0,0,0,0,0};
long unsigned int flybackCurrent1Sum = 0, flybackCurrent2Sum = 0;

// AC Current Variables and Moving Avg Variables/Array
unsigned int averageRectifiedCurrent = 0, maxInverterOutputCurrent = 0, averageRectifiedCurrentoff=0;
unsigned int rectifiedInverterOutputCurrentArray[8] = {0,0,0,0,0,0,0,0};
long unsigned int rectifiedInverterOutputCurrentSum = 0;

// Peak Power and Burst Mode Variables
unsigned int averagePeakOutputPower = 0, peakOutputPower = 0;
volatile int peakInverterOutputCurrent = 0;
unsigned char peakOutputPowerArrayCnt = 0;
unsigned int peakOutputPowerArray[8] = {0,0,0,0,0,0,0,0};
long unsigned int peakOutputPowerSum = 0;
unsigned char burstModeActiveFlag = 0;
unsigned int burstModeActiveCounter = 0;

// Zero Cross Variables
unsigned char zeroCrossDetectFlag = 0, zeroCrossCount = 0, firstQuadrantFlag = 0;
unsigned char thirdQuadrantFlag = 0, avgInputDataReadyFlag = 0;
unsigned char zcCounter = 0, startupZeroCrossCounter = 0;
unsigned char prevFullBridgeState = 0;
unsigned int inverterPeriod = 0, inverterPeriodCounter = 0, prevInverterPeriod = 0, numberofSamples = 0;

// MPPT Variables

long unsigned int inputCurrentSum = 0, inputVoltageSum = 0;
//mpptFactorMaximum = 23215,

// Externally Defined Variables
extern unsigned char switchState, systemState, faultState, zeroCrossDelay;
extern unsigned char inverterFrequencyState, criticalFaultRestartFlag, startFullBridgeFlag;
extern unsigned char zeroCrossDelayNom, zeroCrossDelayMin, zeroCrossDelayMax;
extern unsigned int  inputVoltageAverage, inputCurrentAverage;
extern unsigned int inverterPeriodMin, inverterPeriodMax, inverterOutputOverCurrent, acCurrentOffset;



// Variable to capture with delay

extern int count, max_count, i;
extern int array1[560];		// , array2[560];

// Off grid variables
int gridVoltage, rectifiedGridVoltage,  filteredPeakGridVoltage, filteredMaxGridVoltage, maxGridVoltage = 0, prevGridVoltage = 0, dtime_counter;
volatile int  peakGridVoltage;
unsigned int inverterPeriodCounter_off = 0, softstart_counter;
int sfst_acVoltage = 6000, setRef_acVoltage = 1000, offgridVoltage =6000;	//6150				// 220V AC refers to 8100 in Q15 (220*16384/445)

extern volatile unsigned char gridFlag;

volatile unsigned int Ra = 0, Rsa = 0;
volatile long temp_Ioutput = 800;

extern Uint16 AC_grid_offset;
extern Uint16 AC_inv_offset;

unsigned int flybackDutyCycle = 0;
unsigned int rectifiedVac = 0, decoupleTerm = 0;
int IoRef = 0, currentError = 0, Poutput = 0, VoRef = 0;
long int totalOutput = 0;
Uint32 temp_decouple = 27573; //29163
unsigned int tripcount = 0;
int previnputvoltage = 0,deltaV1 = 0;
extern volatile unsigned int mpptFactor;
unsigned char flybacktron = 0;
unsigned int flybackoncounter = 0;
extern unsigned char acCurrentOffsetFlag,storeFaultState;
unsigned char shortcktcount = 0;
unsigned char flybackshutdownflag=0;
unsigned int startupZeroCrossCounter1=0;
volatile unsigned int Trip_zeroCrossDetectFlag = 0, Trip_inverterPeriod = 0;
unsigned int avgPeakGridVotlage=0,avgPeakGridVotlage1=0 ,peakGridVoltageSum =0,voltageArrayCnt=0,voltageArrayCnt1=0;
unsigned int peakGridVoltageArray[5]={0,0,0,0,0};
unsigned int peakGridVoltageArray1[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long int peakGridVoltageSum1=0;
unsigned char twoAvgPeakGridVoltageflag=0;
unsigned int mpptFactorMaximum = 0;
unsigned int temop = 0;
unsigned int frequencyFaultCounter = 0;
unsigned char Zcd1Flag = 0,OffgridStartedflag =0;
Uint16 grid_failure = 0,grid_present_count = 0,grid_present = 0, peakgridvoltagereadycounter=0, peakgridvoltagereadyflag =0;
volatile unsigned char grid_present_flag = 0 , GraphPlotFlag =0,OffGridzeroCrossCount=0;

unsigned int OffGridStartupFlag =0, ZCStartupCounter=0,StatemachineCheckFlag=0,StatemachineFaultCounter=0;

long unsigned int my_Vrms_acc = 0, my_Irms_acc = 0,  my_Prms_acc = 0,my_Vrms_q15_Sum=0,my_Vrms_q15_AVG=0;
long unsigned int my_Vrms_q15 = 0,my_rms_count = 0,my_Irms_q15 = 0, my_Prms_q15 = 0,my_Irms_q15_result;


extern unsigned int GridVOutofRangeFlag,GridFOutofRangeFlag;

#pragma CODE_SECTION(adc_isr, "ramfuncs");



interrupt void  adc_isr(void)
{

    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    gridFlag = grid_present_flag;

    if(StatemachineCheckFlag ==0)
    {
        StatemachineCheckFlag=1;
        StatemachineFaultCounter=0;
        ServiceDog();

    }

    if ((gridFlag == 0) && (systemState == GRIDMODE))
    {
        PWMSTOP();
        faultState = INVERTER_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time
        criticalFaultFlag = 1;
        startFullBridgeFlag = 0;        // Reset flag to start full-bridge
        EALLOW;
        EPwm3Regs.AQCSFRC.bit.CSFA = 1;
        EPwm3Regs.AQCSFRC.bit.CSFB = 1;
        EDIS;

        // GpioDataRegs.GPASET.bit.GPIO24 = 1;
        //GpioDataRegs.GPBSET.bit.GPIO33 = 1;
        OPTO_DRV2 = 1;
        OPTO_DRV1 = 1;

        systemState = SYSTEMERROR;

    }

    if ((gridFlag == 1) && (systemState == OFFGRIDMODE))
    {

        PWMSTOP();
        faultState = INVERTER_VOLTAGE;
        storeFaultState = (faultState << 1);        // x2 to account for "off" time

        criticalFaultFlag = 1;

        startFullBridgeFlag = 0;        // Reset flag to start full-bridge
        EALLOW;
        EPwm3Regs.AQCSFRC.bit.CSFA = 1;
        EPwm3Regs.AQCSFRC.bit.CSFB = 1;
        EDIS;
        OPTO_DRV2 = 1;
        OPTO_DRV1 = 1;
        GridVOutofRangeFlag =1;
        GridFOutofRangeFlag =1;


        systemState = SYSTEMERROR;
    }




    //-----------------------Dual mode common functionalities loop starts here ------------------------------



    if (Comp3Regs.COMPSTS.bit.COMPSTS == 1||Comp2Regs.COMPSTS.bit.COMPSTS==1||Comp1Regs.COMPSTS.bit.COMPSTS == 1)
     {
         if(tripcount >= 5)
         {
          tripcount=0;
              criticalFaultFlag = 1;

          if (Comp2Regs.COMPSTS.bit.COMPSTS==1||Comp3Regs.COMPSTS.bit.COMPSTS == 1)
            {
                    faultState = FLYBACK_OVERCURRENT;
                    storeFaultState = (faultState << 1);        // x2 to account for "off" time
                    systemState = SYSTEMERROR;
                    PWMSTOP();

            }
          else if (Comp1Regs.COMPSTS.bit.COMPSTS == 1)
                {
                    faultState = FLYBACK_OUTPUT_VOLTAGE;
                    storeFaultState = (faultState << 1);        // x2 to account for "off" time
                    systemState = SYSTEMERROR;

                    PWMSTOP();

                }
             }
             tripcount++;
     }

     if(GpioDataRegs.GPADAT.bit.GPIO10 == 0)
        {
        grid_failure = 0;
        grid_present_count++;
        if(grid_present_count > 150)
        {
            grid_present++;
            if(grid_present > 50)
            {
                grid_present_flag = 1;
                grid_present=0;
                grid_present_count=0;
            }
        }
    }
        else
        {

            grid_failure++;
        }

        if(grid_failure > 1121)
        {
            grid_present_flag = 0;
            grid_present = 0;
            grid_present_count = 0;
        }



	    if(grid_present_flag == 1)
	    {
	        gridVoltage = (AdcResult.ADCRESULT7 << 3) - AC_grid_offset; //AC_grid_offset

	        if(gridVoltage >= 0)
	        {
	            rectifiedGridVoltage = gridVoltage;
	        }
	        else
	        {
	            rectifiedGridVoltage = (-gridVoltage);
	        }

	        if(rectifiedGridVoltage > maxGridVoltage)
	        {
	            maxGridVoltage = rectifiedGridVoltage;
	        }

	    }
	    else
		{
	    		gridVoltage=0;
		}






	// Read Flyback Current, PV Voltage, and 2.5V Reference in Q15 Format
	pvPanelVoltage  = (AdcResult.ADCRESULT0 << 3); 		// Read PV Panel Voltage
	measuredTemperature = (AdcResult.ADCRESULT1 << 3);

	flybackCurrent1 = (AdcResult.ADCRESULT4 << 3); 		// Read PV cell Current at Flyback leg1
    flybackCurrent2 = (AdcResult.ADCRESULT5 << 3); 		// Read PV cell Current at Flyback leg2
    // Read inverter output voltage and inverter output current (Q14)
    inverterOutputCurrent = (AdcResult.ADCRESULT6 << 3) - acCurrentOffset;//
    driveSupplyVoltage = (AdcResult.ADCRESULT8 << 3);

    inverterOutputVoltage = (AdcResult.ADCRESULT7 << 3) - AC_inv_offset;


    inputVoltageSum = inputVoltageSum + pvPanelVoltage;



	//-----------------------Dual mode common functionalities loop starts here ------------------------------



//?


	// Find Peak Inverter Output Voltage
	if(rectifiedInverterOutputVoltage > maxInverterOutputVoltage)
	{
		maxInverterOutputVoltage = rectifiedInverterOutputVoltage;
	}



	// Rectify AC current and check for over current condition on the ouput
	if(inverterOutputCurrent >= 0)
	{
		rectifiedInverterOutputCurrent = inverterOutputCurrent;
	}
	else
	{
		rectifiedInverterOutputCurrent = (-inverterOutputCurrent);
	}

	// Find Peak Inverter Output Current
	if(rectifiedInverterOutputCurrent > maxInverterOutputCurrent)
	{
		maxInverterOutputCurrent = rectifiedInverterOutputCurrent;
	}

//
//	// Moving Average of Flyback Currents for load sharing
//	flybackCurrent1Sum = flybackCurrent1Sum + flybackCurrent1 - flybackCurrent1Array[currentArrayCnt];
//	averageFlybackCurrent1 = flybackCurrent1Sum >> 3;
//
//	flybackCurrent2Sum = flybackCurrent2Sum + flybackCurrent2 - flybackCurrent2Array[currentArrayCnt];
//	averageFlybackCurrent2 = flybackCurrent2Sum >> 3;
//
//	flybackCurrent1Array[currentArrayCnt] = flybackCurrent1;
//	flybackCurrent2Array[currentArrayCnt] = flybackCurrent2;
//
	//-----------------------Dual mode common functionalities loop starts here ------------------------------
	switch (systemState)
	{
		case SYSTEMSTARTUP:
		{
			GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // off grid Zero cross sycn signal
			inverterPeriodCounter_off = 0;


			if(gridFlag == 1)
			{

				startupZeroCrossCounter1 = 0;
				if((zeroCrossDetectFlag == 0) && (inverterPeriodCounter > 200))
				{
					//zeroCrossDetection();


					// Detect the zero crossing at the 1st Quadrant
					if((prevGridVoltage < 0) && (gridVoltage >= 0))
					{
					//	GpioDataRegs.GPASET.bit.GPIO10 = 1;
						// As a precaution change state of Full-Bridge if it hasn't already chaged
						fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

						//	ClrWdt();

						zeroCrossDetectFlag = 1;
						firstQuadrantFlag = 1;				// Allows Full-Bridge to change State
						// Load counter to Grid Period to check grid frequency
						inverterPeriod = inverterPeriodCounter;
						inverterPeriodCounter = 0;

						// Store accumulated voltage/current sum and counter for avg (avg calculated in T2ISR)
						// The average is calculated every 3rd +ve zero cross event

						zeroCrossCount++;

						// inverterPeriod is only half cycle so add prevInverterPeriod
						numberofSamples = numberofSamples + inverterPeriod + prevInverterPeriod;

						if(zeroCrossCount >= 2)
						{
							zeroCrossCount = 0;
							Zcd1Flag = 1;
							avgInputDataReadyFlag = 1;

							inputVoltageAverage = (unsigned int) (inputVoltageSum/ numberofSamples);
							inputCurrentAverage = (unsigned int) (inputCurrentSum / numberofSamples);
							inputVoltageSum = 0;
							inputCurrentSum = 0;
							numberofSamples = 0;


						}
						else
						{
							Zcd1Flag = 0;
						}
					}

					// Detect the zero crossing at the 3rd Quadrant
					else if ((prevGridVoltage >= 0) && (gridVoltage < 0))
					{
					//	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
						// As a precaution change state of Full-Bridge if it hasn't already chaged
						fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

					//	ClrWdt();
						zeroCrossDetectFlag = 1;
						thirdQuadrantFlag = 1;			// Allows Full-Bridge to change State
						// Load counter to Grid Period to check grid frequency
						inverterPeriod = inverterPeriodCounter;
						inverterPeriodCounter = 0;
					}


				}

				if(zeroCrossDetectFlag == 1)
				{
					startupZeroCrossCounter++;
				}


				// Store current grid voltage to compare with next sample for ZCD
				prevGridVoltage = gridVoltage;
				//------------------------------------------------------


				inputCurrentSum = inputCurrentSum + flybackCurrent1 ;



				//************ninetyDegreeDetect,sineAngle,globalAngle calculation loops starts here***************

				// This portion of SW determines the current reference from the sine lookup table based on
				// deltaAngle which is calculated every Zero Cross
				// sineAngle is used to point to specific location in sine lookup table
				if(ninetyDegreeDetectFlag == 0)
				{
					// if sineAngle is less than 90 degrees, then add deltaAngle to point to next sample
					if (sineAngle < NINETYDEGREE)
					{
						currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
						sineAngle = sineAngle + deltaAngle;
					}
					else
					{
						sineAngle = NINETYDEGREE;
						ninetyDegreeDetectFlag = 1;
					}
				}
				else
				{
					// if sineAngle is greater than 90 degrees, then subtract deltaAngle from it
					currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
					sineAngle = sineAngle - deltaAngle;
				}


			}
/*			else if (gridFlag ==0)
			{

				}*/


		}
		break;


//--------------------SYSTEMSTARTUP ends here---------------------
		case SYSTEMERROR:
		{


		    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // off grid Zero cross sycn signal
			EALLOW;
            Comp2Regs.DACVAL.bit.DACVAL = 250;  //190       // Consider current at ~20V
            Comp3Regs.DACVAL.bit.DACVAL = 250;
            EDIS;


			PWMSTOP();
			inverterPeriodCounter_off = 0;

			if(gridFlag==1)
			{

				startupZeroCrossCounter1 = 0;
				startupZeroCrossCounter = 0;
				//----------------GRID MODE related starts here--------------------------
				if((zeroCrossDetectFlag == 0) && (inverterPeriodCounter > 200))
				{
					//zeroCrossDetection();


					// Detect the zero crossing at the 1st Quadrant
					if((prevGridVoltage < 0) && (gridVoltage >= 0))
					{
					//	GpioDataRegs.GPASET.bit.GPIO10 = 1;
						// As a precaution change state of Full-Bridge if it hasn't already chaged
						fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

						//	ClrWdt();

						zeroCrossDetectFlag = 1;
						firstQuadrantFlag = 1;				// Allows Full-Bridge to change State
						// Load counter to Grid Period to check grid frequency
						inverterPeriod = inverterPeriodCounter;
						inverterPeriodCounter = 0;

						// Store accumulated voltage/current sum and counter for avg (avg calculated in T2ISR)
						// The average is calculated every 3rd +ve zero cross event

						zeroCrossCount++;

						// inverterPeriod is only half cycle so add prevInverterPeriod
						numberofSamples = numberofSamples + inverterPeriod + prevInverterPeriod;

						if(zeroCrossCount >= 2)
						{
							zeroCrossCount = 0;
							Zcd1Flag = 1;
							avgInputDataReadyFlag = 1;

							inputVoltageAverage = (unsigned int) (inputVoltageSum/ numberofSamples);
							inputCurrentAverage = (unsigned int) (inputCurrentSum / numberofSamples);
							inputVoltageSum = 0;
							inputCurrentSum = 0;
							numberofSamples = 0;

						}
						else
						{
							Zcd1Flag = 0;
						}
					}

					// Detect the zero crossing at the 3rd Quadrant
					else if ((prevGridVoltage >= 0) && (gridVoltage < 0))
					{
					//	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
						// As a precaution change state of Full-Bridge if it hasn't already chaged
						fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

					//	ClrWdt();
						zeroCrossDetectFlag = 1;
						thirdQuadrantFlag = 1;			// Allows Full-Bridge to change State
						// Load counter to Grid Period to check grid frequency
						inverterPeriod = inverterPeriodCounter;
						inverterPeriodCounter = 0;
					}


				}
				// Store current grid voltage to compare with next sample for ZCD
				prevGridVoltage = gridVoltage;
				//------------------------------------------------------

				inputCurrentSum = inputCurrentSum + flybackCurrent1 ;

			}
			else if(gridFlag ==0)
			{
				setRef_acVoltage =1000;
				OffGridStartupFlag =0;
				ZCStartupCounter=0;
	            peakInverterOutputVoltage=0;
	            averageRectifiedCurrentoff=0;

			}
		}
		break;

//--------------------SYSTEMERROR ends here---------------------



		case GRIDMODE:
		{



	        startupZeroCrossCounter = 0;
	        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // off grid Zero cross sycn signal

            //---------- SINGLE OR TWO TRANSFORMER SELECTION LOOP STARTS HERE------------

	        EALLOW;
           Comp2Regs.DACVAL.bit.DACVAL = 280;  //171       // Consider current at ~29V
           Comp3Regs.DACVAL.bit.DACVAL = 280;
           Comp1Regs.DACVAL.bit.DACVAL = 650;
           EDIS;

            if(flybacktron == 1)
            {
                inputCurrentSum = inputCurrentSum + flybackCurrent1 + flybackCurrent2;
            }
            else
            {
                inputCurrentSum = inputCurrentSum + flybackCurrent1;
            }



            //---------- SINGLE OR TWO TRANSFORMER SELECTION LOOP STARTS HERE------------


            //*****************SHORT CIRCUTI TRIP LOOP STARTS HERE ***************************

            if(((inverterOutputCurrent >= 16100) || (inverterOutputCurrent <= -16100)))
            {
                shortcktcount++;
                if(shortcktcount >= 80)
                {
      			  criticalFaultFlag = 1;

                    PWMSTOP();
                    faultState = SHORT_CIRCUIT;
                    storeFaultState = (faultState << 1);        // x2 to account for "off" time

                    systemState = SYSTEMERROR;

                    shortcktcount = 0;
                }
            }

            //*****************SHORT CIRCUTI TRIP LOOP ENDS HERE ***************************



            //--------------------------------------------------------------


            if (inputVoltageAverage < 12800 )  // if input voltage is less than 21V mpptfactor is reduced by quater
            {
                mpptFactor = mpptFactor - (mpptFactor >> 2);
            }

            // globalAngle is used for detecting grid voltage states
            globalAngle = globalAngle + deltaAngle;




            //--------------------------------------------------------------


            //_______________________ FLYBACK CONTROL FUNCTION CALLING LOOP STARTS HERE ______________________

            if((fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE) || (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE))
            {
                // If in Burst Mode, only deliver power for one complete AC cycle
                // If not in Burst Mode, remove the PWM override when full-bridge state is active

                EALLOW;
                EPwm1Regs.AQCSFRC.bit.CSFA = 0x3;
                EPwm2Regs.AQCSFRC.bit.CSFA = 0x3;
                EDIS;



                if(flybackshutdownflag == 0)
                {

                // Compensator Software
                IoRef = (int)(((long)currentReferenceDynamic * (long)mpptFactor) >> 15);  //
                temop = IoRef;
                currentError = IoRef - (rectifiedInverterOutputCurrent << 1);
                Poutput = (int)(((long)currentError * Ra) >> 15);
                Ioutput  =  Ioutput + (long)(((long)currentError * Rsa) >> 15);
                if(Ioutput > 32767)
                {
                    Ioutput = 32767;
                }
                else if(Ioutput < -32767)
                {
                    Ioutput = -32767;
                }

                totalOutput = (long)Poutput + Ioutput;

                // Input/Output Voltage Decoupling Software (Vo/(Vin + Vo))
                // rectifiedVac is in Q13 format
                rectifiedVac = (unsigned int)(((long)currentReferenceDynamic * (long)peakGridVoltage) >> 16);
                // pvPanel Voltage is multiplied by Q15(0.89) to have the same per unit as the rectifiedVac
                // and both are in Q13 format (56.1 * turns ratio = 392.7, 392/445 = 0.89)
                decoupleTerm = rectifiedVac + (unsigned int)(((long)temp_decouple * (long)pvPanelVoltage) >> 17);
                // Divide Q26/Q13, result is in Q13
                decoupleTerm = (unsigned int)((((long)rectifiedVac) << 13) / decoupleTerm);
                // Total Compensator Output, decoupleTerm is scaled back to Q15 as totalOutput is clamped to Q15
                totalOutput = totalOutput + (long)(decoupleTerm << 2);


                if (mpptFactor <= 250)  // 500
                {
                    totalOutput = totalOutput >> 2 ;
                }


                // Clamp total output to the maximum Duty Cycle
                if(totalOutput > MAXDUTYCLAMPED)
                {
                    totalOutput = MAXDUTYCLAMPED;
                }
                else if(totalOutput < 0)
                {
                    totalOutput = 0;
                }
                // Multiply flyback period by total output to get the flyback duty cycle
                flybackDutyCycle = (int)(((long)totalOutput * FLYBACKPERIOD) >> 15);
                // If duty cycle is less than dead-time make duty cycle equal to dead-time
                if(flybackDutyCycle < FE_DELAY)
                {
                    flyBackDuty1 = 0;
                    flyBackDuty2 = 0;
                }
                else
                {
                    if(flybacktron == 1)
                    {
                        flyBackDuty1 = flybackDutyCycle ;// + deltaDutyCycle ;
                        flyBackDuty2 = flybackDutyCycle ;// - deltaDutyCycle ;
                        /*flyBackDuty1 = 0;
                        flyBackDuty2 = 0;*/
                    }
                    else //if(mpptFactor > 7168)
                    {
                        flyBackDuty1 = flybackDutyCycle;// + deltaDutyCycle ;
                        flyBackDuty2 = 0;// - deltaDutyCycle ;
                    }
                }

                }
                else if (flybackshutdownflag == 1)
                {
                flyBackDuty1 = 0;//flybackDutyCycle;// + deltaDutyCycle ;
                flyBackDuty2 = 0;//flybackDutyCycle;// - deltaDutyCycle ;
                /*EPwm1Regs.TZFRC.bit.OST = 1;
                EPwm2Regs.TZFRC.bit.OST = 1;*/
                }




            }
            else
            {
                EALLOW;
                EPwm1Regs.AQCSFRC.bit.CSFA = 1;
                EPwm2Regs.AQCSFRC.bit.CSFA = 1;
                EDIS;
                // Give some history to the compensator before starting again at the zero cross
                if(inverterPeriodMin == INVERTERPERIOD50HZMIN)
                {
                    Ioutput = temp_Ioutput; //Ioutput = -800;  //-1900
                }
            }
            //_______________________ FLYBACK CONTROL FUNCTION CALLING LOOP ENDS HERE ______________________



            //-------________________ FULL BRIDGE FUNCTION CALLING LOOP STARTS HERE -----_________________
            if ((startFullBridgeFlag == 1) && (faultState == NO_FAULT))
            {
                // Routine that determines the state of the Full-Bridge MOSFETs
                fullBridgeDrive();

            }
            //-------________________ FULL BRIDGE FUNCTION CALLING LOOP ENDS HERE -----_________________


            //************ninetyDegreeDetect,sineAngle,globalAngle calculation loops starts here***************

            // This portion of SW determines the current reference from the sine lookup table based on
            // deltaAngle which is calculated every Zero Cross
            // sineAngle is used to point to specific location in sine lookup table
            if(ninetyDegreeDetectFlag == 0)
            {
                // if sineAngle is less than 90 degrees, then add deltaAngle to point to next sample
                if (sineAngle < NINETYDEGREE)
                {
                    currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
                    sineAngle = sineAngle + deltaAngle;
                }
                else
                {
                    sineAngle = NINETYDEGREE;
                    ninetyDegreeDetectFlag = 1;
                }
            }
            else
            {
                // if sineAngle is greater than 90 degrees, then subtract deltaAngle from it
                currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
                sineAngle = sineAngle - deltaAngle;
            }


            //************  ninetyDegreeDetect,sineAngle,globalAngle calculation loops ends here  ***************


            //+++++++++++++ Zero cross (180/360) function calling loop starts here ++++++++++++++++++++


            if((zeroCrossDetectFlag == 0) && (inverterPeriodCounter > 200))
            {
                //zeroCrossDetection();


                // Detect the zero crossing at the 1st Quadrant
                if((prevGridVoltage < 0) && (gridVoltage >= 0))
                {
                //  GpioDataRegs.GPASET.bit.GPIO10 = 1;
                    // As a precaution change state of Full-Bridge if it hasn't already chaged
                    fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

                    //  ClrWdt();

                    zeroCrossDetectFlag = 1;
                    firstQuadrantFlag = 1;              // Allows Full-Bridge to change State
                    // Load counter to Grid Period to check grid frequency
                    inverterPeriod = inverterPeriodCounter - 4 ;
                    inverterPeriodCounter = 0;

                    // Store accumulated voltage/current sum and counter for avg (avg calculated in T2ISR)
                    // The average is calculated every 3rd +ve zero cross event

                    zeroCrossCount++;

                    // inverterPeriod is only half cycle so add prevInverterPeriod
                    numberofSamples = numberofSamples + inverterPeriod + prevInverterPeriod;

                    if(zeroCrossCount >= 2)
                    {

                        zeroCrossCount = 0;
                        Zcd1Flag = 1;
                        avgInputDataReadyFlag = 1;

                        inputVoltageAverage = (unsigned int) (inputVoltageSum/ numberofSamples);
                        inputCurrentAverage = (unsigned int) (inputCurrentSum / numberofSamples);
                        inputVoltageSum = 0;
                        inputCurrentSum = 0;
                        numberofSamples = 0;


                        deltaV1 = inputVoltageAverage - previnputvoltage;

                        if(deltaV1 <= -1000)  //142
                        {
                            mpptFactor = mpptFactor - (mpptFactor >> 2);
                        }


                        else if(deltaV1 <= -577)  //142
                        {
                            mpptFactor = mpptFactor - (mpptFactor >> 3);
                        }

                        previnputvoltage = inputVoltageAverage;

                    }
                    else
                    {
                        Zcd1Flag = 0;
                    }
                }

                // Detect the zero crossing at the 3rd Quadrant
                else if ((prevGridVoltage >= 0) && (gridVoltage < 0))
                {
                //  GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
                    // As a precaution change state of Full-Bridge if it hasn't already chaged
                    fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

                //  ClrWdt();
                    zeroCrossDetectFlag = 1;
                    thirdQuadrantFlag = 1;          // Allows Full-Bridge to change State
                    // Load counter to Grid Period to check grid frequency
                    inverterPeriod = inverterPeriodCounter -4;
                    inverterPeriodCounter = 0;
                }


            }
            // Store current grid voltage to compare with next sample for ZCD
            prevGridVoltage = gridVoltage;


            //+++++++++++++ Variables update and reinitialization at every Zero cross (180/360) ++++++++++++++++++++

            if(zeroCrossDetectFlag == 1)
            {
                startupZeroCrossCounter1++;


                //Change ZeroCrossDelay based on peak AC Voltage
                if(peakGridVoltage > 13800)
                {
                    zeroCrossDelay = zeroCrossDelayMax;
                }

                else
                {
                    zeroCrossDelay = zeroCrossDelayNom;
                }



                if (inputVoltageAverage < 16350)
                {
                    mpptFactorMaximum = 21875; //I/P power Limited to 250W to maintain I/P Current Limit 10A
                }
                else
                {
                    if (avgPeakGridVotlage1 >= 6300)
                    {
                        mpptFactorMaximum = MPPTFACTORMAXIMUM; //I/P Power Max Limit 280W O/P 250W
                    }
                    else
                    {
                        mpptFactorMaximum= (avgPeakGridVotlage1 * 3) + DERATINGFACTOR;
                    }
                }
                //  peakOutputPower = _IQ14mpy(peakInverterOutputVoltage, peakInverterOutputCurrent);

                if(mpptFactor > 7272)
                {
                    flybackoncounter++;
                    if(flybackoncounter >= 28)
                    {
                        flybackoncounter = 0;
                        flybacktron = 1;
                    }
                }
                else
                {
                    flybackoncounter = 0;
                    flybacktron = 0;
                }




            }



		}
		break;

//--------------------GRIDMODE ends here---------------------
		case OFFGRIDMODE:
		{
            averageRectifiedCurrentoff=averageRectifiedCurrent;

            EALLOW;
            Comp2Regs.DACVAL.bit.DACVAL = 280;  //280       // Consider current at ~29V
            Comp3Regs.DACVAL.bit.DACVAL = 280;
            Comp1Regs.DACVAL.bit.DACVAL = 650;
            EDIS;


			if((inverterPeriodCounter_off ==1120) || (inverterPeriodCounter_off == 0)|| (inverterPeriodCounter_off == 1))
			{
				ZCStartupCounter++;
				if(ZCStartupCounter > 4)
				{
	                GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;  //clear PWMshutdown

					OffGridStartupFlag =1;
					ZCStartupCounter=7;

				}
				GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

			}
			else if (inverterPeriodCounter_off >= 2)
			{
				GpioDataRegs.GPASET.bit.GPIO22 = 1;
			}

			startupZeroCrossCounter1 = 0;
			startupZeroCrossCounter = 0;
			//-------________________ FULL BRIDGE FUNCTION CALLING LOOP STARTS HERE -----_________________
			if ((startFullBridgeFlag == 1) && (faultState == NO_FAULT))
			{
				// Routine that determines the state of the Full-Bridge MOSFETs
				fullBridgeDrive_off();

			}
			//-------________________ FULL BRIDGE FUNCTION CALLING LOOP ENDS HERE -----_________________

			//_______________________ FLYBACK CONTROL FUNCTION CALLING LOOP STARTS HERE ______________________





			if(inverterPeriodCounter_off == 560)
			{
//                if((pvPanelVoltage>=15300)&&(pvPanelVoltage<=16300))
//                    {
//                        offgridVoltage=6600;
//                    }
//                else if ((pvPanelVoltage>16300)&&(pvPanelVoltage<=18000))
//                    {
//                        offgridVoltage=6400;
//                    }
//                else if ((pvPanelVoltage>18000)&&(averageRectifiedCurrent>=10000))
//                    {
//                        offgridVoltage=6350;
//                    }
//                else if ((pvPanelVoltage>18000)&&(averageRectifiedCurrent<=3000))
//                {
//                    offgridVoltage=6150;
//                }

                if(OffgridStartedflag==1)
                {
                    if(my_Vrms_q15_AVG > 5071)
                    {
                       offgridVoltage -=50;
                    }
                    else if (my_Vrms_q15_AVG < 4340)  //4038 - 195
                    {
                       offgridVoltage +=50;

                    }


                    if((inputCurrentAverage < 2000))
                    {
                        offgridVoltage=5500;

                    }

                }





			  //  offgridVoltage = 6150 = 220

                if(setRef_acVoltage < offgridVoltage)   //7000-> {5500 = 50W load = 178V= 3650 (Vrms);  5650 = 50W load = 183V= 3740 (Vrms);    7600 = 50W load = 245V= 5010 (Vrms);    7750 = 50W load = 250V= 5100 (Vrms);                    //6600 = 10W load = 224.2V
                {
                    setRef_acVoltage = setRef_acVoltage + 50;  //200
                    OffgridStartedflag = 0;
                }
                else
                {

                        setRef_acVoltage = offgridVoltage; //OffgridInverterVoltage = 7000 = 230V @ 10W
                        OffgridStartedflag = 1;
                }

            }



            if(OffgridStartedflag==1) //2s for soft start timing
            {
                if((((averageRectifiedCurrentoff >= 15000) )) &&  (peakInverterOutputVoltage >5500))  //14400 - 1.17A
                {
                   shortcktcount++;
                   if(shortcktcount >= 80)
                   {
                       criticalFaultFlag = 1;
                       PWMSTOP();

                       faultState = INVERTER_OVERCURRENT;
                       storeFaultState = (faultState << 1);        // x2 to account for "off" time
                       systemState = SYSTEMERROR;
                       shortcktcount = 0;
                   }
                }

            }






		    // Rectified Inverter Output Voltage
		    if(inverterOutputVoltage >= 0)
		    {
		        rectifiedInverterOutputVoltage = inverterOutputVoltage;
		    }
		    else
		    {
		        rectifiedInverterOutputVoltage = (-inverterOutputVoltage);
		    }

		    // Find Peak Inverter Output Voltage
		    if(rectifiedInverterOutputVoltage > maxInverterOutputVoltage)
		    {
		        maxInverterOutputVoltage = rectifiedInverterOutputVoltage;
		    }


            inputCurrentSum = inputCurrentSum + flybackCurrent1 + flybackCurrent2;


            my_Vrms_acc = my_Vrms_acc +   (_IQ15rsmpy(rectifiedInverterOutputVoltage,rectifiedInverterOutputVoltage));//(((long)rectifiedInverterOutputVoltage * (long)rectifiedInverterOutputVoltage)>>32));
            my_Irms_acc = my_Irms_acc + (_IQ15rsmpy(rectifiedInverterOutputCurrent,rectifiedInverterOutputCurrent));//(((long)rectifiedInverterOutputVoltage * (long)rectifiedInverterOutputVoltage)>>32));
            my_rms_count++;

			if ((fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE) || (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE))
			{
					EALLOW;
					EPwm1Regs.AQCSFRC.bit.CSFA = 0x3;
					EPwm2Regs.AQCSFRC.bit.CSFA = 0x3;
					EDIS;



					rectifiedVac = (unsigned int)(((long)currentReferenceDynamic * (long)setRef_acVoltage) >> 16);


					decoupleTerm = rectifiedVac + (unsigned int)(((long)temp_decouple * (long)pvPanelVoltage) >> 17);

					decoupleTerm = (unsigned int)((((long)rectifiedVac) << 13) / decoupleTerm);

					totalOutput =(long)(decoupleTerm << 2);

					// Clamp total output to the maximum Duty Cycle
					if(totalOutput > MAXDUTYCLAMPED)
					{
						totalOutput = MAXDUTYCLAMPED;
					}
					else if(totalOutput < 0)
					{
						totalOutput = 0;
					}

					// Multiply flyback period by total output to get the flyback duty cycle
					flybackDutyCycle = (((long)totalOutput * FLYBACKPERIOD) >> 15);
					if(flybackDutyCycle < FE_DELAY)
					{
						flyBackDuty1 = 0;
						flyBackDuty2 = 0;
					}
					else
					{
						flyBackDuty1 = flybackDutyCycle; // + deltaDutyCycle ;
						flyBackDuty2 = flybackDutyCycle; // - deltaDutyCycle ;
					}




			}
					else
					{
						EALLOW;
						EPwm1Regs.AQCSFRC.bit.CSFA = 1;
						EPwm2Regs.AQCSFRC.bit.CSFA = 1;
						EDIS;
						// Give some history to the compensator before starting again at the zero cross
						Ioutput = -500;//-1000;
					}
			//_______________________ FLYBACK CONTROL FUNCTION CALLING LOOP ENDS HERE ______________________


			//+++++++++++++ Zero cross (180/360) function calling loop starts here ++++++++++++++++++++

			if(inverterPeriodCounter > 200)
			{
				//zeroCrossDetection_off();
				if(inverterPeriodCounter_off == 1120)
				{
					// As a precaution change state of Full-Bridge if it hasn't already chaged
					fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

					zeroCrossDetectFlag = 1;
					firstQuadrantFlag = 1;				// Allows Full-Bridge to change State
					// Load counter to Grid Period to check grid frequency
					inverterPeriod = inverterPeriodCounter;
					inverterPeriodCounter = 0;
					inverterPeriodCounter_off = 0;
					sineAngle = 0;
					// Store accumulated voltage/current sum and counter for avg (avg calculated in T2ISR)
					// The average is calculated every 3rd +ve zero cross event
					zeroCrossCount++;
					if (zeroCrossCount == 1)
					{
						numberofSamples = 0;
					}

					// inverterPeriod is only half cycle so add prevInverterPeriod
					numberofSamples = numberofSamples + inverterPeriod + prevInverterPeriod;

					if(zeroCrossCount >= 2)
					{
						zeroCrossCount = 0;

						inputVoltageAverage = (unsigned int)(inputVoltageSum / numberofSamples);
						inputCurrentAverage = (unsigned int)(inputCurrentSum / numberofSamples);
						numberofSamples = 0;
						inputVoltageSum = 0;
						inputCurrentSum = 0;
						avgInputDataReadyFlag = 1;
					}
				}

				// Detect the zero crossing at the 3rd Quadrant
				else if(inverterPeriodCounter_off == 560)
				{
					// As a precaution change state of Full-Bridge if it hasn't already chaged
					fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

					// ClrWdt();
					zeroCrossDetectFlag = 1;
					thirdQuadrantFlag = 1;			// Allows Full-Bridge to change State
					// Load counter to Grid Period to check grid frequency
					inverterPeriod = inverterPeriodCounter;
					inverterPeriodCounter = 0;
					sineAngle = 0;
				}
			}





			if(zeroCrossDetectFlag ==1)
			{
			    OffGridzeroCrossCount++;

                prevInverterPeriod = inverterPeriod;            // Store Inverter Period for average calculation added

			    peakInverterOutputVoltage = maxInverterOutputVoltage;
			    maxInverterOutputVoltage = 0;
		        my_Vrms_q15 = _IQ15mpy(_IQ15sqrt(my_Vrms_acc),_IQ15isqrt(my_rms_count<<15));//((_IQ15sqrt((_IQ15div(my_Vrms_acc,my_Vrms_count))))>>15);
		        my_Irms_q15 = _IQ15rmpy(_IQ15sqrt(my_Irms_acc),_IQ15isqrt(my_rms_count<<15));//((_IQ15sqrt((_IQ15div(my_Vrms_acc,my_Vrms_count))))>>15);
		        my_Irms_q15_result = my_Irms_q15;

		        my_Vrms_q15_Sum=my_Vrms_q15_Sum + my_Vrms_q15;

		        my_rms_count = 0;
		        my_Vrms_acc = 0;
		        my_Irms_acc = 0;

		        if(OffGridzeroCrossCount >2)
                {
                    my_Vrms_q15_AVG= my_Vrms_q15_Sum/OffGridzeroCrossCount;
                    my_Vrms_q15_Sum=0;
                    OffGridzeroCrossCount=0;
                }


		        //inverterPeriod = 560;  //fixed for halfcycle

		    }

			//+++++++++++++ Zero cross (180/360) function calling loop ends here ++++++++++++++++++++



			//************  ninetyDegreeDetect,sineAngle,globalAngle calculation loops starts here  ***************


			// This portion of SW determines the current reference from the sine lookup table based on
			// deltaAngle which is calculated every Zero Cross
			// sineAngle is used to point to specific location in sine lookup table
			if(ninetyDegreeDetectFlag == 0)
			{

			    // if sineAngle is less than 90 degrees, then add deltaAngle to point to next sample
				if (inverterPeriodCounter < 280)
				{
					currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
					sineAngle = sineAngle + deltaAngle;
				}
				else
				{
					sineAngle = NINETYDEGREE;
					ninetyDegreeDetectFlag = 1;
				}
			}
			else
			{
			    // if sineAngle is greater than 90 degrees, then subtract deltaAngle from it
				currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
				sineAngle = sineAngle - deltaAngle;

			}

			//************  ninetyDegreeDetect,sineAngle,globalAngle calculation loops ends here  ***************


		}
		break;
//--------------------OFFGRIDMODE ends here---------------------


	}
//--------------------switch loop ends here---------------------



	// Counter for verifying the Grid Frequency (number of ADC interrupts per grid half cycle)
	// variable gets reset when finding the zero crossing event
	inverterPeriodCounter++;
	inverterPeriodCounter_off++;

	// ZeroCrossDetectFlag is set inside the zeroCrossDetection Routine
	if(zeroCrossDetectFlag == 1)
	{
		zeroCrossDetectFlag = 0;
		//startupZeroCrossCounter++;
		startupZeroCrossCounter1++;
		Trip_zeroCrossDetectFlag = 1;

		Trip_inverterPeriod = inverterPeriod;
		prevInverterPeriod = inverterPeriod;


      // deltaAngle calculated based on present grid frequency/period measured
		deltaAngle = (unsigned int) (32767 / inverterPeriod);

		// Load the newly found Peak Voltage/Current of the Inverter Output (done @ zero crossing)
		//peakInverterOutputVoltage = maxInverterOutputVoltage;
		peakGridVoltage = maxGridVoltage;
		peakInverterOutputCurrent = maxInverterOutputCurrent;

		maxInverterOutputCurrent = 0;
		maxGridVoltage = 0;

		// Moving Average of AC Current for AC Current Fault
		rectifiedInverterOutputCurrentSum = rectifiedInverterOutputCurrentSum + peakInverterOutputCurrent - rectifiedInverterOutputCurrentArray[currentArrayCnt];
		averageRectifiedCurrent = rectifiedInverterOutputCurrentSum >> 3;

		rectifiedInverterOutputCurrentArray[currentArrayCnt++] = peakInverterOutputCurrent;

		if(currentArrayCnt >= 8)
		{
			currentArrayCnt = 0;
		}

		//avgPeakGridVotlage is averaged value of 2peak sampling for less than 110V or Greater than 315V


		if(voltageArrayCnt < 2)
		{
			peakGridVoltageSum = peakGridVoltageSum + peakGridVoltage - peakGridVoltageArray[voltageArrayCnt];
			peakGridVoltageArray[voltageArrayCnt++] = peakGridVoltage;
			twoAvgPeakGridVoltageflag =0;
		}
		else
		{
			avgPeakGridVotlage = peakGridVoltageSum >> 1;
			voltageArrayCnt = 0;
			twoAvgPeakGridVoltageflag =1;
		}

		if(twoAvgPeakGridVoltageflag == 1)
		{
			avgPeakGridVotlage1 = peakGridVoltageSum1 >> 4;
			peakGridVoltageSum1 = peakGridVoltageSum1 + avgPeakGridVotlage - peakGridVoltageArray1[voltageArrayCnt1];
			peakGridVoltageArray1[voltageArrayCnt1++] = avgPeakGridVotlage;

			if(voltageArrayCnt1 >= 16)
			{
				voltageArrayCnt1 = 0;
			}
		}


		// Reset Variables every Zero Cross
		sineAngle = 0;
		globalAngle = 0;
		ninetyDegreeDetectFlag = 0;
	}
	EALLOW;
	EPwm1Regs.CMPA.half.CMPA = flyBackDuty1;
	EPwm2Regs.CMPA.half.CMPA = flyBackDuty2;

	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.bit.ACK1= 1;   // Acknowledge interrupt to PIE
	EDIS;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;

}

#pragma CODE_SECTION(fullBridgeDrive, "ramfuncs");
void fullBridgeDrive(void)
{
	// Based on where we are in the sinewave enable/disable PWM3 Gate Drive (Full-Bridge)

	switch(fullBridgeState)
	{
		case FULLBRIDGE_Q3Q4_ACTIVE:
		{
			/*if(globalAngle < NINETYDEGREE)
			{
				prevFullBridgeState = GpioDataRegs.GPADAT.bit.GPIO9;
			}*/

			if (globalAngle > ONEHUNDREDSEVENTYFIVEDEGREE)
			{
				fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

			}
		}
		break;

		case FULLBRIDGE_INACTIVE_2ND_QUADRANT:
		{
			// Check for Hardware/Software Zero Cross
			if(thirdQuadrantFlag == 1)
			// if((GpioDataRegs.GPADAT.bit.GPIO22 == 0) && (thirdQuadrantFlag == 1))
			{
				zcCounter++;

				if(zcCounter >= zeroCrossDelay)
				{
					/*if((prevFullBridgeState == GpioDataRegs.GPADAT.bit.GPIO9) && (systemState == GRIDMODE))
					{
						if(faultState == NO_FAULT)
						{
							// faultState = HARDWAREZEROCROSS;
						}
					}*/


					zcCounter = 0;
					thirdQuadrantFlag = 0;
					fullBridgeState = FULLBRIDGE_Q2Q5_ACTIVE;
					EALLOW;
					EPwm1Regs.TZCLR.bit.OST = 1;
					EPwm1Regs.TZCLR.bit.INT = 1;
					EPwm2Regs.TZCLR.bit.OST = 1;
					EPwm2Regs.TZCLR.bit.INT = 1;
					EPwm3Regs.TZCLR.bit.OST = 1;
					EPwm3Regs.TZCLR.bit.INT = 1;
					EDIS;
					PieCtrlRegs.PIEACK.bit.ACK2 = 1;
				}
			}
			else
			{
				zcCounter = 0;
				break;
			}
		}
		break;

		case FULLBRIDGE_Q2Q5_ACTIVE:
		{
		/*	if(globalAngle < NINETYDEGREE)
			{
				prevFullBridgeState = GpioDataRegs.GPADAT.bit.GPIO9;
			}
*/
			if (globalAngle > ONEHUNDREDSEVENTYFIVEDEGREE)
			{
				fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;
			//	fullbridgechangeoverFlag = 1;

			}
		}
		break;

		case FULLBRIDGE_INACTIVE_4TH_QUADRANT:
		{
			// Check for Hardware/Software Zero Cross
			if (firstQuadrantFlag == 1)
			// if ((GpioDataRegs.GPADAT.bit.GPIO22 == 1) && (firstQuadrantFlag == 1))
			{
				zcCounter++;

				if(zcCounter >= zeroCrossDelay)
				{
					/*if((prevFullBridgeState == GpioDataRegs.GPADAT.bit.GPIO9) && (systemState == GRIDMODE))
					{
						if(faultState == NO_FAULT)
						{
						//	faultState = HARDWAREZEROCROSS;
						}
					}*/


					zcCounter = 0;
					firstQuadrantFlag = 0;
					fullBridgeState = FULLBRIDGE_Q3Q4_ACTIVE;
					EALLOW;
					EPwm1Regs.TZCLR.bit.OST = 1;
					EPwm1Regs.TZCLR.bit.INT = 1;
					EPwm2Regs.TZCLR.bit.OST = 1;
					EPwm2Regs.TZCLR.bit.INT = 1;
					EPwm3Regs.TZCLR.bit.OST = 1;
					EPwm3Regs.TZCLR.bit.INT = 1;
					EDIS;
					PieCtrlRegs.PIEACK.bit.ACK2 = 1;
				}
			}
			else
			{
				zcCounter = 0;
				break;
			}
		}
		break;
	}

	// Now that we know the state we can modify the PWM outputs

	if(fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE)
	{
		//GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
		OPTO_DRV1 = 1;
		// GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
		OPTO_DRV2 = 0;
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFB = 0x3;
		EDIS;
	}
	else if (fullBridgeState == FULLBRIDGE_INACTIVE_2ND_QUADRANT)
	{
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 1;
		EPwm3Regs.AQCSFRC.bit.CSFB = 1;
		EDIS;

		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 1;
	}
	else if (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE)
	{

		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 0;
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 0x3;
		EDIS;
	}
	else
	{
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 1;
		EPwm3Regs.AQCSFRC.bit.CSFB = 1;
		EDIS;
		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 1;
	}
}


void fullBridgeDrive_off(void)
{
	// Based on where we are in the sinewave enable/disable PWM3 Gate Drive (Full-Bridge)

	switch(fullBridgeState)
	{
		case FULLBRIDGE_Q3Q4_ACTIVE:
		{
		if	(inverterPeriodCounter_off < 280)									//	if(globalAngle < NINETYDEGREE)
		{
			firstQuadrantFlag  = 1;
		}
		else
		{
			firstQuadrantFlag  = 0;
		}

		if (inverterPeriodCounter_off > 559)
		{
			fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

		}
		//dtime_counter = 0;
		}
		break;

		case FULLBRIDGE_INACTIVE_2ND_QUADRANT:
		{
			{
				zcCounter++;
				dtime_counter++;
				sineAngle = 0;
				if 	(dtime_counter > 6)
				{
					if(systemState == OFFGRIDMODE)
					{

						zcCounter = 0;
						fullBridgeState = FULLBRIDGE_Q2Q5_ACTIVE;
						EALLOW;
						EPwm1Regs.TZCLR.bit.OST = 1;
						EPwm1Regs.TZCLR.bit.INT = 1;
						EPwm2Regs.TZCLR.bit.OST = 1;
						EPwm2Regs.TZCLR.bit.INT = 1;
						EPwm3Regs.TZCLR.bit.OST = 1;
						EPwm3Regs.TZCLR.bit.INT = 1;
						EDIS;
						PieCtrlRegs.PIEACK.bit.ACK2 = 1;
						dtime_counter = 0;
					}
				}

			}
		}
		break;

		case FULLBRIDGE_Q2Q5_ACTIVE:
		{
			if (inverterPeriodCounter_off < 840)					//if(globalAngle < NINETYDEGREE)
			{
			thirdQuadrantFlag = 1;
			}
			else { thirdQuadrantFlag = 0;}

	  	 	if (inverterPeriodCounter_off > 1118)			//	if (globalAngle > ONEHUNDREDSEVENTYFIVEDEGREE)
			{
				fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

			}
			//dtime_counter = 0;
		}
		break;

		case FULLBRIDGE_INACTIVE_4TH_QUADRANT:
		{

				zcCounter++;
					dtime_counter++;
					sineAngle = 0;
				if 	(dtime_counter > 6){

				{
					if(systemState == OFFGRIDMODE)
					{

						EALLOW;
						EPwm1Regs.TZCLR.bit.OST = 1;
						EPwm1Regs.TZCLR.bit.INT = 1;
						EPwm2Regs.TZCLR.bit.OST = 1;
						EPwm2Regs.TZCLR.bit.INT = 1;
						EPwm3Regs.TZCLR.bit.OST = 1;
						EPwm3Regs.TZCLR.bit.INT = 1;
						EDIS;
						PieCtrlRegs.PIEACK.bit.ACK2 = 1;
						zcCounter = 0;
						fullBridgeState = FULLBRIDGE_Q3Q4_ACTIVE;
						dtime_counter = 0;
					}
				}

		}
		break;
	}
}

	// Now that we know the state we can modify the PWM outputs

	if(fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE)
	{
		//GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
		OPTO_DRV1 = 1;
		// GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
		OPTO_DRV2 = 0;
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFB = 0x3;
		EDIS;
	}
	else if (fullBridgeState == FULLBRIDGE_INACTIVE_2ND_QUADRANT)
	{
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 1;
		EPwm3Regs.AQCSFRC.bit.CSFB = 1;
		EDIS;
		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 1;
	}
	else if (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE)
	{
		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 0;
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 0x3;
		EDIS;
	}
	else
	{
		EALLOW;
		EPwm3Regs.AQCSFRC.bit.CSFA = 1;
		EPwm3Regs.AQCSFRC.bit.CSFB = 1;
		EDIS;
		// GpioDataRegs.GPASET.bit.GPIO24 = 1;
		//GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		OPTO_DRV2 = 1;
		OPTO_DRV1 = 1;
	}
}

void PWMSTOP(void)
{
	EALLOW;
	GpioDataRegs.GPASET.bit.GPIO28 = 1;  //set PWMshutdown
	EPwm1Regs.TZFRC.bit.OST = 1;
	EPwm2Regs.TZFRC.bit.OST = 1;
	EPwm3Regs.TZFRC.bit.OST = 1;
	EDIS;

}
