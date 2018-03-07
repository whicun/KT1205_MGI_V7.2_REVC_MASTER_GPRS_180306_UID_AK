KT1205_MGI_V5_160627_STEP20
INVERTER OVERCURRENT trip is set at 250 (INVERTER_OUTPUTCURRENT_MAX //14000 ~1.089A=250W). now the value is changed to 14500.
inverter will not feed more than 250W because added the following code in statemachine. 
Due to this inverter overcurrent trip will not occur under normal working condition.
	//160629
	if ((systemState == GRIDMODE)&& (mpptFactor > 23200)) //650 ~ 7W => 250W ~ 23200
	{
		mpptlimitcounter++;
		if (mpptlimitcounter >= 2)
		{
			mpptFactorMaximum = 23200;

			mpptlimitcounter = 0;
		}
		/*else
		{
			mpptFactorMaximum = 25000;
		}*/
	}
	//160629


******************************************************************
160623:updated version of KT1205_MGI_V4.6_160622
----------Issue is resolved:If pv supply is off and On means grid tie mode detection loop is not working because of 640ms Peakgrid voltage sampling and averaging.
640ms Peakgrid voltage sampling and averaging loop is not working properly. 
so removed that loop and modified the code in such a way that in ISR.c 2peak grid voltage average value is stored in avgPeakGridVotlage for tripping less than 110v and above 310v. 
At every 2peak grid voltage average the twoAvgPeakGridVoltageflag is set as 1. The avgPeakGridVotlage value is again sampled for 16 time and this value is stored in avgPeakGridVotlage1 for above 110v and less than 310v limit. 
----------
++++
when mpptFactor is less than 500 means total output reduced by 4. so that the number of pv low voltage trip will reduce.
if (mpptFactor <= 300)  // 500
		{
			totalOutput = totalOutput >> 2 ;
		}
+++++

******************************************************************
160622:updated version of KT1205_MGI_V4.5_160621
++++
PV low vow voltage trip is 19 and recovered at 24. PV over voltage trip is at 50v and recovered at 47v.
Inverter will start above 24v in grid tie mode.
MININCREMENTMPPTFACTOR & MINDECREMENTMPPTFACTOR value is reduced from 20 to 10.
++++
In this version added KT1205_MGI_V4.2_160616 version peadgridvoltage average calculation loop. 
In grid tie mode, If pv supply is off and On means inverter is working in off grid mode instead of grid mode. 
For time being peakgridvoltage hysteresis loop is place in statemachine().
Issue:
If pv supply is off and On means grid tie mode detection loop is not working because of 640ms Peakgrid voltage sampling and averaging.




******************************************************************
160621:updated version of KT1205_MGI_V4.4_160620
In this version added KT1205_MGI_V4.2_160616 version peadgridvoltage average calculation loop. 
In grid tie mode, If pv supply is off and On means inverter is working in off grid mode instead of grid mode. 
For time being peakgridvoltage hysteresis loop is place in statemachine().
Issues:
If pv supply is off and On means grid tie mode detection loop is not working because of 640ms Peakgrid voltage sampling and averaging.

******************************************************************

160620: KT1205_MGI_V4.3_160620
In enclosure grid tie mode startup issue is solve in this version.
Inverter starts in grid tie mode after 25seconds from the systemerror state.
	System error Restart_Count is 20 Seconds.
	startFullBridgeFlag is set after the 20 Seconds in systemstartup condition.
	Gridmode relay is enabled when inputvoltage above 19V, then GRIDMODE is set after 5 second. This 5 second delay is resolved the issue of within enclosure inverter overcu 
And also removed the following code in flybackloop().
		if (mpptFactor <= 750)
		{
			totalOutput = totalOutput >> 2 ;
		}

******************************************************************
160618: KT1205_MGI_V4.2_160618
In enclosure grid tie mode startup issue is solve in this version by adding the following code at flybackcontrolloop() in isr.c.
		if (mpptFactor <= 750)
		{
			totalOutput = totalOutput >> 2 ;
		}

This will reduce the inverter output current during inverter startup.


************************************************************************
160603:
Dual Mode code with all trips.
MPPT and THD fine tuned.
Short circuit protection for grid tie mode implemented.
Grid tie mode reconnection time 20s.
Off-grid mode re-connectiontime 20s.
One transformer works till 70W after with both of them work.  
Trip value wrt IEC62116.


*****************************************************************************
1602016:
New Board version B has two relay configuration with RC filter.
Grid-tie mode:
 1.THD fine tunning 
 Hareware Changes: 
 Removing all resisters in ADC section and replacing 0.001uF in Grid_AC_voltage sense and AC_crrent sense line.
 Software Changes:
 volatile float Ra_float = 0.055, Rsa_float = 0.028;
 volatile unsigned int Ra = 0, Rsa = 0,MPPTFACTOR_BENCHTESTING = 23000;
volatile long temp_Ioutput = -500;

Off-Gride mode:
1.No changes.


****************************************************************************
New Board version B has two relay configuration with RC filter.
Grid Tie THD is 90% at full load.
Off grid THD is 2.5% 215W load.

off grid works at minimum 5W load resistor not required 15W Lamp. 


******************************************************************************
151218:
update version of KT1205_MGI_DMI_V2_0_151126
New Board version B has two relay configuration, the following changes are done in statemachine for the respective modes.
Grid tie mode:
				GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;  	//Relay Drive1 in grid mode
				GpioDataRegs.GPASET.bit.GPIO23 = 1;		//Relay Drive2 in grid mode
Off Grid mode:
			   GpioDataRegs.GPBSET.bit.GPIO32 = 1;  	//Relay Drive1 in off grid mode
			   GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;  	//Relay Drive2 in off grid mode

-----------------------------------------

KT1205_MGI_DMI_V2_0_151126

1. Short circuit test code is not implemented. Need to implement.
2. All trips are implemented except inverter frequency and Overcurrent trip. Inverter overcurrent trip is masked and its sampling period is 600ms. LED fault indication is implemented and its working fine. 
3. For MPPT fine tuning added these lines in MPPT routine 	
	if (inputVoltageAverage < 14600)
	{
		mpptFactor = mpptFactor - (mpptFactor >> 2);		// Reset to 75% power
	}
4. In Microchip RDB, inputVoltageAverage is updated every 2 cycle(zeroCrossCount is 0-3) because of 5 Electrolytic capacitor. 
	if(zeroCrossCount >= 3)
 	But Kripya RDB board has 4 Electrolytic capacitor and we could identified that because of that timing issues has arrived.
 	Because of the timing issue, reduced zeroCrossCount to 2.
 	if(zeroCrossCount >= 2)
5. Faut indication delays or sampling time
	1. PV TRIP  - 100ms
	2. Grid Mode		: Grid Voltage sampling time is 10uS
	3. Off Grid mode (IS MASKED)	: Inverer Voltage sampling time is 600uS but during off grid mode startupInverter undervoltage trip is occured. So introduced 6Second time delay
		