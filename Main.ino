/************************************
 * File Name    : main.ino          *
 * Version      : 0.11              *
 * Description  : main program      *
 * IDE          : Arduino IDE       *
 * Author       : Collin Haghuis    *
 * Created on   : 13-12-19          *
 * Last Updated : 13-01-20          *
 * Update note  : Added RTC         *
 ************************************/

/*The calibration inputs are stored in the ADE9000CalibrationInputs.h file. The phase and parameter being calibrated is input through the serial console*/
/*Calbration constants are computed and stored in the EEPROM */
/*Caibration should be done with the end application settings. If any  parameters(GAIN,High pass corner,Integrator settings) are changed, the device should be recalibrated*/
/*This application assumues the PGA_GAIN among all current channels is same.Also, the PGA_GAIN among all voltage channels should be same*/

/*** INCLUDES ***/
#include "ADE9000regmap.h"
#include "ADE9000.h"
#include "ADE9000SPI.h"
#include "RTC.h"
#include <math.h>

/*** DEFINES ***/

/*** VARIABLES ***/
/* Global variables */
struct CurrentGainRegs iGain;
struct CurrentRMSVals measCurVal;
struct TimeAndDate rtcStamp;

/* timer variables */
int32_t timerCounter1 = 500; //500 * 10ms = 5seconds
bool timerFlag1 = false;

/*** FUNCTION PROTOTYPES ***/
void timer_Counter1(void);

void setup() 
{  
  init_RTC(&rtcStamp); //Inits the RTC 
  SPI_Init(SPI_SPEED, CS_PIN); //Inits the SPI connection
  Serial.begin(115200);
  ADE9000_PinInit(); //Inits the pins for the power mode of the ADE9000
  ADE9000_Init(); //setup all registers for the ADE9000 like the PGA and such
  ADE9000_CurrentGainCalibration(&iGain); //calibrates the xIRMS registers
  ADE9000_DeepSleep();  //puts the ADE9000 in energy saving power mode (PSM3)
}


void loop()
{
  timer_Counter1();

  /*** START MEASUREMENT ROUTINE ***/
  if (timerFlag1 == true)
  {
    /* Reset timer */
    timerFlag1 = false;
    timerCounter1 = 500; //500 * 10ms = 5seconds
    /* Waking up the ADE9000 */
    ADE9000_WakeUp(); //puts the ADE9000 in normal mode (PSM0)
    delay(50); //TO DO: Research why the IRQ1 pin interrupt isn't enough in the WakeUp function.
    //TO DO: Make sure ADE9000 is powered on
    /* Initializing the ADE9000 */
    ADE9000_Init();
    
   struct TemperatureRegnValue temp_data;
  ADE9000_ReadTempRegnValue(&temp_data);
  float temperature = temp_data.Temperature;
  uint16_t temp_reg = temp_data.Temperature_Reg;
  
  // Print temperature data
  Serial.print("Temperature Register Value: ");
  Serial.println(temp_reg);
  Serial.print("Temperature: ");
  Serial.println(temperature);

   


    
    ADE9000_WriteCurrentGainRegs(&iGain);  //write the calibration values to the ADE9000
    delay(500); //TO DO: Replace with an interrupt
    /* gets the RTC and current values */
    get_RTC(&rtcStamp);
    ADE9000_ReadCurrentRMSValues(&measCurVal); //measure all current channels in Ampère
    ADE9000_NullingZeroGainChannels(&iGain, &measCurVal); //if the gain of a channel is 0, the irms value is also 0
    /* writes RTC and current values to the debug console */ 
    write_RTC2Debug(&rtcStamp); //writes the RTC to the debug console
    ADE9000_WriteRMSValues2Debug(&measCurVal); //writes the measurements to the debug console    
 
    /* writes RTC and current values to the SRAM */  
    //TO DO: Store the time stamp in the SRAM next to the corresponding xIRMS measurements
    //TO DO: Store the xIRMS values in the SRAM (SRAM_Write(&measCurVal))    
    /* Putting the ADE9000 in sleep mode */
    ADE9000_DeepSleep(); //puts the ADE9000 in sleep mode (PSM3)
  }
}

//TO DO: replace with a timer interrupt of 15 minutes
void timer_Counter1(void)
{
  delay(10);
  --timerCounter1;
  if (timerCounter1 <= 0) timerFlag1 = true;
}
