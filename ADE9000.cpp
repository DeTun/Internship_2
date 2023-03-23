/************************************
 * File Name    : ADE9000.cpp       *
 * Version      : 0.8               *
 * Description  : ADE9000 Driver    *
 *              : register init     *
 *              : calibration       *
 * IDE          : Arduino IDE       *
 * Author       : Collin Haghuis    *
 * Created on   : 13-12-19          *
 * Last Updated : 13-01-20          *
 * Update note  :                   *
 ************************************/

/*** INCLUDES ***/
#include <Arduino.h>
#include "ADE9000.h"
#include "ADE9000regmap.h"
#include "ADE9000SPI.h"

/*** DEFINITIONS ***/
#define DEBUG

/*
Description: Initializes the ADE9000. The initial settings for registers are defined in ADE9000API.h header file
Input: Register settings in header files
Output:
*/
void ADE9000_Init(void)
{
  SPI_Write_16(ADDR_PGA_GAIN,ADE9000_PGA_GAIN);
  SPI_Write_32(ADDR_CONFIG0,ADE9000_CONFIG0);
  SPI_Write_16(ADDR_CONFIG1,ADE9000_CONFIG1);
  SPI_Write_16(ADDR_CONFIG2,ADE9000_CONFIG2);
  SPI_Write_16(ADDR_CONFIG3,ADE9000_CONFIG3);
  SPI_Write_16(ADDR_ACCMODE,ADE9000_ACCMODE);
  SPI_Write_16(ADDR_TEMP_CFG,ADE9000_TEMP_CFG);
  SPI_Write_16(ADDR_ZX_LP_SEL,ADE9000_ZX_LP_SEL);
  SPI_Write_32(ADDR_MASK0,ADE9000_MASK0);
  SPI_Write_32(ADDR_MASK1,ADE9000_MASK1);
  SPI_Write_32(ADDR_EVENT_MASK,ADE9000_EVENT_MASK);
  SPI_Write_16(ADDR_WFB_CFG,ADE9000_WFB_CFG);
  SPI_Write_32(ADDR_VLEVEL,ADE9000_VLEVEL);
  SPI_Write_32(ADDR_DICOEFF,ADE9000_DICOEFF);
  SPI_Write_16(ADDR_EGY_TIME,ADE9000_EGY_TIME);
  SPI_Write_16(ADDR_EP_CFG,ADE9000_EP_CFG);    //Energy accumulation ON
  SPI_Write_16(ADDR_RUN, ADE9000_RUN_ON);   //DSP ON
  
  //Wait till the last register of the init is succesfully written
  while (SPI_Read_16(ADDR_RUN) != ADE9000_RUN_ON); //TO DO: Add error codes
}

/*
Description: Reads the Active power registers AWATT,BWATT and CWATT
Input: Structure name
Output: Active power codes stored in respective structure
*/
void ADE9000_ReadActivePowerRegs(struct ActivePowerRegs *data)
{
//  data->ActivePowerReg_A = (int32_t) (SPI_Read_32(ADDR_AWATT));
//  data->ActivePowerReg_B = (int32_t) (SPI_Read_32(ADDR_BWATT));
//  data->ActivePowerReg_C = (int32_t) (SPI_Read_32(ADDR_CWATT));
}

void ADE9000_ReadReactivePowerRegs(struct ReactivePowerRegs *data)
{
//  data->ReactivePowerReg_A = (int32_t) (SPI_Read_32(ADDR_AVAR));
//  data->ReactivePowerReg_B = (int32_t) (SPI_Read_32(ADDR_BVAR));
//  data->ReactivePowerReg_C = (int32_t) (SPI_Read_32(ADDR_CVAR));
}

void ADE9000_ReadApparentPowerRegs(struct ApparentPowerRegs *data)
{
//  data->ApparentPowerReg_A = (int32_t) (SPI_Read_32(ADDR_AVA));
//  data->ApparentPowerReg_B = (int32_t) (SPI_Read_32(ADDR_BVA));
//  data->ApparentPowerReg_C = (int32_t) (SPI_Read_32(ADDR_CVA));
}

void ADE9000_ReadVoltageRMSRegs(struct VoltageRMSRegs *data)
{
//  data->VoltageRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AVRMS));
//  data->VoltageRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BVRMS));
//  data->VoltageRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CVRMS));
}

void ADE9000_ReadCurrentRMSRegs(struct CurrentRMSRegs *data)
{
  data->CurrentRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AIRMS));
  data->CurrentRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BIRMS));
  data->CurrentRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CIRMS));
  data->CurrentRMSReg_N = (int32_t) (SPI_Read_32(ADDR_NIRMS));
}

void ADE9000_ReadFundActivePowerRegs(struct FundActivePowerRegs *data)
{
//  data->FundActivePowerReg_A = (int32_t) (SPI_Read_32(ADDR_AFWATT));
//  data->FundActivePowerReg_B = (int32_t) (SPI_Read_32(ADDR_BFWATT));
//  data->FundActivePowerReg_C = (int32_t) (SPI_Read_32(ADDR_CFWATT));
}

void ADE9000_ReadFundReactivePowerRegs(struct FundReactivePowerRegs *data)
{
//  data->FundReactivePowerReg_A = (int32_t) (SPI_Read_32(ADDR_AFVAR));
//  data->FundReactivePowerReg_B = (int32_t) (SPI_Read_32(ADDR_BFVAR));
//  data->FundReactivePowerReg_C = (int32_t) (SPI_Read_32(ADDR_CFVAR));
}

void ADE9000_ReadFundApparentPowerRegs(struct FundApparentPowerRegs *data)
{
//  data->FundApparentPowerReg_A = (int32_t) (SPI_Read_32(ADDR_AFVA));
//  data->FundApparentPowerReg_B = (int32_t) (SPI_Read_32(ADDR_BFVA));
//  data->FundApparentPowerReg_C = (int32_t) (SPI_Read_32(ADDR_CFVA));
}

void ADE9000_ReadFundVoltageRMSRegs(struct FundVoltageRMSRegs *data)
{
//  data->FundVoltageRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AVFRMS));
//  data->FundVoltageRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BVFRMS));
//  data->FundVoltageRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CVFRMS));
}

void ADE9000_ReadFundCurrentRMSRegs(struct FundCurrentRMSRegs *data)
{
//  data->FundCurrentRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AIFRMS));
//  data->FundCurrentRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BIFRMS));
//  data->FundCurrentRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CIFRMS));
}

void ADE9000_ReadHalfVoltageRMSRegs(struct HalfVoltageRMSRegs *data)
{
//  data->HalfVoltageRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AVRMSONE));
//  data->HalfVoltageRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BVRMSONE));
//  data->HalfVoltageRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CVRMSONE));
}

void ADE9000_ReadHalfCurrentRMSRegs(struct HalfCurrentRMSRegs *data)
{
//  data->HalfCurrentRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AIRMSONE));
//  data->HalfCurrentRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BIRMSONE));
//  data->HalfCurrentRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CIRMSONE));
//  data->HalfCurrentRMSReg_N = (int32_t) (SPI_Read_32(ADDR_NIRMSONE));
}

void ADE9000_ReadTen12VoltageRMSRegs(struct Ten12VoltageRMSRegs *data)
{
//  data->Ten12VoltageRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AVRMS1012));
//  data->Ten12VoltageRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BVRMS1012));
//  data->Ten12VoltageRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CVRMS1012));
}

void ADE9000_ReadTen12CurrentRMSRegs(struct Ten12CurrentRMSRegs *data)
{
//  data->Ten12CurrentRMSReg_A = (int32_t) (SPI_Read_32(ADDR_AIRMS1012));
//  data->Ten12CurrentRMSReg_B = (int32_t) (SPI_Read_32(ADDR_BIRMS1012));
//  data->Ten12CurrentRMSReg_C = (int32_t) (SPI_Read_32(ADDR_CIRMS1012));
//  data->Ten12CurrentRMSReg_N = (int32_t) (SPI_Read_32(ADDR_NIRMS1012));
}

void ADE9000_ReadVoltageTHDRegsnValues(struct VoltageTHDRegs *data)
{
//  uint32_t tempReg;
//  float tempValue;
//
//  tempReg = (int32_t) (SPI_Read_32(ADDR_AVTHD)); //Read THD register
//  data->VoltageTHDReg_A = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->VoltageTHDValue_A = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_BVTHD)); //Read THD register
//  data->VoltageTHDReg_B = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->VoltageTHDValue_B = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_CVTHD)); //Read THD register
//  data->VoltageTHDReg_C = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->VoltageTHDValue_C = tempValue;
}

void ADE9000_ReadCurrentTHDRegsnValues(struct CurrentTHDRegs *data)
{
//  uint32_t tempReg;
//  float tempValue;
//
//  tempReg = (int32_t) (SPI_Read_32(ADDR_AITHD)); //Read THD register
//  data->CurrentTHDReg_A = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->CurrentTHDValue_A = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_BITHD)); //Read THD register
//  data->CurrentTHDReg_B = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->CurrentTHDValue_B = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_CITHD)); //Read THD register
//  data->CurrentTHDReg_C = tempReg;
//  tempValue = (float)tempReg * 100 / (float)134217728; //Calculate THD in %
//  data->CurrentTHDValue_C = tempValue;
}

void ADE9000_ReadPowerFactorRegsnValues(struct PowerFactorRegs *data)
{
//  uint32_t tempReg;
//  float tempValue;
//
//  tempReg=(int32_t) (SPI_Read_32(ADDR_APF)); //Read PF register
//  data->PowerFactorReg_A = tempReg;
//  tempValue = (float)tempReg / (float)134217728; //Calculate PF
//  data->PowerFactorValue_A = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_BPF)); //Read PF register
//  data->PowerFactorReg_B = tempReg;
//  tempValue = (float)tempReg / (float)134217728; //Calculate PF
//  data->PowerFactorValue_B = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_CPF)); //Read PF register
//  data->PowerFactorReg_C = tempReg;
//  tempValue = (float)tempReg / (float)134217728; //Calculate PF
//  data->PowerFactorValue_C = tempValue;
}

void ADE9000_ReadPeriodRegsnValues(struct PeriodRegs *data)
{
//  uint32_t tempReg;
//  float tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_APERIOD)); //Read PERIOD register
//  data->PeriodReg_A = tempReg;
//  tempValue = (float)(8000 * 65536) / (float)(tempReg + 1); //Calculate Frequency
//  data->FrequencyValue_A = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_BPERIOD)); //Read PERIOD register
//  data->PeriodReg_B = tempReg;
//  tempValue = (float)(8000 * 65536) / (float)(tempReg + 1); //Calculate Frequency
//  data->FrequencyValue_B = tempValue;
//  tempReg = (int32_t) (SPI_Read_32(ADDR_CPERIOD)); //Read PERIOD register
//  data->PeriodReg_C = tempReg;
//  tempValue = (float)(8000 * 65536) / (float)(tempReg + 1); //Calculate Frequency
//  data->FrequencyValue_C = tempValue;
}

void ADE9000_ReadAngleRegsnValues(struct AngleRegs *data)
{
//  uint32_t tempReg;
//  uint16_t temp;
//  float mulConstant;
//  float tempValue;
//
//  temp=SPI_Read_16(ADDR_ACCMODE); //Read frequency setting register
//  if((temp&0x0100) >= 0)
//    {
//      mulConstant = 0.02109375;  //multiplier constant for 60Hz system
//    }
//  else
//    {
//      mulConstant = 0.017578125; //multiplier constant for 50Hz system
//    }
//
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VA_VB)); //Read ANGLE register
//  data->AngleReg_VA_VB = tempReg;
//  tempValue = tempReg * mulConstant;  //Calculate Angle in degrees
//  data->AngleValue_VA_VB = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VB_VC));
//  data->AngleReg_VB_VC = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_VB_VC = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VA_VC));
//  data->AngleReg_VA_VC = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_VA_VC = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VA_IA));
//  data->AngleReg_VA_IA = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_VA_IA = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VB_IB));
//  data->AngleReg_VB_IB = tempReg;
//  tempValue=tempReg * mulConstant;
//  data->AngleValue_VB_IB = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_VC_IC));
//  data->AngleReg_VC_IC = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_VC_IC = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_IA_IB));
//  data->AngleReg_IA_IB = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_IA_IB = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_IB_IC));
//  data->AngleReg_IB_IC = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_IB_IC = tempValue;
//  tempReg = (int16_t) (SPI_Read_32(ADDR_ANGL_IA_IC));
//  data->AngleReg_IA_IC = tempReg;
//  tempValue = tempReg * mulConstant;
//  data->AngleValue_IA_IC = tempValue;
}

/*
Description: Starts a new acquisition cycle. Waits for constant time and returns register value and temperature in Degree Celsius
Input:  Structure name
Output: Register reading and temperature value in Degree Celsius
*/
void ADE9000_ReadTempRegnValue(struct TemperatureRegnValue *Data) {
    uint32_t trim;
    uint16_t gain;
    uint16_t offset;
    uint16_t tempReg; 
    float tempValue;
    
    SPI_Write_16(ADDR_TEMP_CFG, ADE9000_TEMP_CFG); // Start temperature acquisition cycle with settings in defined in ADE9000_TEMP_CFG
    delay(2); // delay of 2ms. Increase delay if TEMP_TIME is changed

    trim = SPI_Read_32(ADDR_TEMP_TRIM);
    gain = (trim & 0xFFFF);  // Extract 16 LSB
    offset = ((trim >> 16) & 0xFFFF); // Extract 16 MSB
    tempReg = SPI_Read_16(ADDR_TEMP_RSLT); // Read Temperature result register
    tempValue = (float)(offset >> 5) - ((float)tempReg * (float)gain / (float)65536); 
    
    Data->Temperature_Reg=tempReg;
    Data->Temperature=tempValue;
}

void ADE9000_ReadCurrentRMSValues(struct CurrentRMSVals *data)
{
  data->CurrentRMSVal_A = (float) (SPI_Read_32(ADDR_AIRMS) * IRMS_CC);
  data->CurrentRMSVal_B = (float) (SPI_Read_32(ADDR_BIRMS) * IRMS_CC);
  data->CurrentRMSVal_C = (float) (SPI_Read_32(ADDR_CIRMS) * IRMS_CC);
  data->CurrentRMSVal_N = (float) (SPI_Read_32(ADDR_NIRMS) * IRMS_CC);
}

void ADE9000_WriteRMSValues2Debug(struct CurrentRMSVals *data)
{
  #ifdef DEBUG
    Serial.print("Measured current channl A: ");
    if (data->CurrentRMSVal_A < 1.0) Serial.print(0.0);
    else Serial.print(data->CurrentRMSVal_A);
    Serial.println(" Arms");
    Serial.print("Measured current channl B: ");
    if (data->CurrentRMSVal_B < 1.0) Serial.print(0.0);
    else Serial.print(data->CurrentRMSVal_B);
    Serial.println(" Arms");
    Serial.print("Measured current channl C: ");
    if (data->CurrentRMSVal_C < 1.0) Serial.print(0.0);
    else Serial.print(data->CurrentRMSVal_C);
    Serial.println(" Arms");
    Serial.print("Measured current channl N: ");
    if (data->CurrentRMSVal_N < 1.0) Serial.print(0.0);
    else Serial.print(data->CurrentRMSVal_N);
    Serial.println(" Arms");
  #endif
}

/*
Description: Computing The xIGAIN registers that calibrates the xIRMS registers
The xIRMS current registers must be in between the inom_min and inom_max to compute the xIGAIN correctly
if not: the xIGAIN for that channel is set to 0
xIGAIN = ((expected IRMS / actual IRMS) - 1) * 2^27
*/
void ADE9000_ComputeCurrentGainRegs(struct CurrentGainRegs *data1, struct CurrentRMSRegs *data2)
{
  if ((data2->CurrentRMSReg_A > I_NOM_MIN) && (data2->CurrentRMSReg_A < I_NOM_MAX))
  {  
    data1->CurrentGainReg_A = (int32_t) ((((float) IRMS_EXP / (float) data2->CurrentRMSReg_A) - 1) * 134217728);
  }
  else data1->CurrentGainReg_A = 0x0000000;
  if ((data2->CurrentRMSReg_B > I_NOM_MIN) && (data2->CurrentRMSReg_B < I_NOM_MAX))
  {
    data1->CurrentGainReg_B = (int32_t) ((((float) IRMS_EXP / (float) data2->CurrentRMSReg_B) - 1) * 134217728);
  }
  else data1->CurrentGainReg_B = 0x0000000;
  if ((data2->CurrentRMSReg_C > I_NOM_MIN) && (data2->CurrentRMSReg_C < I_NOM_MAX))
  {
    data1->CurrentGainReg_C = (int32_t) ((((float) IRMS_EXP / (float) data2->CurrentRMSReg_C) - 1) * 134217728);
  }
  else data1->CurrentGainReg_C = 0x0000000;
  if ((data2->CurrentRMSReg_N > I_NOM_MIN) && (data2->CurrentRMSReg_N < I_NOM_MAX))
  {
    data1->CurrentGainReg_N = (int32_t) ((((float) IRMS_EXP / (float) data2->CurrentRMSReg_N) - 1) * 134217728);
  }
  else data1->CurrentGainReg_N = 0x0000000;
}

/*
Description: Nulls the gain registers
*/
void ADE9000_NullingCurrentGainRegs(struct CurrentGainRegs *data)
{
  data->CurrentGainReg_A = 0x00000000;
  data->CurrentGainReg_B = 0x00000000;
  data->CurrentGainReg_C = 0x00000000;
  data->CurrentGainReg_N = 0x00000000;
}

/*
Description: Writes the current gain of all channels to the xIGAIN registers of the ADE9000
*/
void ADE9000_WriteCurrentGainRegs(struct CurrentGainRegs *data)
{
  SPI_Write_32(ADDR_AIGAIN, data->CurrentGainReg_A);
  SPI_Write_32(ADDR_BIGAIN, data->CurrentGainReg_B);
  SPI_Write_32(ADDR_CIGAIN, data->CurrentGainReg_C);
  SPI_Write_32(ADDR_NIGAIN, data->CurrentGainReg_N);
}

/*
Description: Calibrates the xIRMS registers of the ADE9000
*/
void ADE9000_CurrentGainCalibration(struct CurrentGainRegs *gain)
{  
  char serialReadData; //variable for the serial input
  Serial.println(" Starting calibration process. Select one of the following {Start (Y/y) OR Abort(Q/q)}:");
  while (Serial.read() >= 0);  //Flush any extra characters
  while (!Serial.available());  //wait for serial data to be available
  serialReadData = Serial.read();
  struct CurrentRMSRegs data; //creating data structure for uncalibrated xIRMS registers
  //struct CurrentGainRegs xIGain; //creating data structure for xIGAIN registers
  if (serialReadData == 'Y' || serialReadData == 'y')
  {
    #ifdef DEBUG
      Serial.println("Starting calibration process. \r\n");
      Serial.print("Conversion Constant uArms/code: ");
      Serial.println(IRMS_CC * 1000000);
      Serial.print("Nominal test current: ");
      Serial.println(I_NOM);
    #endif
    
    ADE9000_NullingCurrentGainRegs(gain); //nulling the xIGAIN registers
    ADE9000_WriteCurrentGainRegs(gain); //nulling the xIGAIN registers
    ADE9000_ReadCurrentRMSRegs(&data); //read the uncalibrated actual xIRMS registers
    ADE9000_ComputeCurrentGainRegs (gain, &data); //compute the xIGAIN registers
    ADE9000_WriteCurrentGainRegs(gain); //writes the computed xIGAIN registers
    //WRITE &xIGain TO THE FLASH MEMORY (pflash) //THIS VALUE NEEDS TO BE OUTSIDE OF THE BOOTLOADER MEMORY RANGE
    
    #ifdef DEBUG
      Serial.print("Expected IRMS of current channel A: ");
      Serial.print(IRMS_EXP);
      Serial.println(" decimal code");
      //calculated xIRMS decimal code value
      Serial.print("Actual IRMS of current channel A: ");
      Serial.print(data.CurrentRMSReg_A);
      Serial.println(" decimal code");
      //xIGAIN decimal code value
      Serial.print("IGAIN register for current channel A: "); 
      Serial.print(gain->CurrentGainReg_A);
      Serial.println(" decimal code");
      //deviation floating point value
      if (gain->CurrentGainReg_A != 0)
      {
        Serial.print("IRMS deviation before calibration channel A: "); 
        Serial.print(((1 - ((float)data.CurrentRMSReg_A / (float)IRMS_EXP))) * 100);
        Serial.println(" (%)");
        Serial.println(); //spacer to get more distinction between other channels
      }
  
      Serial.print("Expected IRMS of current channel B: ");
      Serial.print(IRMS_EXP);
      Serial.println(" decimal code");
      //calculated xIRMS decimal code value
      Serial.print("Actual IRMS of current channel B: ");
      Serial.print(data.CurrentRMSReg_B);
      Serial.println(" decimal code");
      //xIGAIN decimal code value
      Serial.print("IGAIN register for current channel B: "); 
      Serial.print(gain->CurrentGainReg_B);
      Serial.println(" decimal code");
      //deviation floating point value
      if (gain->CurrentGainReg_B != 0)
      {
        Serial.print("IRMS deviation before calibration channel B: "); 
        Serial.print(((1 - ((float)data.CurrentRMSReg_B / (float)IRMS_EXP))) * 100);
        Serial.println(" (%)");
        Serial.println(); //spacer to get more distinction between other channels
      }
  
      Serial.print("Expected IRMS of current channel C: ");
      Serial.print(IRMS_EXP);
      Serial.println(" decimal code");
      //calculated xIRMS decimal code value
      Serial.print("Actual IRMS of current channel C: ");
      Serial.print(data.CurrentRMSReg_C);
      Serial.println(" decimal code");
      //xIGAIN decimal code value
      Serial.print("IGAIN register for current channel C: "); 
      Serial.print(gain->CurrentGainReg_C);
      Serial.println(" decimal code");
      //deviation floating point value
      if (gain->CurrentGainReg_C != 0)
      {
        Serial.print("IRMS deviation before calibration channel C: "); 
        Serial.print(((1 - ((float)data.CurrentRMSReg_C / (float)IRMS_EXP))) * 100);
        Serial.println(" (%)");
        Serial.println(); //spacer to get more distinction between other channels 
      } 
      
      Serial.print("Expected IRMS of current channel N: ");
      Serial.print(IRMS_EXP);
      Serial.println(" decimal code");
      //calculated xIRMS decimal code value
      Serial.print("Actual IRMS of current channel N: ");
      Serial.print(data.CurrentRMSReg_N);
      Serial.println(" decimal code");
      //xIGAIN decimal code value
      Serial.print("IGAIN register for current channel N: "); 
      Serial.print(gain->CurrentGainReg_N);
      Serial.println(" decimal code");
      //deviation floating point value
      if (gain->CurrentGainReg_N != 0)
      {
        Serial.print("IRMS deviation before calibration channel N: "); 
        Serial.print(((1 - ((float)data.CurrentRMSReg_N / (float)IRMS_EXP))) * 100);
        Serial.println(" (%)");
        Serial.println(); //spacer to get more distinction between other channels
      }
      Serial.println("Calibration process finished. \r\n");
    #endif
  }
  else 
  {
    #ifdef DEBUG
      Serial.println("Aborting calibration process");
      Serial.println();
    #endif
  }
}

/*
Description: Filters out the channels with no calibration or no load. (because Channel N wont be used most of the time)
if the gain of a channel is 0, the irms value of that channel is also 0
*/
void ADE9000_NullingZeroGainChannels(struct CurrentGainRegs *data1, struct CurrentRMSVals *data2)
{
  if (data1->CurrentGainReg_A == 0) data2->CurrentRMSVal_A = 0.0;
  if (data1->CurrentGainReg_B == 0) data2->CurrentRMSVal_B = 0.0;
  if (data1->CurrentGainReg_C == 0) data2->CurrentRMSVal_C = 0.0;
  if (data1->CurrentGainReg_N == 0) data2->CurrentRMSVal_N = 0.0;
}

/*
Description: Initializes the pins for the interrupts and powermodes
*/
void ADE9000_PinInit(void)
{
  pinMode(IRQ1_PIN, INPUT); //IRQ1 geeft aan wanneer de start up sequence voltooid is
  pinMode(IRQ0_PIN, INPUT); //IRQ0 
  pinMode(PM1_PIN, OUTPUT); //Set PM1 select pin as output 
  digitalWrite(PM1_PIN, LOW); //ADE9000 normal mode (PSM0)
}

/*
Description: Wakes up the ADE9000 (PSM3) by makeing PM1 low, when IRQ1 is low, ADE9000 is in PSM0.
IRQ1 is an interrupt pin and the RESETDONE interrupt is activated when the ADE9000 is succesfully in PSM0 mode, this will trigger IRQ1 to go low.
*/
void ADE9000_WakeUp(void) //replace this with an interrupt routine
{
  //write PM1 LOW (activate PSM1)
  digitalWrite(PM1_PIN, LOW);
  //while IRQ1 is HIGH
  while (digitalRead(IRQ1_PIN) == HIGH); //TO DO: Add error codes
  #ifdef DEBUG
    Serial.println("ADE9000 STATUS: AWAKE");
  #endif
}

/*
Description: Puts the ADE9000 from normal mode (PSM0) to sleep mode (PSM3) by makeing PM1 high.
*/
void ADE9000_DeepSleep(void)
{
  digitalWrite(PM1_PIN, HIGH);   //Set PM1 select pin high
  //write PM1 HIGH (activate PSM3)
  //ADE9000_STATUS is ASLEEP
  #ifdef DEBUG
    Serial.println("ADE9000 STATUS: ASLEEP");
    Serial.println();
  #endif
}
