/************************************
 * File Name    : ADE9000.h         *
 * Version      : 0.8               *
 * Description  : ADE9000 driver    *
 * IDE          : Arduino IDE       *
 * Author       : Collin Haghuis    *
 * Created on   : 13-12-19          *
 * Last Updated : 13-01-20          *
 * Update note  :                   *
 ************************************/
#ifndef ADE9000_h
#define ADE9000_h

#ifdef __cplusplus
extern "C" {
#endif

/*** INCLUDES ***/
#include <Arduino.h>
#include "ade9000regmap.h"
#include "math.h"

/*** DEFINITIONS ***/
/* register configurations */
#define ADE9000_PGA_GAIN    0x00FF      //(P85) PGA of all current channels = 4X. change PGA_GAIN_CUR_CHAN correspondingly.
#define ADE9000_CONFIG0     0x00000000  //(P65) Integrators disabled
#define ADE9000_CONFIG1     0x0002      //(P78) CF3/ZX pin outputs Zero crossing
#define ADE9000_CONFIG2     0x0C00      //(P83) Default High pass corner frequency of 1.25Hz
#define ADE9000_CONFIG3     0x0000      //(P80) Peak and overcurrent detection disabled
#define ADE9000_ACCMODE     0x0000      //(P80) 50Hz operation, 3P4W Wye configuration, signed accumulation
#define ADE9000_TEMP_CFG    0x000C      //(P85) Temperature sensor enabled
#define ADE9000_ZX_LP_SEL   0x001E      //(P81) Line period and zero crossing obtained from combined signals VA,VB and VC
#define ADE9000_MASK0       0x00000001  //(P71) Enable EGYRDY interrupt
#define ADE9000_MASK1       0x00000000  //(P73) MASK1 interrupts disabled
#define ADE9000_EVENT_MASK  0x00000000  //(P74) Events disabled
#define ADE9000_VLEVEL      0x0022EA28  //(P75) Assuming Vnom=1/2 of full scale.
#define ADE9000_DICOEFF     0x00000000  //(P56) Set DICOEFF= 0xFFFFE000 when integrator is enabled

/* constant Definitions */
#define ADE90xx_FDSP 8000         //ADE9000 FDSP: 8000sps
#define ADE9000_RUN_ON 0x0001     //DSP ON

/* energy Accumulation Settings */
#define ADE9000_EP_CFG 0x0011     //(P83) Enable energy accumulation, accumulate samples at 8ksps
                                  //latch energy accumulation after EGYRDY
                                  //If accumulation is changed to half line cycle mode, change EGY_TIME
#define ADE9000_EGY_TIME 0x1F3F   //Accumulate 8000 samples

/* waveform buffer Settings */
#define ADE9000_WFB_CFG 0x1000      //(P82) Neutral current samples enabled, Resampled data enabled
                                    //Burst all channels
#define WFB_ELEMENT_ARRAY_SIZE 512  //size of buffer to read. 512 Max.Each element IA,VA...IN has max 512 points
                                    //[Size of waveform buffer/number of sample sets = 2048/4 = 512]
                                    //(Refer ADE9000 technical reference manual for more details)

//Full scale Codes referred from ADE9000 datasheet (P13).
//Respective digital codes are produced when ADC inputs are at full scale.
//Do not Change.
#define ADE9000_RMS_FULL_SCALE_CODES  52702092
#define ADE9000_WATT_FULL_SCALE_CODES 20694066
#define ADE9000_RESAMPLED_FULL_SCALE_CODES  18196
#define ADE9000_PCF_FULL_SCALE_CODES  74532013

/* current sensor and current channel calibration settings */
#define I_NOM 10 //I nominal value that is used for calibration
#define CTF 0.0000225 //current sensor transfer function
#define PGA_GAIN_CUR_CHAN 4 //extracted from the ADE9000_PGA_GAIN. if ADE9000_PGA_GAIN = 0x00FF, CUR_PGA_GAIN = 4
#define IRMS_CC (float) (1 / (CTF * PGA_GAIN_CUR_CHAN * sqrt(2) * ADE9000_RMS_FULL_SCALE_CODES))
#define IRMS_EXP (int) (I_NOM * CTF * PGA_GAIN_CUR_CHAN * sqrt(2) * ADE9000_RMS_FULL_SCALE_CODES)
#define DEVIATION 4 // deviation that is allowed when calculating the I_NOM_MIN and I_NOM_MAX
#define I_NOM_MIN (int) ((I_NOM - DEVIATION) / IRMS_CC)
#define I_NOM_MAX (int) ((I_NOM + DEVIATION) / IRMS_CC)

/* pin definitions */
#define PM1_PIN 4
#define IRQ1_PIN 3
#define IRQ0_PIN 2

/*** STRUCTURES ***/
struct ResampledWfbData
{
  int16_t VA_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t IA_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t VB_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t IB_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t VC_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t IC_Resampled[WFB_ELEMENT_ARRAY_SIZE];
  int16_t IN_Resampled[WFB_ELEMENT_ARRAY_SIZE];
};

struct ActivePowerRegs
{
  int32_t ActivePowerReg_A;
  int32_t ActivePowerReg_B;
  int32_t ActivePowerReg_C;
};

struct ReactivePowerRegs
{
  int32_t ReactivePowerReg_A;
  int32_t ReactivePowerReg_B;
  int32_t ReactivePowerReg_C;
};

struct ApparentPowerRegs
{
  int32_t ApparentPowerReg_A;
  int32_t ApparentPowerReg_B;
  int32_t ApparentPowerReg_C;
};

struct VoltageRMSRegs
{
  int32_t VoltageRMSReg_A;
  int32_t VoltageRMSReg_B;
  int32_t VoltageRMSReg_C;
};
struct CurrentRMSRegs
{
  int32_t CurrentRMSReg_A;
  int32_t CurrentRMSReg_B;
  int32_t CurrentRMSReg_C;
  int32_t CurrentRMSReg_N;
};

struct FundActivePowerRegs
{
  int32_t FundActivePowerReg_A;
  int32_t FundActivePowerReg_B;
  int32_t FundActivePowerReg_C;
};

struct FundReactivePowerRegs
{
  int32_t FundReactivePowerReg_A;
  int32_t FundReactivePowerReg_B;
  int32_t FundReactivePowerReg_C;
};

struct FundApparentPowerRegs
{
  int32_t FundApparentPowerReg_A;
  int32_t FundApparentPowerReg_B;
  int32_t FundApparentPowerReg_C;
};

struct FundVoltageRMSRegs
{
  int32_t FundVoltageRMSReg_A;
  int32_t FundVoltageRMSReg_B;
  int32_t FundVoltageRMSReg_C;
};

struct FundCurrentRMSRegs
{
  int32_t FundCurrentRMSReg_A;
  int32_t FundCurrentRMSReg_B;
  int32_t FundCurrentRMSReg_C;
  //Fundamental neutral RMS is not calculated
};
struct HalfVoltageRMSRegs
{
  int32_t HalfVoltageRMSReg_A;
  int32_t HalfVoltageRMSReg_B;
  int32_t HalfVoltageRMSReg_C;
};

struct HalfCurrentRMSRegs
{
  int32_t HalfCurrentRMSReg_A;
  int32_t HalfCurrentRMSReg_B;
  int32_t HalfCurrentRMSReg_C;
  int32_t HalfCurrentRMSReg_N;
};

struct Ten12VoltageRMSRegs
{
  int32_t Ten12VoltageRMSReg_A;
  int32_t Ten12VoltageRMSReg_B;
  int32_t Ten12VoltageRMSReg_C;
};
struct Ten12CurrentRMSRegs
{
  int32_t Ten12CurrentRMSReg_A;
  int32_t Ten12CurrentRMSReg_B;
  int32_t Ten12CurrentRMSReg_C;
  int32_t Ten12CurrentRMSReg_N;
};

struct VoltageTHDRegs
{
  int32_t VoltageTHDReg_A;
  int32_t VoltageTHDReg_B;
  int32_t VoltageTHDReg_C;
  float VoltageTHDValue_A;
  float VoltageTHDValue_B;
  float VoltageTHDValue_C;
};

struct CurrentTHDRegs
{
  int32_t CurrentTHDReg_A;
  int32_t CurrentTHDReg_B;
  int32_t CurrentTHDReg_C;
  float CurrentTHDValue_A;
  float CurrentTHDValue_B;
  float CurrentTHDValue_C;
};

struct PowerFactorRegs
{
  int32_t PowerFactorReg_A;
  int32_t PowerFactorReg_B;
  int32_t PowerFactorReg_C;
  float PowerFactorValue_A;
  float PowerFactorValue_B;
  float PowerFactorValue_C;
};

struct PeriodRegs
{
  int32_t PeriodReg_A;
  int32_t PeriodReg_B;
  int32_t PeriodReg_C;
  float FrequencyValue_A;
  float FrequencyValue_B;
  float FrequencyValue_C;
};

struct AngleRegs
{
  int16_t AngleReg_VA_VB;
  int16_t AngleReg_VB_VC;
  int16_t AngleReg_VA_VC;
  int16_t AngleReg_VA_IA;
  int16_t AngleReg_VB_IB;
  int16_t AngleReg_VC_IC;
  int16_t AngleReg_IA_IB;
  int16_t AngleReg_IB_IC;
  int16_t AngleReg_IA_IC;
  float AngleValue_VA_VB;
  float AngleValue_VB_VC;
  float AngleValue_VA_VC;
  float AngleValue_VA_IA;
  float AngleValue_VB_IB;
  float AngleValue_VC_IC;
  float AngleValue_IA_IB;
  float AngleValue_IB_IC;
  float AngleValue_IA_IC;
};

struct TemperatureRegnValue
{
  int16_t Temperature_Reg;
  float Temperature;
};

struct CurrentRMSVals
{
  float CurrentRMSVal_A;
  float CurrentRMSVal_B;
  float CurrentRMSVal_C;
  float CurrentRMSVal_N;
};

struct CurrentGainRegs
{
  int32_t CurrentGainReg_A;
  int32_t CurrentGainReg_B;
  int32_t CurrentGainReg_C;
  int32_t CurrentGainReg_N;
};

/*** FUNCTION PROTOTYPES ***/
void ADE9000_Init(void);
void ADE9000_ReadActivePowerRegs(struct ActivePowerRegs *data);
void ADE9000_ReadReactivePowerRegs(struct ReactivePowerRegs *data);
void ADE9000_ReadApparentPowerRegs(struct ApparentPowerRegs *data);
void ADE9000_ReadVoltageRMSRegs(struct VoltageRMSRegs *data);
void ADE9000_ReadCurrentRMSRegs(struct CurrentRMSRegs *data);
void ADE9000_ReadFundActivePowerRegs(struct FundActivePowerRegs *data);
void ADE9000_ReadFundReactivePowerRegs(struct FundReactivePowerRegs *data);
void ADE9000_ReadFundApparentPowerRegs(struct FundApparentPowerRegs *data);
void ADE9000_ReadFundVoltageRMSRegs(struct FundVoltageRMSRegs *data);
void ADE9000_ReadFundCurrentRMSRegs(struct FundCurrentRMSRegs *data);
void ADE9000_ReadHalfVoltageRMSRegs(struct HalfVoltageRMSRegs *data);
void ADE9000_ReadHalfCurrentRMSRegs(struct HalfCurrentRMSRegs *data);
void ADE9000_ReadTen12VoltageRMSRegs(struct Ten12VoltageRMSRegs *data);
void ADE9000_ReadTen12CurrentRMSRegs(struct Ten12CurrentRMSRegs *data);
void ADE9000_ReadVoltageTHDRegsnValues(struct VoltageTHDRegs *data);
void ADE9000_ReadCurrentTHDRegsnValues(struct CurrentTHDRegs *data);
void ADE9000_ReadPowerFactorRegsnValues(struct PowerFactorRegs *data);
void ADE9000_ReadPeriodRegsnValues(struct PeriodRegs *data);
void ADE9000_ReadAngleRegsnValues(struct AngleRegs *data);
void ADE9000_ReadTempRegnValue(struct TemperatureRegnValue *data);
void ADE9000_ReadCurrentRMSValues(struct CurrentRMSVals *data);
void ADE9000_WriteRMSValues2Debug(struct CurrentRMSVals *data);
void ADE9000_ComputeCurrentGainRegs(struct CurrentGainRegs *data1, struct CurrentRMSRegs *data2);
void ADE9000_NullingCurrentGainRegs(struct CurrentGainRegs *data);
void ADE9000_WriteCurrentGainRegs(struct CurrentGainRegs *data);
void ADE9000_CurrentGainCalibration(struct CurrentGainRegs *gain);
void ADE9000_NullingZeroGainChannels(struct CurrentGainRegs *data1, struct CurrentRMSVals *data2);
void ADE9000_PinInit(void);
void ADE9000_WakeUp(void);
void ADE9000_DeepSleep(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ADE9000_ADE9000_H_ */
