/**************************************
 * File Name    : ADE9000SPI.h        *
 * Version      : 0.3                 *
 * Description  : ADE9000 SPI driver  *
 * IDE          : Arduino IDE         *
 * Author       : Collin Haghuis      *
 * Created on   : 13-12-19            *
 * Last Updated : 13-01-20            *
 * Update note  :                     *
 **************************************/

#ifndef ADE9000SPI_h

#define ADE9000SPI_h

#ifdef __cplusplus
extern "C" {
#endif

/*** INCLUDES ***/
#include <Arduino.h>

/*** DEFINITIONS ***/
#define CS_PIN 1 //F10 or can i use SPI2_CS0 //chip select pin for the ADE9000 is PTB20
#define SPI_SPEED 5000000

/*** GLOBAL VARIABLES ***/

/*** STRUCTURES ***/

/*** FUNCTION PROTOTYPES ***/
void SPI_Init(uint32_t SPI_speed, uint8_t chipSelect_Pin);
void SPI_Write_16(uint16_t Address, uint16_t Data ); //addres of the register, data that needs to be sent to that register
void SPI_Write_32(uint16_t Address, uint32_t Data );
uint16_t SPI_Read_16(uint16_t Address);
uint32_t SPI_Read_32(uint16_t Address);
void SPI_Burst_Read_Resampled_Wfb(uint16_t Address, uint16_t Read_Element_Length, struct ResampledWfbData *ResampledData);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SPI_SPI_H_ */
