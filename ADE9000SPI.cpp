/**************************************
 * File Name    : ADE9000SPI.cpp      *
 * Version      : 0.3                 *
 * Description  : ADE9000 SPI driver  *
 * IDE          : Arduino IDE         *
 * Author       : Collin Haghuis      *
 * Created on   : 13-12-19            *
 * Last Updated : 13-01-20            *
 * Update note  :                     *
 **************************************/

/*** INCLUDES ***/
#include <Arduino.h>
#include <SPI.h>
#include "ADE9000SPI.h"
#include <Wire.h>


/*** DEFINITIONS ***/

/*** VARIABLES ***/

/*** FUNCTION PROTOTYPES ***/

/*** FUNCTIONS ***/

/*
Description: Initializes the arduino SPI port using SPI.h library
Input: SPI speed, chip select pin
Output:-
*/

void SPI_Init(uint32_t SPI_speed, uint8_t chipSelect_Pin)
{
  //PAGINA 37 datasheet ADE9000
  //ADE9000 is always a slave
  //SPI_speed = maximum speed of the ADE9000 is 20MHz
  //15:4 = ADDR
  //3 = R=1/W=0
  //2:0 = Don't care bits
  SPI.begin(); //Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high.
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));    //Setup SPI parameters
  pinMode(chipSelect_Pin, OUTPUT);   //Set Chip select pin as output
  digitalWrite(CS_PIN, HIGH);   //Set Chip select pin high
  
}

/*
Description: Writes 16bit data to a 16 bit register.
Input: Register address, data
Output:-
*/
void SPI_Write_16(uint16_t Address, uint16_t Data)
{
  uint16_t temp_address;

  digitalWrite(CS_PIN, LOW);
  //16bit temp = 12bit address << 4 & FFF0
  //FFF0 is to filter the 12bit address and make the first 4 bits 0/
  temp_address = ((Address << 4) & 0xFFF0); //shift address + R/W + to align with cmd packet
  SPI.transfer16(temp_address); //send address + Write to ADE9000
  SPI.transfer16(Data); //send data
  digitalWrite(CS_PIN, HIGH);
}

/*
Description: Writes 32bit data to a 32 bit register.
Input: Register address, data
Output:-
*/
void SPI_Write_32(uint16_t Address, uint32_t Data)
{
  uint16_t temp_address;
  uint16_t temp_highpacket;
  uint16_t temp_lowpacket;

  temp_highpacket = (Data & 0xFFFF0000)>>16;
  temp_lowpacket = (Data & 0x0000FFFF);

  digitalWrite(CS_PIN, LOW);

  temp_address = ((Address << 4) & 0xFFF0); //shift address  to align with cmd packet
  SPI.transfer16(temp_address);
  SPI.transfer16(temp_highpacket);
  SPI.transfer16(temp_lowpacket);

  digitalWrite(CS_PIN, HIGH); //uitzetten van de SPI verbinding
}

/*
Description: Reads 16bit data from register.
Input: Register address
Output: 16 bit data
*/
uint16_t SPI_Read_16(uint16_t Address)
{
  uint16_t temp_address;
  uint16_t returnData;

  digitalWrite(CS_PIN, LOW);

  temp_address = (((Address << 4) & 0xFFF0)+8); //+8 is 4th bit and sets Read bit high
  SPI.transfer16(temp_address);
  returnData = SPI.transfer16(0);

  digitalWrite(CS_PIN, HIGH);
  return returnData;
}

/*
Description: Reads 32bit data from register.
Input: Register address
Output: 32 bit data
*/
uint32_t SPI_Read_32(uint16_t Address)
{
  uint16_t temp_address;
  uint16_t temp_highpacket;
  uint16_t temp_lowpacket;
  uint32_t returnData;

  digitalWrite(CS_PIN, LOW);

  temp_address = (((Address << 4) & 0xFFF0)+8);
  SPI.transfer16(temp_address);
  temp_highpacket = SPI.transfer16(0);
  temp_lowpacket = SPI.transfer16(0);

  digitalWrite(CS_PIN, HIGH);

  returnData = temp_highpacket << 16;
  returnData = returnData + temp_lowpacket;

  return returnData;
}

/*
Description: Burst reads the content of waveform buffer. This function only works with resampled data. Configure waveform buffer to have Resampled data, and burst enabled (BURST_CHAN=0000 in WFB_CFG Register).
Input: The starting address. Use the starting address of a data set. e.g 0x800, 0x804 etc to avoid data going into incorrect arrays.
       Read_Element_Length is the number of data sets to read. If the starting address is 0x800, the maximum sets to read are 512.
Output: Resampled data returned in structure
*/
/*void SPI_Burst_Read_Resampled_Wfb(uint16_t Address, uint16_t Read_Element_Length, struct ResampledWfbData *ResampledData)
{
  uint16_t temp;
  uint16_t i;

  digitalWrite(chipSelect_Pin, LOW);

  SPI.transfer16(((Address << 4) & 0xFFF0)+8);  //Send the starting address

  //burst read the data upto Read_Length
  for(i=0;i<Read_Element_Length;i++)
    {
      ResampledData->IA_Resampled[i] =  SPI.transfer16(0);
      ResampledData->VA_Resampled[i] =  SPI.transfer16(0);
      ResampledData->IB_Resampled[i] =  SPI.transfer16(0);
      ResampledData->VB_Resampled[i] =  SPI.transfer16(0);
      ResampledData->IC_Resampled[i] =  SPI.transfer16(0);
      ResampledData->VC_Resampled[i] =  SPI.transfer16(0);
      ResampledData->IN_Resampled[i] =  SPI.transfer16(0);
    }
  digitalWrite(chipSelect_Pin, HIGH);
}*/
