/**********************************
 * File Name    : RTC.cpp         *
 * Version      : 0.1             *
 * Description  : RTC driver      *
 * IDE          : Arduino IDE     *
 * Author       : Collin Haghuis  *
 * Created on   : 13-01-20        *
 * Last Updated : 13-01-20        *
 * Update note  :                 *
 **********************************/

/*** INCLUDES ***/
#include <Arduino.h>
#include "RTC.h"
#include <RTCZero.h>

/*** DEFINITIONS ***/
#define DEBUG

/*** VARIABLES ***/
RTCZero rtc; //Creating an cpp object

/*** FUNCTION PROTOTYPES ***/

/*** FUNCTIONS ***/

void init_RTC(struct TimeAndDate *data)
{
  rtc.begin();

  /* replace this with the time synchronization from the modem */
  data->seconds = SECONDS;
  data->minutes = MINUTES;
  data->hours = HOURS;
  data->day = DAY;
  data->month = MONTH;
  data->year = YEAR;
  /* end replacement */
  
  rtc.setSeconds(data->seconds);
  rtc.setMinutes(data->minutes);
  rtc.setHours(data->hours);
  rtc.setDay(data->day);
  rtc.setMonth(data->month);
  rtc.setYear(data->year);
}

void get_RTC(struct TimeAndDate *data)
{
  data->seconds = rtc.getSeconds();
  data->minutes = rtc.getMinutes();
  data->hours = rtc.getHours();
  data->day = rtc.getDay();
  data->month = rtc.getMonth();
  data->year = rtc.getYear(); 
}

void write_RTC2Debug(struct TimeAndDate *data)
{
  #ifdef DEBUG
    if (data->hours < 10) Serial.print("0");
    Serial.print(data->hours);
    Serial.print(":");
    if (data->minutes < 10) Serial.print("0");
    Serial.print(data->minutes);
    Serial.print(":");
    if (data->seconds < 10) Serial.print("0");
    Serial.print(data->seconds);
    Serial.print("  ");
    
    if (data->day < 10) Serial.print("0");
    Serial.print(data->day);
    Serial.print("/");
    if (data->month < 10) Serial.print("0");
    Serial.print(data->month);
    Serial.print("/");
    if (data->year < 10) Serial.print("0");
    Serial.print(data->year);
    Serial.println();
  #endif
}
