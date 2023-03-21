/**********************************
 * File Name    : RTC.h           *
 * Version      : 0.1             *
 * Description  : RTC driver      *
 * IDE          : Arduino IDE     *
 * Author       : Collin Haghuis  *
 * Created on   : 13-01-20        *
 * Last Updated : 13-01-20        *
 * Update note  :                 *
 **********************************/

/*** INCLUDES ***/

/*** DEFINITIONS ***/
//Change these values to set the current initial time 
#define SECONDS 0
#define MINUTES 1
#define HOURS 16

//Change these values to set the current initial date
#define DAY 13
#define MONTH 1
#define YEAR 20

/*** VARIABLES ***/

/*** STRUCTURES ***/
struct TimeAndDate
{
  int8_t seconds;
  int8_t minutes;
  int8_t hours;
  int8_t day;
  int8_t month;
  int8_t year;
};

/*** FUNCTION PROTOTYPES ***/
void init_RTC(struct TimeAndDate *data);
void get_RTC(struct TimeAndDate *data);
void write_RTC2Debug(struct TimeAndDate *data);
