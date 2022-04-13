#ifndef RTC_H
#define RTC_H


#define device_id_write 0xD0
#define device_id_read 0xD1

extern int sec,min,hour,Day,Date,Month,Year;

void RTC_Read_Clock(char read_clock_address);
void RTC_Read_Calendar(char read_calendar_address);

#endif