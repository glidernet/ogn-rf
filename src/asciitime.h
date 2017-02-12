/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __ASCIITIME_H__
#define __ASCIITIME_H__
/*
int SplitTime(int &Hour, int &Min, int &Sec, double &Frac, double Time)
{ int UnixTime = (int)floor(Time); Frac=Time-UnixTime;
  const int SecondsPerDay = 24*60*60;
  int Day = UnixTime/SecondsPerDay;
  UnixTime -= Day*SecondsPerDay;
  Hour = UnixTime/3600; UnixTime-=Hour*3600;
  Min  = UnixTime/60;   UnixTime-=Min*60;
  Sec  = UnixTime;
  return Day; }

void AsciiTime_HHMMSSFFF(char *String, double Time)
{ int Hour, Min, Sec; double Frac;
  SplitTime(Hour, Min, Sec, Frac, Time);
  sprintf(String, "%02d:%02d:%02d.%03d", Hour, Min, Sec, (int)floor(Frac*1000)); }

void AsciiTime_DDDDDHHMMSSFFF(char *String, double Time)
{ int Hour, Min, Sec; double Frac;
  int Day=SplitTime(Hour, Min, Sec, Frac, Time);
  sprintf(String, "%05d:%02d:%02d:%02d.%03d", Day, Hour, Min, Sec, (int)floor(Frac*1000)); }

int Time_HHMMSS(time_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  return 10000*Hour + 100*Min + Sec; }

int Format_HHMMSS(char *Str, time_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  uint32_t HourH=Hour/10; Str[0]='0'+HourH; Str[1]='0'+(Hour-10*HourH);
  uint32_t  MinH=Min /10; Str[2]='0'+MinH;  Str[3]='0'+(Min -10*MinH);
  uint32_t  SecH=Sec /10; Str[4]='0'+SecH;  Str[5]='0'+(Sec -10*SecH);
  return 6; }
*/
#endif
