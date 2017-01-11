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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#if !defined(__MACH__)   // OSX and Cygwin have no ntp_ calls
#include <sys/sysinfo.h>
#if !defined(__CYGWIN__)
#include <sys/time.h>
#include <sys/timex.h>
#endif
#endif

// ===================================================================================================

int getMemoryUsage(int &Total, int &Free) // get total and free RAM [kB]
{ FILE *File=fopen("/proc/meminfo","rt"); if(File==0) return -1;
  char Line[64];
  if(fgets(Line, 64, File)==0) goto Error;
  if(memcmp(Line, "MemTotal:", 9)!=0) goto Error;
  Total = atoi(Line+10);
  if(fgets(Line, 64, File)==0) goto Error;
  if(memcmp(Line, "MemFree:", 8)!=0) goto Error;
  Free = atoi(Line+10);
  fclose(File); return 0;
Error:
  fclose(File); return -1; }
template <class Float>
 int getMemoryUsage(Float &Used) // get used RAM as a ratio
{ int Total, Free;
  int Error = getMemoryUsage(Total, Free); if(Error<0) return Error;
  Used = (Float)(Total-Free)/Total; return 0; }

// ===================================================================================================

int getCpuUsage(int &DiffTotal, int &DiffUser, int &DiffSystem) // get CPU usage
{ static int RefTotal=0, RefUser=0, RefSystem=0;
  FILE *File=fopen("/proc/stat","rt"); if(File==0) return -1;
  char Line[64];
  if(fgets(Line, 64, File)==0) goto Error;
  if(memcmp(Line, "cpu ", 4)!=0) goto Error;
  int Total, User, Nice, System, Idle;
  if(sscanf(Line+4, "%d %d %d %d", &User, &Nice, &System, &Idle)!=4) goto Error;
  User+=Nice;
  Total = User+System+Idle;
  DiffTotal=Total-RefTotal; DiffUser=User-RefUser; DiffSystem=System-RefSystem;
  RefTotal=Total; RefUser=User; RefSystem=System;
  fclose(File); return 0;
Error:
  fclose(File); return -1; }

int getCpuSerial(long long int &SerialNumber) // get the CPU serial number
{ FILE *File=fopen("/proc/cpuinfo","rt"); if(File==0) return -1;
  char Line[128];
  for( ; ; )
  { if(fgets(Line, 128, File)==0) goto Error;
    if(memcmp(Line, "Serial", 6)==0) break;
  }
  if(sscanf(Line+6, " : %llx", &SerialNumber)!=1) goto Error;
  fclose(File); return 0;
Error:
  fclose(File); return -1; }

int getCpuUsage(void)                       // get CPU usage - initialize
{ int DiffTotal, DiffUser, DiffSystem; return getCpuUsage(DiffTotal, DiffUser, DiffSystem); }

int getCpuUsage(float &User, float &System) // get CPU usage as ratios
{ int DiffTotal, DiffUser, DiffSystem;
  int Error=getCpuUsage(DiffTotal, DiffUser, DiffSystem); if(Error<0) return Error;
  if(DiffTotal<=0) return -1;
  User = (float)DiffUser/DiffTotal; System = (float)DiffSystem/DiffTotal; return 0; }

// ===================================================================================================

template <class Type>
 int getSysValue(Type &Value, const char *Name, const char *Format="%d")
{ FILE *File=fopen(Name, "rt"); if(File==0) return -1;
  int Ret=fscanf(File, Format, &Value);
  fclose(File); return (Ret==1) ? 1:-2; }

template <class Float>
 int getCpuTemperature(Float &Temperature)
{ int IntValue;
  if(getSysValue(IntValue, "/sys/class/thermal/thermal_zone0/temp", "%d")<0)
  { if(getSysValue(IntValue, "/sys/class/hwmon/hwmon0/device/temp1_input", "%d")<0) return -1; }
  Temperature=0.001*IntValue; return 0; }

template <class Float>
 int getAcSupplyVoltage(Float &Voltage)
{ int IntValue;
  if(getSysValue(IntValue, "/sys/class/power_supply/ac/voltage_now", "%d")<0) return -1;
  Voltage = 1e-6*IntValue; return 0; }

template <class Float>
 int getAcSupplyCurrent(Float &Current)
{ int IntValue;
  if(getSysValue(IntValue, "/sys/class/power_supply/ac/current_now", "%d")<0) return -1;
  Current = 1e-6*IntValue; return 0; }

template <class Float>
 int getUsbSupplyVoltage(Float &Voltage)
{ int IntValue;
  if(getSysValue(IntValue, "/sys/class/power_supply/usb/voltage_now", "%d")<0) return -1;
  Voltage = 1e-6*IntValue; return 0; }

template <class Float>
 int getUsbSupplyCurrent(Float &Current)
{ int IntValue;
  if(getSysValue(IntValue, "/sys/class/power_supply/usb/current_now", "%d")<0) return -1;
  Current = 1e-6*IntValue; return 0; }

template <class Float>
 int getSupplyVoltage(Float &Voltage)
{ Float UsbVoltage = 0; int UsbErr=getUsbSupplyVoltage(UsbVoltage);
  Float  AcVoltage = 0; int  AcErr= getAcSupplyVoltage( AcVoltage);
  if(UsbErr<0) Voltage= AcVoltage; return AcErr;
  if( AcErr<0) Voltage=UsbVoltage; return UsbErr;
  if(UsbVoltage>AcVoltage) Voltage=UsbVoltage;
                    else   Voltage= AcVoltage;
  return 0; }

template <class Float>
 int getSupplyCurrent(Float &Current)
{ Float UsbCurrent = 0; int UsbErr=getUsbSupplyCurrent(UsbCurrent);
  Float  AcCurrent = 0; int  AcErr= getAcSupplyCurrent( AcCurrent);
  if(UsbErr<0) Current= AcCurrent; return AcErr;
  if( AcErr<0) Current=UsbCurrent; return UsbErr;
  if(UsbCurrent>AcCurrent) Current=UsbCurrent;
                    else   Current= AcCurrent;
  return 0; }


/*
template <class Float>
 int getCpuTemperature(Float &Temperature)
{ FILE *File=fopen("/sys/class/thermal/thermal_zone0/temp","rt"); if(File==0) return -1;
  char Line[16];
  if(fgets(Line, 16, File)==0) goto Error;
  Temperature = 0.001*atoi(Line);
  fclose(File); return 0;
Error:
  fclose(File); return -1; }
*/
// ===================================================================================================

#if defined(__MACH__) || defined(__CYGWIN__) // for OSX we cannot read NTP status

template <class Float>
 int getNTP(Float &Time, Float &EstError, Float &RefFreqCorr) { return -1; }

#else
template <class Float>
 int getNTP(Float &Time, Float &EstError, Float &RefFreqCorr)
{ struct timex TimeX; TimeX.modes=0;
  int Error=ntp_adjtime(&TimeX);
  if(TimeX.status&STA_NANO)
    Time = TimeX.time.tv_sec + 1e-9*TimeX.time.tv_usec;
  else
    Time = TimeX.time.tv_sec + 1e-6*TimeX.time.tv_usec;
  EstError = 1e-6*TimeX.esterror;
  RefFreqCorr = (Float)TimeX.freq/(1<<16);
  return Error; }

#endif // of __MACH__

// ===================================================================================================
