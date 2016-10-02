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

#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <time.h>
#include <sys/time.h>

#include <math.h>

// #include "alloc.h"
#include "asciitime.h"
#include "thread.h"

#include "buffer.h"

// =================================================================================

#include <rtl-sdr.h>

class RTLSDR
{ public:
   MutEx         Lock;           // for multi-threading

   uint32_t      DeviceIndex;    // RTL dongle index
   rtlsdr_dev_t *Device;         // RTL dongle handle
   int           Gains;          // number of possible gain settings
   int           Gain[64];       // [0.1 dB] list of possible gain settings

   int           Bandwidths;
   int           Bandwidth[16];

   int           Stages;
   int           StageGains[8];
   char          StageName[8][32];
   int           StageGain[8][32];

   uint64_t      BytesRead;      // Counts number of bytes read (1 sample = 2 bytes: I/Q)
   int         (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex);
   void         *CallbackContext;

#ifndef __MACH__ // _POSIX_TIMERS
   clockid_t     RefClock;           // CLOCK_REALTIME, CLOCK_MONOTONIC or CLOCK_MONOTONIC_RAW
#endif

   double        SampleTime;         // [sec] time when a batch of samples starts
   double        StartTime;          // [sec] time when acquisition started
   double        AverPeriod;         // [sec] averaging period for TimeRef, TimeRef_DMS and SamplePeriod
   double        SamplePeriod;       // [sec] time per sample
   double        PrevTime;           // [sec]
   double        SampleTime_DMS;     // [sec^2] mean square variation of SampleTime

  public:
   RTLSDR()
   { DeviceIndex=0; Device=0; Callback=0; CallbackContext=0; // Gain=0;
     AverPeriod=100.0;
#ifndef __MACH__ // _POSIX_TIMERS
     RefClock=CLOCK_REALTIME;
#endif
   }

  ~RTLSDR()
   { Close(); }

   bool isOpen(void) const { return Device!=0; }

   void Close(void)
   { if(Device)
     { // printf("RTLSDR::Close() => %3.1f MB read, %3.1f samples/sec\n", BytesRead/(1024*1024.0), 1.0/SamplePeriod);
       rtlsdr_cancel_async(Device); rtlsdr_close(Device); }
     // free(Gain); Gain=0;
     Gains=0; Stages=0; Bandwidths=0;
     // free(SampleTimePipe); free(SampleIdxPipe); PipeSize=0; SampleTimePipe=0; SampleIdxPipe=0;
     Device=0; Callback=0; }

   static int  getNumberOfDevices(void)            { return rtlsdr_get_device_count(); }            // number of connected devices (USB RTL dongles)

   static int  getDeviceUsbStrings(uint32_t DeviceIndex, char *Manufacturer, char *Product, char *Serial)
             { return rtlsdr_get_device_usb_strings(DeviceIndex, Manufacturer, Product, Serial); }  // USB description strings
   int         getUsbStrings(char *Manufacturer, char *Product, char *Serial)
             { return rtlsdr_get_usb_strings(Device, Manufacturer, Product, Serial); }  // USB description strings
             // { return rtlsdr_get_device_usb_strings(DeviceIndex, Manufacturer, Product, Serial); }  // USB description strings

   static const char *getDeviceName(uint32_t DeviceIndex) { return rtlsdr_get_device_name(DeviceIndex); }  // name of given device
          const char *getDeviceName(void)                 { return rtlsdr_get_device_name(DeviceIndex); }  // name of this device open by this object

   int     getTunerType(void) { return rtlsdr_get_tuner_type(Device); }
   const char *getTunerTypeName(void)
   { const char *TunerType[7] = { "UNKNOWN", "E4000", "FC0012", "FC0013", "FC2580", "R820T", "R828D" } ;
     int Type=getTunerType(); if((Type<0)&&(Type>=7)) Type=0; return TunerType[Type]; }

   int     getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq) { return rtlsdr_get_xtal_freq(Device, &RtlFreq, &TunerFreq); }
   int     setXtalFreq(uint32_t  RtlFreq, uint32_t  TunerFreq) { return rtlsdr_set_xtal_freq(Device,  RtlFreq,  TunerFreq); }

   int     ReadEEPROM (uint8_t *Data, uint8_t Offset, uint16_t Size) { return rtlsdr_read_eeprom (Device, Data, Offset, Size); } // read  the EEPROM
   int     WriteEEPROM(uint8_t *Data, uint8_t Offset, uint16_t Size) { return rtlsdr_write_eeprom(Device, Data, Offset, Size); } // write the EEPROM

   int     setOffsetTuning(int ON=1) { return rtlsdr_set_offset_tuning(Device, ON); }
   int     getOffsetTuning(void)     { return rtlsdr_get_offset_tuning(Device);     }

   int     setCenterFreq(uint32_t Frequency)    { return rtlsdr_set_center_freq(Device, Frequency); } // [Hz]
  uint32_t getCenterFreq(void)                  { return rtlsdr_get_center_freq(Device); } // (fast call)

   int     setFreqCorrection(int PPM)           { return rtlsdr_set_freq_correction(Device, PPM); } // [PPM] (Part-Per-Million)
   int     getFreqCorrection(void)              { return rtlsdr_get_freq_correction(Device); } // (fast call)

#ifdef NEW_RTLSDR_LIB
   int setTunerBandwidth(int Bandwidth) { return rtlsdr_set_tuner_bandwidth(Device, Bandwidth); }    // [Hz] a new (advanced) function
   // int getTunerBandwidth(void)     { int Bandwidth; return rtlsdr_get_tuner_bandwidth(Device, &Bandwidth); return Bandwidth; }
   int getTunerBandwidths(int *Bandwidth=0) { return rtlsdr_get_tuner_bandwidths(Device, Bandwidth); }
#endif

   int getTunerGains(int *Gain=0)           { return rtlsdr_get_tuner_gains(Device, Gain); }
#ifdef NEW_RTLSDR_LIB
   int getTunerStageGains(int Stage, int32_t *Gain, char *Description=0) { return rtlsdr_get_tuner_stage_gains(Device, Stage, Gain, Description); }
   int setTunerStageGain(int Stage, int Gain) { return rtlsdr_set_tuner_stage_gain(Device, Stage, Gain); }
#endif
   int setTunerGain(int Gain) { return rtlsdr_set_tuner_gain(Device, Gain); }    // [0.1 dB]    set tuner gain when in manual mode
   int getTunerGain(void)     { return rtlsdr_get_tuner_gain(Device); }

   // note: new gain modes are possible with the more advanced drivers: 2=Linearity, 3=Sensitivity
   int setTunerGainMode(int Manual=1) { return rtlsdr_set_tuner_gain_mode(Device, Manual); }  // set radio-tuner gain mode: manual or automatic
   int setTunerGainManual(int Manual=1) { return setTunerGainMode(Manual); }                  // set manual mode
   int setTunerGainAuto(void) { return setTunerGainManual(0); }                               // set automatic mode

   int setTestMode(int Test=1) { return rtlsdr_set_testmode(Device, Test); }  // Enable/Disable test mode - a counter is send, not real data
   int ResetBuffer(void) { return rtlsdr_reset_buffer(Device); }              // obligatory, the docs say, before you start reading

   double getTime(void) const                                                 // read the system time at this very moment
#ifndef __MACH__ // _POSIX_TIMERS
   { struct timespec now; clock_gettime(RefClock, &now); return now.tv_sec + 1e-9*now.tv_nsec; }
#else                                                                         // for OSX, there is no clock_gettime()
   { struct timeval now; gettimeofday(&now, 0); return now.tv_sec + 1e-6*now.tv_usec; }
#endif

    int     setSampleRate(uint32_t SampleRate)  { SamplePeriod = 1.0/SampleRate; SampleTime_DMS=0.0001*0.0001;
                                                  return rtlsdr_set_sample_rate(Device, SampleRate); } // [samples-per-second]
   uint32_t getSampleRate(void)                 { return rtlsdr_get_sample_rate(Device); }

   int getDeviceIndexBySerial(const char *Serial) { return rtlsdr_get_index_by_serial(Serial); }

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868000000, uint32_t SampleRate=2048000) // open given device (by the index)
   { Close();

     this->DeviceIndex=DeviceIndex;
     if(rtlsdr_open(&Device, DeviceIndex)<0)                                                   // open the RTLSDR device
     { printf("Cannot open device #%d\n", DeviceIndex); Device=0; return -1; }
     if(setCenterFreq(Frequency)<0)                                                            // set the desired frequency
     { printf("Cannot set the frequency %d for device #%d\n", Frequency, DeviceIndex); }
     if(setSampleRate(SampleRate)<0)                                                           // set the desired sample rate
     { printf("Cannot set the sample rate %d for device #%d\n", SampleRate, DeviceIndex); }
     printf("RTLSDR::Open(%d,%d,%d) => %s, %8.3f MHz, %5.3f Msps\n",
            DeviceIndex, Frequency, SampleRate, getDeviceName(), 1e-6*getCenterFreq(), 1e-6*getSampleRate());

     Gains=getTunerGains(Gain);                                                  // get list of possible tuner gains
#ifdef NEW_RTLSDR_LIB
     for(Stages=0; Stages<8; Stages++)
     { StageGains[Stages]=getTunerStageGains(Stages, StageGain[Stages], StageName[Stages]);
       if(StageGains[Stages]<=0) break; }
#endif
     PrintGains();

#ifdef NEW_RTLSDR_LIB
     Bandwidths=getTunerBandwidths(Bandwidth);
     PrintBandwidths();
#endif

     if(ResetBuffer()<0)                                                                        // reset the buffers (after the manual...)
     { printf("Cannot reset buffer for device #%d\n", DeviceIndex); }
     return 1; }

   void PrintGains(void) const
   {
#ifdef NEW_RTLSDR_LIB
     for(int Stage=0; Stage<Stages; Stage++)
     { printf("RTLSDR::%s[%d] =", StageName[Stage], StageGains[Stage]);
       for(int Idx=0; Idx<StageGains[Stage]; Idx++) printf(" %+5.1f", 0.1*StageGain[Stage][Idx]); printf(" [dB]\n"); }
#endif
     printf("RTLSDR::Gain[%d] =", Gains); for(int Idx=0; Idx<Gains; Idx++) printf(" %+5.1f", 0.1*Gain[Idx]); printf(" [dB]\n"); }

   void PrintBandwidths(void) const
   { printf("RTLSDR::Bandwidth[%d] =", Bandwidths); for(int Idx=0; Idx<Bandwidths; Idx++) printf(" %5.3f", 1e-6*Bandwidth[Idx]); printf(" [MHz]\n"); }

   double SampleTimeJitter(void) { return sqrt(SampleTime_DMS); }

   static void StaticCallback(unsigned char *Buffer, uint32_t Len, void *Contex)          // callback that receives the data
   { RTLSDR *This = (RTLSDR *)Contex; return This->ClassCallback(Buffer, Len); }          // "This" points now to this class instance

   void ClassCallback(unsigned char *Buffer, uint32_t Len)                                // callback but already in this class instance
   { Lock.Lock();
     int Samples = Len/2;                                                                 // number of samples is half the buffer size
     BytesRead+=Len;                                                                      // count number of bytes read
     double ReadTime=getTime();                                                           // read the time at this moment
/*
     uint32_t PrevSampleIdx=SampleIdxPipe[PipeWrite];                                     // previous SampleIdx
     PipeWrite++; if(PipeWrite>=PipeSize) PipeWrite=0;                                    // advance pipe write pointer
     double FirstSampleTime = SampleTimePipe[PipeRead];
     uint32_t FirstSampleIdx  = SampleIdxPipe[PipeRead];
     if(PipeWrite==PipeRead) { PipeRead++; if(PipeRead>=PipeSize) PipeRead=0; }
     SampleTimePipe[PipeWrite]=ReadTime;                                                  // ReadTime -> Pipe
     SampleIdxPipe[PipeWrite]=PrevSampleIdx+Samples;                                      // next SampleIdx -> Pipe
     double   SampleTimeDiff = ReadTime-FirstSampleTime;
     uint32_t SampleIdxDiff  = (PrevSampleIdx+Samples)-FirstSampleIdx;;
*/
     double AcqTime = Samples * SamplePeriod;                                             // time it took to acquire these samples
     double AverWeight = AcqTime / AverPeriod; // ratio: acquisition period : averaging period

     int Ret=0;
     if(Callback)
     { Ret=(*(Callback))(Buffer, Samples,                                 // buffer, number of samples
                         SampleTime - Samples*SamplePeriod, SamplePeriod, // SampleTime = time of the first sample, SamplePeriod = time period of one sample
                         CallbackContext);
     }
     if(Ret) CancelAsync();                                               // call the user callback, if it returns non-zero, then stop data acquisition

     SampleTime += Samples * SamplePeriod;            // increment predicted time for this batch
     double TimeDiff = ReadTime - SampleTime;         // difference: time read now versus predicted time
     SampleTime += 0.125*TimeDiff;                    // follow the ReadTime with weight 1/8 (a bit arbitrary...)
     SampleTime_DMS += AverWeight * (TimeDiff*TimeDiff - SampleTime_DMS); // integrate the difference RMS

     double PeriodDiff = (ReadTime - PrevTime) - AcqTime; // difference: measured time period to acquire this batch versus predicted time period
     SamplePeriod += AverWeight*(PeriodDiff/Samples);
     PrevTime = ReadTime;

     // printf("%14.3f (%+7.3f:%+7.3f ms): RTLSDR::Callback( , %d, ) => %10.1f (%10.1f) samples/sec, %6.3f ms\r",
     //        ReadTime, 1e3*TimeDiff, 1e3*PeriodDiff, Len, 1.0/SamplePeriod, SampleIdxDiff/SampleTimeDiff, 1e3*SampleTimeJitter() );
/*
     char Time[24]; AsciiTime_DDDDDHHMMSSFFF(Time, ReadTime);
     printf("%s (%+7.3f:%+7.3f ms): RTLSDR::Callback( , %d, ) => %10.1f samples/sec, %6.3f ms\r",
            Time, 1e3*TimeDiff, 1e3*PeriodDiff, Len, 1.0/SamplePeriod, 1e3*SampleTimeJitter() );
     fflush(stdout);
*/
     Lock.Unlock(); }

   // read in async. mode, call Callback() for the data being received, block, wait and return when Callback() returns non-zero
   int ReadAsync(int (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex)=0, void *Contex=0,
                 int Buffers=0, int BlockSize=0)
   { this->Callback = Callback; StartTime=SampleTime=PrevTime=getTime();
     this->CallbackContext = Contex;
     // SampleTimePipe[0]=SampleTime; SampleIdxPipe[0]=0; PipeWrite=0; PipeRead=0;
     return rtlsdr_read_async(Device, StaticCallback, this, Buffers, BlockSize); }

   int CancelAsync(void) { return rtlsdr_cancel_async(Device); }

   // read directly given number of samples (remember to ResetBuffer() !)
   int Read(uint8_t *Buffer, int Samples)
   { Samples&=0xFFFFFF00;                 // number of samples must be a multiply of 256
     int BufferSize = 2*Samples;
     int ReadSize=0;
     if(rtlsdr_read_sync(Device, Buffer, BufferSize, &ReadSize)<0) return -1;
     return ReadSize/2; }

   int Read(SampleBuffer<uint8_t> &Buffer, int Samples)
   { if(Buffer.Allocate(2,Samples)<=0) return 0;
     int ReadSamples=Read(Buffer.Data, Samples);
     double Time = getTime();
     if(ReadSamples>0)
     { Buffer.Full=ReadSamples*2;
       Buffer.Rate=getSampleRate();
       Buffer.Freq=getCenterFreq();
       Buffer.Time=Time-(double)ReadSamples/Buffer.Rate; }
     // printf("RTLSDR::Read( , %d) => %d, %7.3fMHz %14.3fsec\n", Samples, ReadSamples, 1e-6*getCenterFreq(), Buffer.Time );
     return ReadSamples;
   }

} ;

// =================================================================================

