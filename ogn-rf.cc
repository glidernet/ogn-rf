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
#include <unistd.h>
#include <math.h>

#include <libconfig.h>

#include <algorithm>

#include "thread.h"     // multi-thread stuff
#include "fft.h"        // Fast Fourier Transform
#include "rtlsdr.h"     // SDR radio

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#ifndef VERSION
#define VERSION 0.0.0
#endif

#include "jpeg.h"
#include "socket.h"
#include "sysmon.h"

#include "pulsefilter.h"
#include "tonefilter.h"

// ==================================================================================================

template <class Float> // scale floating-point data to 8-bit gray scale image
 void LogImage(SampleBuffer<uint8_t> &Image, SampleBuffer<Float> &Data, Float LogRef=0, Float Scale=1, Float Bias=0)
{ Image.Allocate(Data);
  int Pixels=Data.Full;
  for(int Idx=0; Idx<Pixels; Idx++)
  { Float Pixel=Data.Data[Idx];
    if(LogRef)
    { if(Pixel) { Pixel=logf((Float)Pixel/LogRef); Pixel = Pixel*Scale + Bias; }
           else { Pixel=0; } }
    else
    { Pixel = Pixel*Scale + Bias; }
    if(Pixel<0x00) Pixel=0x00;
    else if(Pixel>0xFF) Pixel=0xFF;
    Image.Data[Idx]=(uint8_t)Pixel;
  }
  Image.Full=Pixels;
}

// ==================================================================================================

class RF_Acq                                    // acquire wideband (1MHz) RF data thus both OGN frequencies at same time
{ public:
   int    SampleRate;                           // [Hz] sampling rate

   int    OGN_GainMode;                         // 0=Auto, 1=Manual, 2=Linearity, 3=Sensitivity
   int    OGN_Gain;                             // [0.1dB] Rx gain for OGN reception
   int    OGN_CenterFreq;                       // [Hz] center acquisition frequency: can be 868.3+/-0.2 MHz
   double OGN_StartTime;                        // [sec] when to start acquisition on the center frequency
   int    OGN_SamplesPerRead;                   // [samples] should correspond to about 800 ms of data and be a multiple of 256
                                                // the goal is to listen on center frequency from 0.4 to 1.2 sec
   int    OGN_SaveRawData;
   const static uint32_t OGN_RawDataSync = 0x254F7D01;

                                                // for frequency hopping
   // int              FreqBase;                   // [Hz] for Australia: 917.0MHz, for USA: 902.2MHz
   static const int FreqHopChannelWidth = 400000;  // [Hz] 0.4MHz per channel
   int              OGN_FreqHopChannels;           // [number] Australia: 24, USA: 65

   int  DeviceIndex;                            // rtl-sdr device index
   char DeviceSerial[64];                       // serial number of the rtl-sdr device to be selected
   int  OffsetTuning;                           // [bool] this option might be good for E4000 tuner
   int  FreqCorr;                               // [ppm] frequency correction applied to the Rx chip
   RTLSDR SDR;                                  // SDR receiver (DVB-T stick)
   ReuseObjectQueue< SampleBuffer<uint8_t> > OutQueue; // OGN sample batches are sent there

   Thread Thr;                                  // acquisition thread
   volatile int StopReq;                        // request to stop the acquisition thread

   PulseFilter PulseFilt;

   static const int GSM_GainMode = 1;           // Manual gain mode for GSM
   int GSM_Gain;                                // [0.1dB] Rx gain for GSM frequency calibration
   int GSM_CenterFreq;                          // [Hz] should be selected to cover at lease one broadcast channel in the area
   int GSM_Scan;                                // [bool] scan around the whole GSM band
   int GSM_SamplesPerRead;                      // [samples] should cover one or more frequency correction bursts (100 ms should be enough ?)
   volatile float GSM_FreqCorr;                 // [ppm] frequency correction measured by the GSM frequency calibration
   static const int GSM_LowEdge = 925100000;    // [Hz] E-GSM-900 band, excluding the guards of 100kHz
   static const int GSM_UppEdge = 959900000;    // [Hz]
   static const int GSM_ScanStep =   800000;    // [Hz]
   ReuseObjectQueue< SampleBuffer<uint8_t> > GSM_OutQueue; // GSM sample batches are sent there

   MessageQueue<Socket *>  RawDataQueue;        // sockets send to this queue should be written with a most recent raw data

  public:
   RF_Acq() { Config_Defaults();
              GSM_FreqCorr=0;
              // PulseBox.Preset(PulseBoxSize);
              StopReq=0; Thr.setExec(ThreadExec); }

  ~RF_Acq() { }

  void Config_Defaults(void)
  { SampleRate=1000000;
    OGN_CenterFreq=868300000;
    OGN_StartTime=0.375; OGN_SamplesPerRead=(850*SampleRate)/1000;
    OGN_GainMode=1; OGN_Gain=600;
    OGN_FreqHopChannels=0;
    OGN_SaveRawData=0;
    PulseFilt.Threshold=0;
    DeviceIndex=0; DeviceSerial[0]=0; OffsetTuning=0; FreqCorr=0;
    GSM_CenterFreq=GSM_LowEdge+GSM_ScanStep/2; GSM_Scan=1; GSM_SamplesPerRead=(250*SampleRate)/1000; GSM_Gain=200; }

  int config_lookup_float_or_int(config_t *Config, const char *Path, double *Value)
  { int Ret = config_lookup_float(Config, Path, Value); if(Ret==CONFIG_TRUE) return Ret;
    int IntValue; Ret = config_lookup_int(Config, Path, &IntValue); if(Ret==CONFIG_TRUE) { (*Value) = IntValue; return Ret; }
    return Ret; }

  int Config(config_t *Config)
  { double Corr=0.0;
    config_lookup_float_or_int(Config,   "RF.FreqCorr", &Corr); FreqCorr = (int)floor(Corr+0.5);
    config_lookup_int(Config,   "RF.Device",         &DeviceIndex);
    const char *Serial = 0;
    config_lookup_string(Config,"RF.DeviceSerial",   &Serial);
    if(Serial) { strncpy(DeviceSerial, Serial, 64); DeviceSerial[63]=0; }
    config_lookup_int(Config,   "RF.OfsTune",        &OffsetTuning);
    config_lookup_int(Config,   "RF.OGN.GainMode",   &OGN_GainMode);

    config_lookup_int(Config,   "RF.OGN.SaveRawData",   &OGN_SaveRawData);

    SampleRate=1000000;
    if(config_lookup_int(Config, "RF.OGN.SampleRate", &SampleRate)!=CONFIG_TRUE)
    { double Rate;
      if(config_lookup_float(Config, "RF.OGN.SampleRate", &Rate)==CONFIG_TRUE) SampleRate=(int)floor(1e6*Rate+0.5);
      else if(config_lookup_int(Config, "RF.SampleRate", &SampleRate)!=CONFIG_TRUE)
      { if(config_lookup_float(Config, "RF.SampleRate", &Rate)==CONFIG_TRUE) SampleRate=(int)floor(1e6*Rate+0.5); }
    }

    double InpGain= 60.0; config_lookup_float_or_int(Config, "RF.OGN.Gain",         &InpGain); OGN_Gain=(int)floor(InpGain*10+0.5);
    double Freq=868.3;    config_lookup_float_or_int(Config, "RF.OGN.CenterFreq",   &Freq);    OGN_CenterFreq=(int)floor(Freq*1e6+0.5);
           InpGain= 20.0; config_lookup_float_or_int(Config, "RF.GSM.Gain",         &InpGain); GSM_Gain=(int)floor(InpGain*10+0.5);
           Freq=958.4;    config_lookup_float_or_int(Config, "RF.GSM.CenterFreq",   &Freq);    GSM_CenterFreq=(int)floor(Freq*1e6+0.5); GSM_Scan=0;

    OGN_FreqHopChannels=0;
    config_lookup_int(Config, "RF.OGN.FreqHopChannels",  &OGN_FreqHopChannels);

    PulseFilt.Threshold=0;
    config_lookup_int(Config, "RF.PulseFilter.Threshold",  &PulseFilt.Threshold);

    config_lookup_float(Config, "RF.OGN.StartTime", &OGN_StartTime);
    double SensTime=0.850;
    config_lookup_float(Config, "RF.OGN.SensTime",  &SensTime);
    OGN_SamplesPerRead=(int)floor(SensTime*SampleRate+0.5);
           SensTime=0.250;
    config_lookup_float(Config, "RF.GSM.SensTime",  &SensTime);
    GSM_SamplesPerRead=(int)floor(SensTime*SampleRate+0.5);

    return 0; }

   int QueueSize(void) { return OutQueue.Size(); }

   int Start(void) { StopReq=0; return Thr.Create(this); }
   int Stop(void)  { StopReq=1; return Thr.Join(); }

   static void *ThreadExec(void *Context)
   { RF_Acq *This = (RF_Acq *)Context; return This->Exec(); }

   void *Exec(void)
   { // printf("RF_Acq.Exec() ... Start\n");
     char Header[256];
     int Priority = Thr.getMaxPriority(); Thr.setPriority(Priority);
     int CurrCenterFreq = OGN_CenterFreq;
     while(!StopReq)
     { if(SDR.isOpen())                                                    // if device is already open
       { double Now  = SDR.getTime();
         int    IntTimeNow = (int)floor(Now); int ReadGSM = (IntTimeNow%20) == 0; // do the GSM calibration every 20 seconds

         int NextCenterFreq = OGN_CenterFreq;
         if(OGN_FreqHopChannels)
         { int Channel = FreqHopChannel1(IntTimeNow+1);
           NextCenterFreq += FreqHopChannelWidth*Channel; }

         double FracTimeNow = Now-IntTimeNow;
         double WaitTime = OGN_StartTime-FracTimeNow; if(WaitTime<0) WaitTime+=1.0;
         int SamplesToRead=OGN_SamplesPerRead;
         if( ReadGSM || (QueueSize()>1) ) SamplesToRead/=2; // when GSM calibration or data is not being processed fast enough we only read half-time
         if(WaitTime<0.200)
         { usleep((int)floor(1e6*WaitTime+0.5));                              // wait right before the time slot starts
           SampleBuffer<uint8_t> *Buffer = OutQueue.New();                    // get the next buffer to fill with raw I/Q data
           SDR.ResetBuffer();                                                 // needed before every Read()
           int Read=SDR.Read(*Buffer, SamplesToRead);                         // read the time slot raw RF data
           Buffer->Freq += CurrCenterFreq * (1e-6*GSM_FreqCorr);              // correct the frequency (sign ?)
           if(OGN_SaveRawData>0)
           { time_t Time=(time_t)floor(Buffer->Time);
             struct tm *TM = gmtime(&Time);
             char FileName[32]; sprintf(FileName, "OGNraw_%04d.%02d.%02d.u8", 1900+TM->tm_year, TM->tm_mon+1, TM->tm_mday);
             FILE *File=fopen(FileName, "ab");
             if(File)
             { SerializeWriteSync(File, OGN_RawDataSync);
               Buffer->Serialize(File);
               fclose(File);
               OGN_SaveRawData--; }
           }
           PulseFilt.Process(*Buffer);
           if(QueueSize()>1) printf("RF_Acq.Exec() ... Half time slot\n");
           // printf("RF_Acq.Exec() ... SDR.Read() => %d, Time=%16.3f, Freq=%6.1fMHz\n", Read, Buffer->Time, 1e-6*Buffer->Freq);
           if(Read>0) // RF data Read() successful
           { while(RawDataQueue.Size())                                       // when a raw data for this slot was requested
             { Socket *Client; RawDataQueue.Pop(Client);
               sprintf(Header, "HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: audio/basic\r\nContent-Disposition: attachment; filename=\"time-slot-rf_%14.3f.u8\"\r\n\r\n", Buffer->Time);
               Client->Send(Header);
               Client->Send(Buffer->Data, Buffer->Full);
               Client->SendShutdown(); Client->Close(); delete Client; }
             if(OutQueue.Size()<4) { OutQueue.Push(Buffer); }
                              else { OutQueue.Recycle(Buffer); printf("RF_Acq.Exec() ... Dropped a slot\n"); }
           } else     // RF data Read() failed
           { SDR.Close(); printf("RF_Acq.Exec() ... SDR.Read() failed => SDR.Close()\n"); continue; }
           if(ReadGSM) // if we are to read GSM in the second half-slot
           { SDR.setCenterFreq(GSM_CenterFreq);      // setup for the GSM reception
             SDR.setTunerGainMode(GSM_GainMode);
             SDR.setTunerGain(GSM_Gain);
             GSM_FreqCorr-=(FreqCorr-SDR.getFreqCorrection());
             SDR.setFreqCorrection(FreqCorr);
             SampleBuffer<uint8_t> *Buffer = GSM_OutQueue.New();
             SDR.ResetBuffer();
             int Read=SDR.Read(*Buffer, GSM_SamplesPerRead);
             // printf("RF_Acq.Exec() ...(GSM) SDR.Read() => %d, Time=%16.3f, Freq=%6.1fMHz\n", Read, Buffer->Time, 1e-6*Buffer->Freq);
             if(Read>0)
             { if(GSM_OutQueue.Size()<3) GSM_OutQueue.Push(Buffer);
                                  else { GSM_OutQueue.Recycle(Buffer); printf("RF_Acq.Exec() ... Dropped a GSM batch\n"); }
             }
             SDR.setTunerGainMode(OGN_GainMode);
             SDR.setTunerGain(OGN_Gain);                // back to OGN reception setup
             if(GSM_Scan)
             { GSM_CenterFreq+=GSM_ScanStep;
               if(GSM_CenterFreq>=GSM_UppEdge) GSM_CenterFreq=GSM_LowEdge+GSM_ScanStep/2;
             }
           }
           if(ReadGSM | OGN_FreqHopChannels)
           { SDR.setCenterFreq(NextCenterFreq); CurrCenterFreq=NextCenterFreq; }
         }
         else usleep(100000);
       }
       else                                                                // if not open yet or was closed due to an error
       { int Index=(-1);
         if(DeviceSerial[0]) Index=SDR.getDeviceIndexBySerial(DeviceSerial);
         if(Index<0) Index=DeviceIndex;
         if(SDR.Open(Index, OGN_CenterFreq, SampleRate)<0)                    // try to open it
         { printf("RF_Acq.Exec() ... SDR.Open(%d, , ) fails, retry after 1 sec\n", Index); usleep(1000000); }
         else
         { SDR.setOffsetTuning(OffsetTuning);
           SDR.setTunerGainMode(OGN_GainMode);
           SDR.setTunerGain(OGN_Gain); }
           SDR.setFreqCorrection(FreqCorr);
       }
     }

     SDR.Close();
     // printf("RF_Acq.Exec() ... Stop\n");
     return  0; }

   uint32_t FreqHopHash(uint32_t Time)
   { Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16); }

   uint16_t FreqHopChannel1(uint32_t Time)
   { return FreqHopHash((Time<<1)  )%OGN_FreqHopChannels; }

   uint16_t FreqHopChannel2(uint32_t Time)
   { return FreqHopHash((Time<<1)+1)%OGN_FreqHopChannels; }

} ;

// ==================================================================================================

template <class Float>
 class Inp_Filter
{ public:

   Thread Thr;                                      // processing thread
   volatile int StopReq;
   RF_Acq *RF;

   int              Enable;
   ToneFilter<Float> ToneFilt;

   ReuseObjectQueue< SampleBuffer< std::complex<Float> > > OutQueue;

  public:

   Inp_Filter(RF_Acq *RF)
   { this->RF=RF; Config_Defaults(); Preset(); }

   void Config_Defaults(void)
   { Enable  = 0; ToneFilt.FFTsize = 32768; ToneFilt.Threshold=32; }

   int Config(config_t *Config)
   { config_lookup_int(Config,   "RF.ToneFilter.Enable",    &Enable);
     config_lookup_int(Config,   "RF.ToneFilter.FFTsize",   &ToneFilt.FFTsize);
     config_lookup_float(Config, "RF.ToneFilter.Threshold", &ToneFilt.Threshold);
     return 0; }

   int Preset(void) { return ToneFilt.Preset(); }

   int QueueSize(void) { return OutQueue.Size(); }
   void Start(void)
   { StopReq=0; Thr.setExec(ThreadExec); Thr.Create(this); }

  ~Inp_Filter()
   { Thr.Cancel(); }

   double getCPU(void) // get CPU time for this thread
   {
#if !defined(__MACH__)
       struct timespec now; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now); return now.tv_sec + 1e-9*now.tv_nsec;
#else
       return 0;
#endif
   }

   static void *ThreadExec(void *Context)
   { Inp_Filter<Float> *This = (Inp_Filter<Float> *)Context; return This->Exec(); }

   void *Exec(void)
   { // printf("Inp_Filter.Exec() ... Start\n");
     while(!StopReq)
     { if(!Enable) { sleep(1); continue; }
       double ExecTime=getCPU();
       SampleBuffer<uint8_t> *InpBuffer = RF->OutQueue.Pop();   // here we wait for a new data batch
       // printf("Inp_Filter.Exec() ... Input(%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
       SampleBuffer< std::complex<Float> > *OutBuffer = OutQueue.New();
       ToneFilt.Process(OutBuffer, InpBuffer);
       RF->OutQueue.Recycle(InpBuffer);                         // let the input buffer go free
       // printf("Inp_Filter.Exec() ... Output(%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*OutBuffer->Freq, OutBuffer->Time, OutBuffer->Full/2);
       if(OutQueue.Size()<4) { OutQueue.Push(OutBuffer); }
                        else { OutQueue.Recycle(OutBuffer); printf("Inp_Filter.Exec() ... Dropped a slot\n"); }
       ExecTime=getCPU()-ExecTime; // printf("Inp_FFT.Exec() ... %5.3fsec\n", ExecTime);
     }
     // printf("Inp_FFT.Exec() ... Stop\n");
     return 0; }

   // classical sliding box filter - calc. the sum within box of 2*Radius+1
   static void BoxSum(Float *Output, Float *Input, int Size, int Radius)
   { int BoxSize=2*Radius+1;
     Float Sum=0; int InpIdx=0; int OutIdx=0;
     for( ; InpIdx<Radius; InpIdx++)
     { Sum+=Input[InpIdx]; }
     for( ; InpIdx<BoxSize; InpIdx++)
     { Sum+=Input[InpIdx]; Output[OutIdx++]=Sum; }
     for( ; InpIdx<Size; InpIdx++)
     { Sum+=Input[InpIdx]-Input[InpIdx-BoxSize]; Output[OutIdx++]=Sum; }
     for( ; OutIdx<Size; )
     { Sum-=Input[InpIdx-BoxSize]; Output[OutIdx++]=Sum; }
   }


} ;

// ==================================================================================================

template <class Float>
 class Inp_FFT                                      // FFT of the RF data
{ public:

   Thread Thr;                                      // processing thread
   volatile int StopReq;
   RF_Acq *RF;
   Inp_Filter<Float> *Filter;

   int              FFTsize;
#ifdef USE_RPI_GPU_FFT
   RPI_GPU_FFT      FFT;
#else
   DFT1d<Float>     FFT;
#endif
   Float           *Window;

   SampleBuffer< std::complex<Float> > OutBuffer;
   int  OutPipe;
   char OutPipeName[32];
   const static uint32_t OutPipeSync = 0x254F7D00 + sizeof(Float);

  public:
   Inp_FFT(RF_Acq *RF, Inp_Filter<Float> *Filter=0)
   { Window=0; this->RF=RF; this->Filter=Filter; Preset(); OutPipe=(-1); Config_Defaults(); }

   void Config_Defaults(void)
   { strcpy(OutPipeName, "ogn-rf.fifo"); }

   int Config(config_t *Config)
   { const char *PipeName = "ogn-rf.fifo";
     config_lookup_string(Config, "RF.PipeName",   &PipeName);
     strcpy(OutPipeName, PipeName);
     return 0; }

  int Preset(void) { return Preset(RF->SampleRate); }
   int Preset(int SampleRate)
   { FFTsize=(8*SampleRate)/15625;
     FFT.PresetForward(FFTsize);
     Window=(Float *)realloc(Window, FFTsize*sizeof(Float));
     FFT.SetSineWindow(Window, FFTsize, (Float)(1.0/sqrt(FFTsize)) );
     return 1; }

  int WriteToPipe(void) // write OutBuffer to the output pipe
  { if(OutPipe<0)
    { OutPipe=open(OutPipeName, O_WRONLY);
      if(OutPipe<0)
      { printf("Inp_FFT.Exec() ... Cannot open %s\n", OutPipeName);
        // here we could try to create the missing pipe
        return -1; } }
    int Len=SerializeWriteSync(OutPipe, OutPipeSync);
    if(Len>=0) { Len=SerializeWriteName(OutPipe, "FreqCorr"); }
    if(Len>=0) { Len=SerializeWriteData(OutPipe, (void *)&(RF->FreqCorr),     sizeof(int)   ); }
    if(Len>=0) { Len=SerializeWriteData(OutPipe, (void *)&(RF->GSM_FreqCorr), sizeof(float) ); }
    if(Len>=0) { Len=SerializeWriteSync(OutPipe, OutPipeSync); }
    if(Len>=0) { Len=SerializeWriteName(OutPipe, "Spectra"); }
    if(Len>=0) { Len=OutBuffer.Serialize(OutPipe); }
    if(Len<0) { printf("Inp_FFT.Exec() ... Error while writing to %s\n", OutPipeName); close(OutPipe); OutPipe=(-1); return -1; }
    return 0; }

   void Start(void)
   { StopReq=0; Thr.setExec(ThreadExec); Thr.Create(this); }

  ~Inp_FFT()
   { Thr.Cancel();
     if(Window) free(Window); }

   double getCPU(void) // get CPU time for this thread
   {
#if !defined(__MACH__)
       struct timespec now; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now); return now.tv_sec + 1e-9*now.tv_nsec;
#else
       return 0;
#endif
   }

   static void *ThreadExec(void *Context)
   { Inp_FFT *This = (Inp_FFT *)Context; return This->Exec(); }

   void *Exec(void)
   { // printf("Inp_FFT.Exec() ... Start\n");
     while(!StopReq)
     { double ExecTime=getCPU();
#ifndef USE_RPI_GPU_FFT
       if(Filter && Filter->Enable)
       { SampleBuffer< std::complex<Float> > *InpBuffer = Filter->OutQueue.Pop();
         // printf("Inp_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
         SlidingFFT(OutBuffer, *InpBuffer, FFT, Window);  // Process input samples, produce FFT spectra
         Filter->OutQueue.Recycle(InpBuffer);
       }
       else
#endif
       { SampleBuffer<uint8_t> *InpBuffer = RF->OutQueue.Pop(); // here we wait for a new data batch
         // printf("Inp_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
         SlidingFFT(OutBuffer, *InpBuffer, FFT, Window);  // Process input samples, produce FFT spectra
         RF->OutQueue.Recycle(InpBuffer);
       }
       WriteToPipe(); // here we send the FFT spectra in OutBuffer to the demodulator
       ExecTime=getCPU()-ExecTime; // printf("Inp_FFT.Exec() ... %5.3fsec\n", ExecTime);
     }
     // printf("Inp_FFT.Exec() ... Stop\n");
     if(OutPipe>=0) { close(OutPipe); OutPipe=(-1); }
     return 0; }

} ;

// ==================================================================================================

template <class Float>
 class GSM_FFT                                      // FFT of the GSM RF data
{ public:

   Thread Thr;                                      // processing thread
   volatile int StopReq;
   RF_Acq *RF;                                      // pointer to the RF acquisition

   int              FFTsize;
   DFT1d<Float>     FFT;
   Float           *Window;

   SampleBuffer< std::complex<Float> > Spectra;     // (complex) spectra
   SampleBuffer< Float >               Power;       // spectra power (energy)

   MessageQueue<Socket *>  SpectrogramQueue;        // sockets send to this queue should be written with a most recent spectrogram
   SampleBuffer<uint8_t>   Image;
   JPEG                    JpegImage;

   std::vector<Float>  PPM_Values;                  // [ppm] measured frequency correction values (a vector of)
   Float               PPM_Aver;                    // [ppm] average frequency correction
   Float               PPM_RMS;                     // [ppm] RMS of the frequency correction
   int                 PPM_Points;                  // number of measurements taken into the average
   time_t              PPM_Time;                    // time when correction measured
   Float            getPPM(void)  const { Float Value=PPM_Aver; return Value; }

  public:
   GSM_FFT(RF_Acq *RF)
   { Window=0; this->RF=RF; Preset(); }

   int Preset(void) { return Preset(RF->SampleRate); }
   int Preset(int SampleRate)
   { FFTsize=(8*SampleRate)/15625;
     FFT.PresetForward(FFTsize);
     Window=(Float *)realloc(Window, FFTsize*sizeof(Float));
     FFT.SetSineWindow(Window, FFTsize, (Float)(1.0/sqrt(FFTsize)) );
     PPM_Values.clear(); PPM_Aver=0; PPM_RMS=0; PPM_Points=0; PPM_Time=0;
     return 1; }

   void Start(void)
   { StopReq=0; Thr.setExec(ThreadExec); Thr.Create(this); }

  ~GSM_FFT()
   { Thr.Cancel();
     if(Window) free(Window); }

   double getCPU(void) // get CPU time for this thread
   {
#if !defined(__MACH__)
       struct timespec now; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now); return now.tv_sec + 1e-9*now.tv_nsec;
#else
       return 0.0;
#endif
   }

   static void *ThreadExec(void *Context)
   { GSM_FFT *This = (GSM_FFT *)Context; return This->Exec(); }

   void *Exec(void)
   { // printf("GSM_FFT.Exec() ... Start\n");
     while(!StopReq)
     { double ExecTime=getCPU();
       SampleBuffer<uint8_t> *InpBuffer = RF->GSM_OutQueue.Pop();
       // printf("GSM_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
       SlidingFFT(Spectra, *InpBuffer, FFT, Window);
       SpectraPower(Power, Spectra);
       RF->GSM_OutQueue.Recycle(InpBuffer);
       if(SpectrogramQueue.Size())
       { LogImage(Image, Power, (Float)0.33, (Float)32.0, (Float)32.0);
         JpegImage.Compress_MONO8(Image.Data, Image.Len, Image.Samples() ); }
       while(SpectrogramQueue.Size())
       { Socket *Client; SpectrogramQueue.Pop(Client);
         Client->Send("HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: image/jpeg\r\nRefresh: 10\r\n\r\n");
         // printf("GSM_FFT.Exec() ... Request for (GSM)spectrogram\n");
         Client->Send(JpegImage.Data, JpegImage.Size);
         Client->SendShutdown(); Client->Close(); delete Client; }
       Process();
       ExecTime=getCPU()-ExecTime; // printf("GSM_FFT.Exec() ... %5.3fsec\n", ExecTime);
     }
     // printf("GSM_FFT.Exec() ... Stop\n");
     return 0; }

   static const int ChanWidth = 200000; // [Hz] GSM channel width
   static const int DataRate  = 270833; // [Hz] GSM data rate
   SampleBuffer<Float> Aver, Peak, PeakPos, Bkg;

   void Process(void)
   { Float BinWidth=Power.Rate/2;                              // [Hz] FFT bin spectral width
     int Bins = Power.Len;                                     // [int] number of FFT bins
     Float FirstBinFreq = Power.Freq-BinWidth*Bins/2;          // [Hz] center frequency of the first FFT bin
     Float LastBinFreq  = Power.Freq+BinWidth*Bins/2;          // [Hz] center frequency of the one-after-the-last FFT bin

     int Chan = (int)ceil(FirstBinFreq/ChanWidth);             // integer channel number corr. to the first FFT bin (GSM channels are on multiples of 200kHz)
     for( ; ; Chan++)                                          // loop over (possible) channels in this scan
     { Float CenterFreq=Chan*ChanWidth; if(CenterFreq>=LastBinFreq) break; // center frequency of the channel
       Float LowFreq = CenterFreq-0.45*ChanWidth;              // [Hz] lower frequency to measure the channel
       Float UppFreq = CenterFreq+0.45*ChanWidth;              // [Hz] upper frequency to measure the channel
       int LowBin=(int)floor((LowFreq-FirstBinFreq)/BinWidth+0.5); // FFT bins corresponding to the channel frequency range
       int UppBin=(int)floor((UppFreq-FirstBinFreq)/BinWidth+0.5);
       if( (LowBin<0) || (LowBin>=Bins) ) continue;            // skip this channel if range to measure
       if( (UppBin<0) || (UppBin>=Bins) ) continue;            // not contained completely in this scan
       Float AverPower;
       int Marks=ProcessChan(AverPower, LowBin, UppBin, (CenterFreq-FirstBinFreq)/BinWidth, BinWidth, CenterFreq);
       if(Marks==1) PPM_Values.pop_back(); // if only one mark found, drop it - likely a false signal
       // printf("Process: Chan=%d, Freq=%8.3fMHz [%3d-%3d] %+6.1fdB %d\n", Chan, 1e-6*CenterFreq, LowBin, UppBin, 10*log10(AverPower), Marks);
       // { char FileName[32]; sprintf(FileName, "GSM_%5.1fMHz.dat", 1e-6*CenterFreq);
       //   FILE *File=fopen(FileName, "wt");
       //   for(int Idx=0; Idx<Aver.Full; Idx++)
       //  { fprintf(File, "%5d %12.6f %12.6f %+10.6f %10.6f\n",
       //                   Idx, Aver[Idx], Peak[Idx], PeakPos[Idx], Bkg[Idx]); }
       //   fclose(File); }
     }

     std::sort(PPM_Values.begin(), PPM_Values.end());

     if(PPM_Values.size()>=16)                                         // if at least 16 measured points
     { Float Aver, RMS; int Margin=PPM_Values.size()/4;
       AverRMS(Aver, RMS, PPM_Values.data()+Margin, PPM_Values.size()-2*Margin);
       // printf("PPM = %+7.3f (%5.3f) [%d]\n", Aver, RMS, PPM_Values.size()-2*Margin);
       if(RMS<0.5)
       { PPM_Aver=Aver; PPM_RMS=RMS; PPM_Points=PPM_Values.size()-2*Margin; PPM_Time=(time_t)floor(Power.Time+0.5); PPM_Values.clear();
         printf("GSM freq. calib. = %+7.3f +/- %5.3f ppm, %d points\n", PPM_Aver, PPM_RMS, PPM_Points);
         Float Corr=RF->GSM_FreqCorr; Corr+=0.25*(PPM_Aver-Corr); RF->GSM_FreqCorr=Corr; }
       PPM_Values.clear();
     }

     if(PPM_Values.size()>=8)                                          // if at least 8 measured points
     { Float Aver, RMS;
       AverRMS(Aver, RMS, PPM_Values.data()+1, PPM_Values.size()-2);   // calc. the average excluding two extreme points
       // printf("PPM = %+7.3f (%5.3f) [%d]\n", Aver, RMS, PPM_Values.size()-2);
       if(RMS<0.5)
       { PPM_Aver=Aver; PPM_RMS=RMS; PPM_Points=PPM_Values.size()-2; PPM_Time=(time_t)floor(Power.Time+0.5); PPM_Values.clear();
         printf("GSM freq. calib. = %+7.3f +/- %5.3f ppm, %d points\n", PPM_Aver, PPM_RMS, PPM_Points);
         Float Corr=RF->GSM_FreqCorr; Corr+=0.25*(PPM_Aver-Corr); RF->GSM_FreqCorr=Corr; }
     }

   }

   // Average, Peak (with Position) and Background = Average - values around the Peak
   static void AverPeakBkg(Float &Aver, Float &Peak, Float &PeakPos, Float &Bkg, Float *Data, int Size)
   { Aver=0; Peak=0; PeakPos=0; int PeakIdx=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Float Dat=Data[Idx];
       if(Dat>Peak) { Peak=Dat; PeakIdx=Idx; }
       Aver+=Dat; }
     if(PeakIdx==0)             { Peak+=Data[     1];                    PeakPos=PeakIdx+Data[     1]/Peak;                      Bkg=(Aver-Peak)/(Size-2); }
     else if(PeakPos==(Size-1)) { Peak+=Data[Size-2];                    PeakPos=PeakIdx-Data[Size-2]/Peak;                      Bkg=(Aver-Peak)/(Size-2); }
     else                       { Peak+=Data[PeakIdx+1]+Data[PeakIdx-1]; PeakPos=PeakIdx+(Data[PeakIdx+1]-Data[PeakIdx-1])/Peak; Bkg=(Aver-Peak)/(Size-3); }
     Aver/=Size; }

   // average and RMS of a data series
   static void AverRMS(Float &Aver, Float &RMS, Float *Data, int Size)
   { Aver=0; RMS=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Aver+=Data[Idx]; }
     Aver/=Size;
     for(int Idx=0; Idx<Size; Idx++)
     { Float Diff=Data[Idx]-Aver; RMS+=Diff*Diff; }
     RMS=sqrt(RMS/Size); }

   int ProcessChan(Float &AverPower, int LowBin, int UppBin, Float CenterBin, Float BinWidth, Float CenterFreq)
   { int Slides = Power.Samples();
     int Bins = Power.Len;
     Aver.Allocate(1, Slides); Peak.Allocate(1, Slides); PeakPos.Allocate(1, Slides); Bkg.Allocate(1, Slides);
     Float *Data = Power.Data;
     for(int Idx=0; Idx<Slides; Idx++, Data+=Bins)
     { AverPeakBkg(Aver[Idx], Peak[Idx], PeakPos[Idx], Bkg[Idx], Data+LowBin, UppBin-LowBin+1);
       PeakPos[Idx]+=LowBin-CenterBin; }
     Aver.Full=Slides; Peak.Full=Slides; PeakPos.Full=Slides; Bkg.Full=Slides;
     Float PowerRMS; AverRMS(AverPower, PowerRMS, Aver.Data, Slides);
     // printf("AverPower=%3.1f, PowerRMS=%3.1f\n", AverPower, PowerRMS);
     if(PowerRMS>(0.5*AverPower)) return 0;          // skip pulsing channels

     Float AverPeak, PeakRMS; AverRMS(AverPeak, PeakRMS, Peak.Data, Slides);
     Float AverBkg, BkgRMS; AverRMS(AverBkg, BkgRMS, Bkg.Data, Slides);
     // printf("AverPeak=%3.1f, PeakRMS=%3.1f, AverBkg=%5.3f, BkgRMS=%5.3f\n", AverPeak, PeakRMS, AverBkg, BkgRMS);

     int Marks=0;
     Float PeakThres = 4*PeakRMS;
     Float BkgThres  = 4*BkgRMS;
     for(int Idx=1; Idx<(Slides-1); Idx++)
     { Float PeakL=Peak.Data[Idx-1]-AverPeak;
       Float PeakM=Peak.Data[Idx  ]-AverPeak;
       Float PeakR=Peak.Data[Idx+1]-AverPeak;
       Float PeakSum = PeakL+PeakM+PeakR;
       if(PeakSum<=PeakThres) continue;
       if(PeakM<PeakL)  continue;
       if(PeakM<=PeakR) continue;
       if(PeakM<=((PeakL+PeakR)/2)) continue;
       Float BkgSum = Bkg.Data[Idx-1]+Bkg.Data[Idx]+Bkg.Data[Idx+1];
       if((3*AverBkg-BkgSum)<BkgThres) continue;
       if(Peak.Data[Idx]<(40*Bkg.Data[Idx])) continue;
       Float PPM = -1e6*(PeakPos.Data[Idx]*BinWidth-(Float)DataRate/4)/CenterFreq;
       // printf("Mark: PeakSum[%5d]=%8.1f/%6.1f Bkg=%8.3f/%6.3f Peak/Bkg=%8.1f PeakPos=%+7.3f %+7.3fppm\n",
       //         Idx, PeakSum, PeakThres, 3*AverBkg-BkgSum, BkgThres, Peak.Data[Idx]/Bkg.Data[Idx], PeakPos.Data[Idx], PPM);
       PPM_Values.push_back(PPM);
       Marks++; }

     return Marks; }

} ;

// ==================================================================================================


template <class Float>
 class HTTP_Server
{ public:

   int                 Port;      // listenning port
   Thread              Thr;       // processing thread
   RF_Acq             *RF;        // pointer to RF acquisition
   GSM_FFT<Float>     *GSM;
   char                Host[32];  // Host name

  public:
   HTTP_Server(RF_Acq *RF, GSM_FFT<Float> *GSM)
   { this->RF=RF; this->GSM=GSM;
     Host[0]=0; SocketAddress::getHostName(Host, 32);
     Config_Defaults(); }

   void Config_Defaults(void)
   { Port=8080; }

   int Config(config_t *Config)
   { config_lookup_int(Config, "HTTP.Port", &Port); return 0; }

   void Start(void)
   { if(Port<=0) return;
     Thr.setExec(ThreadExec); Thr.Create(this); }

  ~HTTP_Server()
   { if(Port) Thr.Cancel(); }

   static void *ThreadExec(void *Context)
   { HTTP_Server *This = (HTTP_Server *)Context; return This->Exec(); }

   void *Exec(void)
   { printf("HTTP_Server.Exec() ... Start\n");
     while(1)
     { Socket Listen;
       if(Listen.Create_STREAM()<0) { printf("HTTP_Server.Exec() ... Cannot Create_STREAM()\n"); sleep(1); continue; }
       if(Listen.setReuseAddress()<0) { printf("HTTP_Server.Exec() ... Cannot setReuseAddress()\n"); sleep(1); continue; }
       if(Listen.Listen(Port)<0) { printf("HTTP_Server.Exec() ... Cannot listen() on port %d\n", Port); sleep(1); continue; }
       printf("HTTP_Server.Exec() ... Listening on port %d\n", Port);
       while(1)
       { Socket *Client = new Socket; SocketAddress ClientAddress;
         if(Listen.Accept(*Client, ClientAddress)<0) { printf("HTTP_Server.Exec() ... Cannot accept()\n"); sleep(1); break; }
         printf("HTTP_Server.Exec() ... Client from %s\n", ClientAddress.getIPColonPort());
         Client->setReceiveTimeout(2.0); Client->setSendTimeout(5.0); Client->setLinger(1, 5);
         SocketBuffer Request; time_t ConnectTime; time(&ConnectTime);
         while(1)
         { if(Client->Receive(Request)<0) { printf("HTTP_Server.Exec() ... Cannot receive()\n"); Client->SendShutdown(); Client->Close(); delete Client; Client=0; break; }
           if( Request.Len && strstr(Request.Data, "\r\n\r\n") ) break;
           time_t Now; time(&Now);
           if((Now-ConnectTime)>2) { printf("HTTP_Server.Exec() ... Request timeout\n"); Client->SendShutdown(); Client->Close(); delete Client; Client=0; break; }
         }
         if(Client)
         { // printf("HTTP_Server.Exec() ... Request[%d]:\n", Request.Len); Request.WriteToFile(stdout); fflush(stdout);
           ProcessRequest(Client, Request); }
       }
       Listen.Close();
     }
     printf("HTTP_Server.Exec() ... Stop\n");
     return 0; }

   int CopyWord(char *Dst, char *Src, int MaxLen)
   { int Count=0; MaxLen-=1;
     for( ; ; )
     { char ch = (*Src++); if(ch<=' ') break;
       if(Count>=MaxLen) return -1;
       (*Dst++) = ch; Count++; }
     (*Dst++)=0;
     return Count; }

   void ProcessRequest(Socket *Client, SocketBuffer &Request)
   { if(memcmp(Request.Data, "GET ", 4)!=0) goto BadRequest;
     char File[64]; if(CopyWord(File, Request.Data+4, 64)<0) goto BadRequest;
     printf("HTTP_Server.Exec() ... Request for %s\n", File);

          if(strcmp(File, "/")==0)
     { Status(Client); return; }
     else if( (strcmp(File, "/status.html")==0)         || (strcmp(File, "status.html")==0) )
     { Status(Client); return; }
     else if( (strcmp(File, "/gsm-spectrogram.jpg")==0) || (strcmp(File, "gsm-spectrogram.jpg")==0) )
     { GSM->SpectrogramQueue.Push(Client); return; }
     else if( (strcmp(File, "/time-slot-rf.u8")==0)  || (strcmp(File, "time-slot-rf.u8")==0) )
     { RF->RawDataQueue.Push(Client); return; }
     // NotFound:
       Client->Send("HTTP/1.0 404 Not Found\r\n\r\n"); Client->SendShutdown(); Client->Close(); delete Client; return;

     BadRequest:
       Client->Send("HTTP/1.0 400 Bad Request\r\n\r\n"); Client->SendShutdown(); Client->Close(); delete Client; return;
   }

   void Status(Socket *Client)
   { Client->Send("\
HTTP/1.1 200 OK\r\n\
Cache-Control: no-cache\r\n\
Content-Type: text/html\r\n\
Refresh: 5\r\n\
\r\n\
<!DOCTYPE html>\r\n\
<html>\r\n\
");
     // time_t Now; time(&Now);
     dprintf(Client->SocketFile, "\
<title>RTLSDR-OGN RF processor " STR(VERSION) " status</title>\n\
<b>RTLSDR OGN RF processor " STR(VERSION) "/"__DATE__"</b><br /><br />\n\n");

     dprintf(Client->SocketFile, "<table>\n<tr><th>System</th><th></th></tr>\n");

     dprintf(Client->SocketFile, "<tr><td>Host name</td><td align=right><b>%s</b></td></tr>\n", Host);
     time_t Now; time(&Now);
     struct tm TM; localtime_r(&Now, &TM);
     dprintf(Client->SocketFile, "<tr><td>Local time</td><td align=right><b>%02d:%02d:%02d</b></td></tr>\n", TM.tm_hour, TM.tm_min, TM.tm_sec);
     dprintf(Client->SocketFile, "<tr><td>Software</td><td align=right><b>" STR(VERSION) "</b></td></tr>\n");

#ifndef __MACH__
     struct sysinfo SysInfo;
     if(sysinfo(&SysInfo)>=0)
     { dprintf(Client->SocketFile, "<tr><td>CPU load</td><td align=right><b>%3.1f/%3.1f/%3.1f</b></td></tr>\n",
                                   SysInfo.loads[0]/65536.0, SysInfo.loads[1]/65536.0, SysInfo.loads[2]/65536.0);
       dprintf(Client->SocketFile, "<tr><td>RAM [free/total]</td><td align=right><b>%3.1f/%3.1f MB</b></td></tr>\n",
                                   1e-6*SysInfo.freeram*SysInfo.mem_unit, 1e-6*SysInfo.totalram*SysInfo.mem_unit);
     }
#endif

     float CPU_Temperature;
     if(getCpuTemperature(CPU_Temperature)>=0)
       dprintf(Client->SocketFile, "<tr><td>CPU temperature</td><td align=right><b>%+5.1f &#x2103;</b></td></tr>\n",    CPU_Temperature);
     float SupplyVoltage;
     if(getSupplyVoltage(SupplyVoltage)>=0)
       dprintf(Client->SocketFile, "<tr><td>Supply voltage</td><td align=right><b>%5.3f V</b></td></tr>\n",    SupplyVoltage);
     float SupplyCurrent;
     if(getSupplyCurrent(SupplyCurrent)>=0)
       dprintf(Client->SocketFile, "<tr><td>Supply current</td><td align=right><b>%5.3f A</b></td></tr>\n",    SupplyCurrent);

     double NtpTime, EstError, RefFreqCorr;
     if(getNTP(NtpTime, EstError, RefFreqCorr)>=0)
     { time_t Time = floor(NtpTime);
       struct tm TM; gmtime_r(&Time, &TM);
       dprintf(Client->SocketFile, "<tr><td>NTP UTC time</td><td align=right><b>%02d:%02d:%02d</b></td></tr>\n", TM.tm_hour, TM.tm_min, TM.tm_sec);
       dprintf(Client->SocketFile, "<tr><td>NTP est. error</td><td align=right><b>%3.1f ms</b></td></tr>\n", 1e3*EstError);
       dprintf(Client->SocketFile, "<tr><td>NTP freq. corr.</td><td align=right><b>%+5.2f ppm</b></td></tr>\n", RefFreqCorr);
     }

     if(RF->SDR.isOpen())
     { dprintf(Client->SocketFile, "<tr><th>RTL-SDR device #%d</th><th></th></tr>\n",                                  RF->SDR.DeviceIndex);
       dprintf(Client->SocketFile, "<tr><td>Name</td><td align=right><b>%s</b></td></tr>\n",                           RF->SDR.getDeviceName());
       dprintf(Client->SocketFile, "<tr><td>Tuner type</td><td align=right><b>%s</b></td></tr>\n",                     RF->SDR.getTunerTypeName());
       char Manuf[256], Product[256], Serial[256];
       RF->SDR.getUsbStrings(Manuf, Product, Serial);
       dprintf(Client->SocketFile, "<tr><td>Manufacturer</td><td align=right><b>%s</b></td></tr>\n",                    Manuf);
       dprintf(Client->SocketFile, "<tr><td>Product</td><td align=right><b>%s</b></td></tr>\n",                         Product);
       dprintf(Client->SocketFile, "<tr><td>Serial</td><td align=right><b>%s</b></td></tr>\n",                          Serial);
#ifdef NEW_RTLSDR_LIB
       for(int Stage=0; Stage<8; Stage++)
       { char Descr[256]; int Gains=RF->SDR.getTunerStageGains(Stage, 0, Descr); if(Gains<=0) break;
         dprintf(Client->SocketFile, "<tr><td>Tuner stage #%d</td><td align=right><b>%s [%2d]</b></td></tr>\n",  Stage, Descr, Gains);
       }
       dprintf(Client->SocketFile, "<tr><td>Tuner bandwidths</td><td align=right><b>[%d]</b></td></tr>\n",             RF->SDR.getTunerBandwidths());
       dprintf(Client->SocketFile, "<tr><td>Tuner gains</td><td align=right><b>[%d]</b></td></tr>\n",                  RF->SDR.getTunerGains());
#endif
       dprintf(Client->SocketFile, "<tr><td>Center frequency</td><td align=right><b>%7.3f MHz</b></td></tr>\n",        1e-6*RF->SDR.getCenterFreq());
       dprintf(Client->SocketFile, "<tr><td>Sample rate</td><td align=right><b>%5.3f MHz</b></td></tr>\n",             1e-6*RF->SDR.getSampleRate());
       dprintf(Client->SocketFile, "<tr><td>Frequency correction</td><td align=right><b>%+5.1f ppm</b></td></tr>\n",   RF->FreqCorr + RF->GSM_FreqCorr);
       uint32_t RtlFreq, TunerFreq; RF->SDR.getXtalFreq(RtlFreq, TunerFreq);
       dprintf(Client->SocketFile, "<tr><td>RTL Xtal</td><td align=right><b>%8.6f MHz</b></td></tr>\n",    1e-6*RtlFreq);
       dprintf(Client->SocketFile, "<tr><td>Tuner Xtal</td><td align=right><b>%8.6f MHz</b></td></tr>\n",  1e-6*TunerFreq);
/*
       dprintf(Client->SocketFile, "<tr><td>Gain[%d]</td><td align=right><b>", RF->SDR.Gains);
       for(int Idx=0; Idx<RF->SDR.Gains; Idx++)
       { dprintf(Client->SocketFile, "%c%3.1f", Idx?',':' ', 0.1*RF->SDR.Gain[Idx]);
       }
       dprintf(Client->SocketFile, " [dB]</b></td></tr>\n");
*/
     }

     dprintf(Client->SocketFile, "<tr><th>RF</th><th></th></tr>\n");
     dprintf(Client->SocketFile, "<tr><td>RF.Device</td><td align=right><b>%d</b></td></tr>\n",                       RF->DeviceIndex);
     if(RF->DeviceSerial[0])
       dprintf(Client->SocketFile, "<tr><td>RF.DeviceSerial</td><td align=right><b>%s</b></td></tr>\n",               RF->DeviceSerial);
     dprintf(Client->SocketFile, "<tr><td>RF.SampleRate</td><td align=right><b>%3.1f MHz</b></td></tr>\n",   1e-6*RF->SampleRate);
     dprintf(Client->SocketFile, "<tr><td>RF.FreqCorr</td><td align=right><b>%+3d ppm</b></td></tr>\n",               RF->FreqCorr);
     dprintf(Client->SocketFile, "<tr><td>Fine calib. FreqCorr</td><td align=right><b>%+5.1f ppm</b></td></tr>\n",    RF->GSM_FreqCorr);
     dprintf(Client->SocketFile, "<tr><td>RF.PulseFilter.Threshold</td><td align=right><b>%d</b></td></tr>\n",        RF->PulseFilt.Threshold);
     dprintf(Client->SocketFile, "<tr><td>RF.PulseFilter duty</td><td align=right><b>%5.1f ppm</b></td></tr>\n",    1e6*RF->PulseFilt.Duty);
     // dprintf(Client->SocketFile, "<tr><td>RF.ToneFilter.Enable</td><td align=right><b>%d</b></td></tr>\n",                  FFT->Filter->Enable);
     // dprintf(Client->SocketFile, "<tr><td>RF.ToneFilter.FFTsize</td><td align=right><b>%d</b></td></tr>\n",                 FFT->Filter->FFTsize);
     // dprintf(Client->SocketFile, "<tr><td>RF.ToneFilter.Threshold</td><td align=right><b>%3.1f</b></td></tr>\n",            FFT->Filter->Threshold);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.GainMode</td><td align=right><b>%d</b></td></tr>\n",                 RF->OGN_GainMode);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.Gain</td><td align=right><b>%4.1f dB</b></td></tr>\n",           0.1*RF->OGN_Gain);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.CenterFreq</td><td align=right><b>%5.1f MHz</b></td></tr>\n",   1e-6*RF->OGN_CenterFreq);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.FreqHopChannels</td><td align=right><b>%d</b></td></tr>\n",          RF->OGN_FreqHopChannels);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.StartTime</td><td align=right><b>%5.3f sec</b></td></tr>\n",         RF->OGN_StartTime);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.SensTime</td><td align=right><b>%5.3f sec</b></td></tr>\n", (double)(RF->OGN_SamplesPerRead)/RF->SampleRate);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.SaveRawData</td><td align=right><b>%d sec</b></td></tr>\n", RF->OGN_SaveRawData);
     dprintf(Client->SocketFile, "<tr><td>RF.GSM.CenterFreq</td><td align=right><b>%5.1f MHz</b></td></tr>\n",   1e-6*RF->GSM_CenterFreq);
     dprintf(Client->SocketFile, "<tr><td>RF.GSM.Gain</td><td align=right><b>%4.1f dB</b></td></tr>\n",           0.1*RF->GSM_Gain);
     dprintf(Client->SocketFile, "<tr><td>RF.GSM.SensTime</td><td align=right><b>%5.3f sec</b></td></tr>\n", (double)(RF->GSM_SamplesPerRead)/RF->SampleRate);


     dprintf(Client->SocketFile, "</table>\n");

     Client->Send("\
<br />\r\n\
RF spectrograms:\r\n\
<a href='gsm-spectrogram.jpg'>GSM frequency calibration</a><br />\r\n\
<br /><br />\r\n\
<a href='time-slot-rf.u8'>RF raw data</a> of a time-slot (8-bit unsigned I/Q) - a 2 MB binary file !<br />\r\n\
");

     Client->Send("</html>\r\n");
     Client->SendShutdown(); Client->Close(); delete Client; }

} ;

// ==================================================================================================

  RF_Acq             RF;                         // RF input: acquires RF data for OGN and selected GSM frequency

  Inp_Filter<float>  Filter(&RF);                // Coherent interference filter

  Inp_FFT<float>     FFT(&RF, &Filter);          // FFT for OGN demodulator
  GSM_FFT<float>     GSM(&RF);                   // GSM frequency calibration

  HTTP_Server<float> HTTP(&RF, &GSM);            // HTTP server to show status and spectrograms

void SigHandler(int signum) // Signal handler, when user pressed Ctrl-C or process stops for whatever reason
{ RF.StopReq=1; }

// ----------------------------------------------------------------------------------------------------

int SetUserValue(const char *Name, float Value)
{ // printf("%s = %f\n", Name, Value);
  if(strcmp(Name, "RF.FreqCorr")==0)
  { RF.FreqCorr=(int)floor(Value+0.5);
    printf("RF.FreqCorr=%+d ppm\n", RF.FreqCorr);
    return 1; }
  if(strcmp(Name, "RF.OGN.Gain")==0)
  { RF.OGN_Gain=(int)floor(10*Value+0.5);
    printf("RF.OGN.Gain=%3.1f dB\n", 0.1*RF.OGN_Gain);
    return 1; }
  if(strcmp(Name, "RF.OGN.GainMode")==0)
  { RF.OGN_GainMode=(int)floor(Value+0.5);
    printf("RF.OGN.GainMode=%d\n", RF.OGN_GainMode);
    return 1; }
  if(strcmp(Name, "RF.OGN.SaveRawData")==0)
  { RF.OGN_SaveRawData=(int)floor(Value+0.5);
    printf("RF.OGN.SaveRawData=%d\n", RF.OGN_SaveRawData);
    return 1; }
  if(strcmp(Name, "RF.GSM.Gain")==0)
  { RF.GSM_Gain=(int)floor(10*Value+0.5);
    printf("RF.GSM.Gain=%3.1f dB\n", 0.1*RF.GSM_Gain);
    return 1; }
  if(strcmp(Name, "RF.OGN.CenterFreq")==0)
  { RF.OGN_CenterFreq=(int)floor(1e6*Value+0.5);
    printf("RF.OGN.CenterFreq=%7.3f MHz\n", 1e-6*RF.OGN_CenterFreq);
    return 1; }
  if(strcmp(Name, "RF.GSM.CenterFreq")==0)
  { RF.GSM_CenterFreq=(int)floor(1e6*Value+0.5);
    printf("RF.GSM.CenterFreq=%7.3f MHz\n", 1e-6*RF.GSM_CenterFreq);
    return 1; }
  if(strcmp(Name, "RF.OGN.FreqHopChannels")==0)
  { RF.OGN_FreqHopChannels=(int)floor(Value+0.5);
    printf("RF.OGN_FreqHopChannels=%d\n", RF.OGN_FreqHopChannels);
    return 1; }
  if(strcmp(Name, "RF.PulseFilter.Threshold")==0)
  { RF.PulseFilt.Threshold=(int)floor(Value+0.5);
    printf("RF.PulseFilter.Threshold=%d\n", RF.PulseFilt.Threshold);
    return 1; }
  return 0; }

int PrintUserValues(void)
{ printf("Settable parameters:\n");
  printf("RF.FreqCorr=%+d(%+3.1f) ppm\n",      RF.FreqCorr, RF.GSM_FreqCorr);
  printf("RF.PulseFilter.Threshold=%d",        RF.PulseFilt.Threshold);
  if(RF.PulseFilt.Threshold) printf(" .Duty=%3.1fppm", 1e6*RF.PulseFilt.Duty);
  printf("\n");
  printf("RF.OGN.CenterFreq=%7.3f MHz\n", 1e-6*RF.OGN_CenterFreq);
  printf("RF.OGN_FreqHopChannels=%d\n",        RF.OGN_FreqHopChannels);
  printf("RF.OGN.Gain=%3.1f dB\n",         0.1*RF.OGN_Gain);
  printf("RF.OGN.GainMode=%d\n",               RF.OGN_GainMode);
  printf("RF.OGN.SaveRawData=%d\n",            RF.OGN_SaveRawData);
  printf("RF.GSM.CenterFreq=%7.3f MHz\n", 1e-6*RF.GSM_CenterFreq);
  printf("RF.GSM.Gain=%3.1f dB\n",         0.1*RF.GSM_Gain);
  return 0; }

int UserCommand(char *Cmd)
{ if(strchr(Cmd, '\n')==0) return 0;
  char *Equal = strchr(Cmd, '=');
  // printf("User command: %s", Cmd);
  if(Equal)
  { Equal[0]=0; const char *ValuePtr=Equal+1;
    char *Name=Cmd;
    for( ; ; )
    { char ch=Name[0]; if(ch==0) break;
      if(ch>' ') break;
      Name++; }
    for( ; ; )
    { Equal--;
      char ch=Equal[0]; if(ch==0) break;
      if(ch>' ') break;
      Equal[0]=0; }
    float Value;
    if(sscanf(ValuePtr, "%f", &Value)==1) SetUserValue(Name, Value);
  }
  PrintUserValues();
  return 0; }

// ----------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  const char *ConfigFileName = "rtlsdr-ogn.conf";
  if(argc>1) ConfigFileName = argv[1];

  config_t Config;
  config_init(&Config);
  if(config_read_file(&Config, ConfigFileName)==CONFIG_FALSE)
  { printf("Could not read %s as configuration file\n", ConfigFileName); config_destroy(&Config); return -1; }

  struct sigaction SigAction;
  SigAction.sa_handler = SigHandler;              // setup the signal handler (for Ctrl-C or when process is stopped)
  sigemptyset(&SigAction.sa_mask);
  SigAction.sa_flags = 0;

  struct sigaction SigIgnore;
  SigIgnore.sa_handler = SIG_IGN;
  sigemptyset(&SigIgnore.sa_mask);
  SigIgnore.sa_flags = 0;

  sigaction(SIGINT,  &SigAction, 0);
  sigaction(SIGTERM, &SigAction, 0);
  sigaction(SIGQUIT, &SigAction, 0);
  sigaction(SIGPIPE, &SigIgnore, 0);              // we want to ignore pipe/fifo read/write errors, we handle them by return codes

  RF.Config_Defaults();
  RF.Config(&Config);

  Filter.Config_Defaults();
  Filter.Config(&Config);
  if(Filter.Enable) Filter.Preset();

  FFT.Config_Defaults();
  FFT.Config(&Config);
  FFT.Preset();

  GSM.Preset();

  HTTP.Config_Defaults();
  HTTP.Config(&Config);
  HTTP.Start();

  config_destroy(&Config);

  if(Filter.Enable) Filter.Start();
  FFT.Start();
  GSM.Start();
  RF.Start();

  char Cmd[128];
  while(!RF.StopReq)
  { if(fgets(Cmd, 128, stdin)==0) break;
    UserCommand(Cmd); }

  sleep(4);
  RF.Stop();

  return 0; }


