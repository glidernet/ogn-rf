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
                int SampleRate;                 // [Hz] sampling rate
                int Gain;                       // [0.1dB] Rx gain for OGN reception
                int CenterFreq;                 // [Hz] center acquisition frequency: can be 868.3+/-0.2 MHz
                double StartTime;               // [sec] when to start acquisition on the center frequency
                int SamplesPerRead;             // [samples] should correspond to about 800 ms of data and be a multiple of 256
                                                // the goal is to listen on center frequency from 0.3 to 1.1 sec
   int  DeviceIndex;                            // rtl-sdr device index
   char DeviceSerial[12];                       // serial number of the rtl-sdr device to be selected
   int  OffsetTuning;                           // [bool] this option might be good for E4000 tuner
   int  FreqCorr;                               // [ppm] frequency correction applied to the Rx chip
   RTLSDR SDR;                                  // SDR receiver (DVB-T stick)
   ReuseObjectQueue< SampleBuffer<uint8_t> > OutQueue; // OGN sample batches are sent there

   Thread Thr;                                  // acquisition thread
   volatile int StopReq;                        // request to stop the acquisition thread

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

   RF_Acq() { Config_Defaults();
              GSM_FreqCorr=0;
              StopReq=0; Thr.setExec(ThreadExec); }

  ~RF_Acq() { }

  void Config_Defaults(void)
  { SampleRate=1000000;
    CenterFreq=868300000; StartTime=0.400; SamplesPerRead=(850*SampleRate)/1000; Gain=600;
    DeviceIndex=0; DeviceSerial[0]=0; OffsetTuning=0; FreqCorr=0;
    GSM_CenterFreq=GSM_LowEdge+GSM_ScanStep/2; GSM_Scan=1; GSM_SamplesPerRead=(250*SampleRate)/1000; GSM_Gain=200; }

  int Config(config_t *Config)
  { config_lookup_int(Config,   "RF.FreqCorr",       &FreqCorr);
    config_lookup_int(Config,   "RF.Device",         &DeviceIndex);
    const char *Serial = 0;
    config_lookup_string(Config,"RF.DeviceSerial",   &Serial);
    if(Serial) { strncpy(DeviceSerial, Serial, 12); DeviceSerial[11]=0; }
    config_lookup_int(Config,   "RF.OfsTune",        &OffsetTuning);

    SampleRate=1000000;
    config_lookup_int(Config, "RF.OGN.SampleRate", &SampleRate);

    double InpGain= 60.0; config_lookup_float(Config, "RF.OGN.Gain",         &InpGain); Gain=(int)floor(InpGain*10+0.5);
    double Freq=868.3;    config_lookup_float(Config, "RF.OGN.CenterFreq",   &Freq);    CenterFreq=(int)floor(Freq*1e6+0.5);
           InpGain= 20.0; config_lookup_float(Config, "RF.GSM.Gain",         &InpGain); GSM_Gain=(int)floor(InpGain*10+0.5);
           Freq=958.4;    config_lookup_float(Config, "RF.GSM.CenterFreq",   &Freq);    GSM_CenterFreq=(int)floor(Freq*1e6+0.5); GSM_Scan=0;

    config_lookup_float(Config, "RF.OGN.StartTime", &StartTime);
    double SensTime=0.850;
    config_lookup_float(Config, "RF.OGN.SensTime",  &SensTime);
    SamplesPerRead=(int)floor(SensTime*SampleRate+0.5);
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
     int Priority = Thr.getMaxPriority(); Thr.setPriority(Priority);
     while(!StopReq)
     { if(SDR.isOpen())                                                    // if device is already open
       { double Now  = SDR.getTime();
         int    IntTimeNow = (int)floor(Now); int ReadGSM = (IntTimeNow%10) == 0; // do the GSM calibration every 10 seconds
         double FracTimeNow = Now-IntTimeNow;
         double WaitTime = StartTime-FracTimeNow; if(WaitTime<0) WaitTime+=1.0;
         int SamplesToRead=SamplesPerRead;
         if( ReadGSM || (QueueSize()>1) ) SamplesToRead/=2; // when GSM calibration or data is not being processed fast enough we only read half-time
         if(WaitTime<0.200)
         { usleep((int)floor(1e6*WaitTime+0.5));                              // wait right before the time slot starts
           SampleBuffer<uint8_t> *Buffer = OutQueue.New();                    // get the next buffer to fill with raw I/Q data
           SDR.ResetBuffer();                                                 // needed before every Read()
           int Read=SDR.Read(*Buffer, SamplesToRead);                         // read the time slot raw RF data
           Buffer->Freq += CenterFreq * (1e-6*GSM_FreqCorr);                  // correct the frequency (sign ?)
           if(QueueSize()>1) printf("RF_Acq.Exec() ... Half time slot\n");
           // printf("RF_Acq.Exec() ... SDR.Read() => %d, Time=%16.3f, Freq=%6.1fMHz\n", Read, Buffer->Time, 1e-6*Buffer->Freq);
           if(Read>0) // RF data Read() successful
           { while(RawDataQueue.Size())
             { Socket *Client; RawDataQueue.Pop(Client);
               Client->Send("HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: audio/basic\r\n\r\n");
               Client->Send(Buffer->Data, Buffer->Full);
               Client->SendShutdown(); Client->Close(); delete Client; }
             if(OutQueue.Size()<4) { OutQueue.Push(Buffer); }
                              else { OutQueue.Recycle(Buffer); printf("RF_Acq.Exec() ... Dropped a slot\n"); }
           } else     // RF data Read() failed
           { SDR.Close(); printf("RF_Acq.Exec() ... SDR.Read() failed => SDR.Close()\n"); continue; }
           if(ReadGSM) // if we are to read GSM in the second half-slot
           { SDR.setTunerGain(GSM_Gain);             // setup for the GSM reception
             SDR.setCenterFreq(GSM_CenterFreq);
             SampleBuffer<uint8_t> *Buffer = GSM_OutQueue.New();
             SDR.ResetBuffer();
             int Read=SDR.Read(*Buffer, GSM_SamplesPerRead);
             // printf("RF_Acq.Exec() ...(GSM) SDR.Read() => %d, Time=%16.3f, Freq=%6.1fMHz\n", Read, Buffer->Time, 1e-6*Buffer->Freq);
             if(Read>0)
             { if(GSM_OutQueue.Size()<3) GSM_OutQueue.Push(Buffer);
                                  else { GSM_OutQueue.Recycle(Buffer); printf("RF_Acq.Exec() ... Dropped a GSM batch\n"); }
             }
             SDR.setTunerGain(Gain);                // back to OGN reception setup
             SDR.setCenterFreq(CenterFreq);
             if(GSM_Scan)
             { GSM_CenterFreq+=GSM_ScanStep;
               if(GSM_CenterFreq>=GSM_UppEdge) GSM_CenterFreq=GSM_LowEdge+GSM_ScanStep/2;
             }
           }
         }
         else usleep(100000);
       }
       else                                                                // if not open yet
       { int Index=(-1);
         if(DeviceSerial[0]) Index=SDR.getDeviceIndexBySerial(DeviceSerial);
         if(Index<0) Index=DeviceIndex;
         if(SDR.Open(Index, CenterFreq, SampleRate)<0)                    // try to open it
         { printf("RF_Acq.Exec() ... SDR.Open(%d, , ) fails, retry after 1 sec\n", Index); usleep(1000000); }
         else
         { SDR.setOffsetTuning(OffsetTuning); SDR.setFreqCorrection(FreqCorr); SDR.setTunerGainManual(); SDR.setTunerGain(Gain); }
       }
     }

     SDR.Close();
     // printf("RF_Acq.Exec() ... Stop\n");
     return  0; }

} ;

// ==================================================================================================

template <class Float>
 class Inp_FFT                                      // FFT of the RF data
{ public:

   Thread Thr;                                      // processing thread
   volatile int StopReq;
   RF_Acq *RF;

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

  public:
   Inp_FFT(RF_Acq *RF)
   { Window=0; this->RF=RF; Preset(); OutPipe=(-1); Config_Defaults(); }

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
    int Len=OutBuffer.Write(OutPipe);
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
       SampleBuffer<uint8_t> *InpBuffer = RF->OutQueue.Pop(); // here we wait for a new data batch
       // printf("Inp_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
       SlidingFFT(OutBuffer, *InpBuffer, FFT, Window);  // Process input samples, produce FFT spectra
       RF->OutQueue.Recycle(InpBuffer);
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

   int    Port;                   // listenning port
   Thread Thr;                    // processing thread
   RF_Acq             *RF;        // pointer to RF acquisition
   GSM_FFT<Float>     *GSM;

  public:
   HTTP_Server(RF_Acq *RF, GSM_FFT<Float> *GSM)
   { this->RF=RF; this->GSM=GSM;
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
     // else if( (strcmp(File, "/spectrogram.jpg")==0)     || (strcmp(File, "spectrogram.jpg")==0) )
     // { Demod->SpectrogramQueue.Push(Client); return; }
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
     Client->Send("\
<title>RTLSDR-OGN RF processor " STR(VERSION) " status</title>\n\
<b>RTLSDR OGN RF processor " STR(VERSION) " status</b><br /><br />\n\
");

     { dprintf(Client->SocketFile, "RF center frequency: %5.1fMHz, bandwidth: %3.1fMHz<br />\n", 1e-6*RF->CenterFreq, 1e-6*RF->SampleRate);
       // dprintf(Client->SocketFile, "RF channels: [%d]<br />\n", Demod->Demod.Channels);
       // dprintf(Client->SocketFile, "RF input noise: <b>%+3.1f</b>dB (ref. to internal noise of R820T with maximum gain)<br />\n",
       //        10*log10(Demod->LastBkgNoise/RefBkgNoise) );
       dprintf(Client->SocketFile, "RF frequency correction: <b>%+d</b>ppm (set in the USB receiver) <b>%+5.2f</b>ppm (measured against GSM reference)<br />\n",
                  RF->FreqCorr, RF->GSM_FreqCorr);
     }

     { Float Temperature;
       if(getTemperature_RasberryPi(Temperature)>=0)
         dprintf(Client->SocketFile, "CPU temperature: <b>%+3.1f</b>&#x2103;<br />\n", Temperature);
     }

     { Float Time, EstError, RefFreqCorr;
       if(getNTP(Time, EstError, RefFreqCorr)>=0)
       { dprintf(Client->SocketFile, "NTP est. error: <b>%3.1f</b>ms local clock correction: <b>%+5.2f</b>ppm<br />\n", 1e3*EstError, RefFreqCorr); }
     }

#ifndef __MACH__
     { struct sysinfo SysInfo;
       if(sysinfo(&SysInfo)>=0)
       { dprintf(Client->SocketFile, "CPU load [1/5/15min]: <b>%3.1f/%3.1f/%3.1f</b><br />\n",
                 SysInfo.loads[0]/65536.0, SysInfo.loads[1]/65536.0, SysInfo.loads[2]/65536.0);
         dprintf(Client->SocketFile, "RAM [free/total]: <b>%3.1f/%3.1f</b>MB<br />\n",
                 1e-6*SysInfo.freeram*SysInfo.mem_unit, 1e-6*SysInfo.totalram*SysInfo.mem_unit);
       }
     }
#endif

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

  Inp_FFT<float>     FFT(&RF);                   // FFT for OGN demodulator
  GSM_FFT<float>     GSM(&RF);                   // GSM frequency calibration

  HTTP_Server<float> HTTP(&RF, &GSM);            // HTTP server to show status and spectrograms

void SigHandler(int signum) // Signal handler, when user pressed Ctrl-C or process stops for whatever reason
{ RF.StopReq=1; }

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
  FFT.Config_Defaults();
  FFT.Config(&Config);
  FFT.Preset();
  GSM.Preset();

  HTTP.Config_Defaults();
  HTTP.Config(&Config);
  HTTP.Start();

  config_destroy(&Config);

  FFT.Start();
  GSM.Start();
  RF.Start();

  for( int t=0; /* t<1000 */ ; t++)
  { if(RF.StopReq) break;
    sleep(1); }

  sleep(4);
  RF.Stop();

  return 0; }


