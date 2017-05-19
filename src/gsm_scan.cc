#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include <math.h>

#include <vector>
#include <algorithm>

#include "rtlsdr.h"
#include "buffer.h"
#include "fft.h"
#include "image.h"

#define FloatType float

// ==================================================================================================================

const int GSM_ChannelWidth = 200000; // [Hz]
const int GSM_DataRate     = 270833; // [bps]

// ==================================================================================================================

   template <class Float>
    void AverPeakBkg(Float &Aver, Float &Peak, Float &PeakPos, Float &Bkg, Float *Data, int Size)
   { Aver=0; Peak=0; PeakPos=0; int PeakIdx=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Float Dat=Data[Idx];
       if(Dat>Peak) { Peak=Dat; PeakIdx=Idx; }
       Aver+=Dat; }
     if(PeakIdx==0)             { Peak+=Data[     1];                    PeakPos=PeakIdx+Data[     1]/Peak;                      Bkg=(Aver-Peak)/(Size-2); }
     else if(PeakPos==(Size-1)) { Peak+=Data[Size-2];                    PeakPos=PeakIdx-Data[Size-2]/Peak;                      Bkg=(Aver-Peak)/(Size-2); }
     else                       { Peak+=Data[PeakIdx+1]+Data[PeakIdx-1]; PeakPos=PeakIdx+(Data[PeakIdx+1]-Data[PeakIdx-1])/Peak; Bkg=(Aver-Peak)/(Size-3); }
     Aver/=Size; }

   template <class Float>
    void AverRMS(Float &Aver, Float &RMS, Float *Data, int Size) // classical average and RMS of a data series
   { Aver=0; RMS=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Aver+=Data[Idx]; }
     Aver/=Size;
     for(int Idx=0; Idx<Size; Idx++)
     { Float Diff=Data[Idx]-Aver; RMS+=Diff*Diff; }
     RMS=sqrt(RMS/Size); }

   template <class Float>
    int ProcessChan(std::vector<Float> &PPM_Values, Float &AverPower,
                    int LowBin, int UppBin, Float CenterBin, Float BinWidth, Float CenterFreq,
                    SampleBuffer<Float> &Power)
   { int Slides = Power.Samples();
     int Bins = Power.Len;
     SampleBuffer<Float> Aver, Peak, PeakPos, Bkg;
     Aver.Allocate(1, Slides); Peak.Allocate(1, Slides); PeakPos.Allocate(1, Slides); Bkg.Allocate(1, Slides);
     Float *Data = Power.Data;
     for(int Idx=0; Idx<Slides; Idx++, Data+=Bins)
     { AverPeakBkg(Aver[Idx], Peak[Idx], PeakPos[Idx], Bkg[Idx], Data+LowBin, UppBin-LowBin+1);
       PeakPos[Idx]+=LowBin-CenterBin; }
     Aver.Full=Slides; Peak.Full=Slides; PeakPos.Full=Slides; Bkg.Full=Slides;
     Float PowerRMS; AverRMS(AverPower, PowerRMS, Aver.Data, Slides);
     // printf("AverPower=%3.1f, PowerRMS=%3.1f\n", AverPower, PowerRMS);
     if(PowerRMS>(0.5*AverPower)) return 0;                                   // skip pulsing channels

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
       if(Peak.Data[Idx]<(32*Bkg.Data[Idx])) continue;
       Float PPM = -1e6*(PeakPos.Data[Idx]*BinWidth-(Float)GSM_DataRate/4)/CenterFreq;
       // printf("Mark: PeakSum[%5d]=%8.1f/%6.1f Bkg=%8.3f/%6.3f Peak/Bkg=%8.1f PeakPos=%+7.3f %+7.3fppm\n",
       //         Idx, PeakSum, PeakThres, 3*AverBkg-BkgSum, BkgThres, Peak.Data[Idx]/Bkg.Data[Idx], PeakPos.Data[Idx], PPM);
       PPM_Values.push_back(PPM);
       Marks++; }

     return Marks; }
/*
   template <class Float>
    int EstimatePPM(double &EstPPM, SampleBuffer<Float> &Power)
   { int Slides = Power.Samples();
     int Bins = Power.Len;
     double Sum[Bins];
     for(int Bin=0; Bin<Bins; Bin++)
     { Sum[Bin]=0; }
     Float *Data = Power.Data;
     for(int Slide=0; Slide<Slides; Slide++)
     { for(int Bin=0; Bin<Bins; Bin++)
       { Sum[Bin]+=Data[Bin]; }
       Data+=Bins; }
     double I=0, Q=0;
     for(int Bin=0; Bin<Bins; Bin++)
     { double Phase=(10.0*Bin)/Bins; Phase-=floor(Phase); Phase*=2*M_PI;
       double LogPwr = Sum[Bin];
       I += LogPwr*cos(Phase);
       Q += LogPwr*sin(Phase); }
     printf("EstimatePPM(%7.3fMHz, %dx%d) I/Q=%+8.1f/%+8.1f\n", 1e-6*Power.Freq, Slides, Bins, I, Q);
     return 0; }
*/
// ==================================================================================================================

  const char *OptionHelp = "\
  --device <device index>         [int] RTLSDR device index\n\
  --serial <serial number>        [string] RTLSDR device serial number\n\
  --ppm <crystal correction>      [ppm] receiver crystal correction\n\
  --gain <receiver gain>          [dB] receiver gain [default is auto-gain]\n\
  --offset-tuning                      enable offset tuning - for E4000 tuner\n\
  --gsm-850                            scan the GSM-850 band for USA and Canada (default is E-GSM for Europe)\n\
" ;

int main(int argc, char **argv)
{ 
  RTLSDR SDR;     // DVB-T device with the RTL2832U control chip

  int RxDevice       =         0;    // [Index] device index for RTLSDR device
  int FreqRaster     =     28125;    // [Hz] 28800000>>10
  int RxCrystalCorr  =         0;    // [PPM] crystal frequency correction for the DVB-T receiver
  int RxGain         =       200;    // [0.1dB] receiver gain - low default gain for the GSM band as signals are strong
  int RxOffsetTuning =         0;    // [bool]
  int SampleRate     =   2000000;    // [Hz] => single scan takes 10 GSM channels (0.2MHz/channel)
  int LowerFreq      = 920000000;    // [Hz] => scan whole E-GSM band
  int UpperFreq      = 960000000;    // [Hz]
  // int LowerFreq      = 868000000;    // [Hz] => scan whole GSM-850 band
  // int UpperFreq      = 900000000;    // [Hz]
  int GuardBand      =    100000;    // [Hz] first channel starts at 921.0 + 0.1 = 921.1 MHz
  int FFTsize        =      1024;    // [FFT bins] => FFT resolution = 3200000/1024 = ~6.4kHz/bin
  int SamplesPerScan =    500000;    // 0.250sec of RF data per scan

  int arg=1;
  for( ; arg<argc; )
  {
    if(memcmp(argv[arg],"--",2)==0)
    { char *OptName=argv[arg]+2;
      if(strcmp(OptName,"help")==0)
      { printf("Usage: %s [options]\n%s", argv[0], OptionHelp); exit(0); }
      else if(strcmp(OptName,"device")==0)
      { if(sscanf(argv[++arg], "%d", &RxDevice)!=1)
        { printf("Not a valid number: %s for option %s\n", argv[arg], OptName); exit(0); }
        arg++; }
      else if(strcmp(OptName,"serial")==0)
      { int Index=SDR.getDeviceIndexBySerial(argv[++arg]);
        if(Index<0)
        { printf("Device with serial %s not found (%d)\n", argv[arg], Index); exit(0); }
        RxDevice=Index;
        arg++; }
      else if(strcmp(OptName,"ppm")==0)
      { if(sscanf(argv[++arg], "%d", &RxCrystalCorr)!=1)
        { printf("Not a valid number: %s for option %s\n", argv[arg], OptName); exit(0); }
        arg++; }
      else if(strcmp(OptName,"gain")==0)
      { float Gain;
        if(sscanf(argv[++arg], "%f", &Gain)!=1)
        { printf("Not a valid number: %s for option %s\n", argv[arg], OptName); exit(0); }
        RxGain = (int)floor(10*Gain+0.5);
        arg++; }
      else if(strcmp(OptName,"offset-tuning")==0)
      { RxOffsetTuning=1;
        arg++; }
      else if(strcmp(OptName,"gsm-850")==0)
      { LowerFreq = 868000000; UpperFreq = 900000000;
        arg++; }
      else if( (strcmp(OptName,"sample-rate")==0) || (strcmp(OptName,"bandwidth")==0) )
      { if(sscanf(argv[++arg], "%d", &SampleRate)!=1)
        { printf("Not a valid number: %s for option %s\n", argv[arg], OptName); exit(0); }
        arg++; }
    }
    else if(argv[arg][0]=='-')
    { printf("Unknown option: %s\n", argv[arg]); exit(0);
    }
    else
    { printf("File names not allowed, only options\n"); exit(0);
      arg++; }
  }

  int CenterFreq     = (LowerFreq+UpperFreq)/2;                      // [Hz] center frequency of the scan
  int FreqStep       = SampleRate;                                   // [Hz] scanning step
  int FFTsize2       = FFTsize/2;
  int Scans          = (UpperFreq-LowerFreq-2*GuardBand+FreqStep-1)/FreqStep;    // number of scan to cover the desired band
      UpperFreq      =  LowerFreq+Scans*FreqStep+2*GuardBand;                    // [Hz]
      CenterFreq     = (LowerFreq+UpperFreq)/2;                                  // [Hz]

#ifdef USE_RPI_GPU_FFT
  RPI_GPU_FFT       FFT;
#else
  DFT1d<FloatType>  FFT;
#endif
  FloatType         Window[FFTsize];
  FFT.PresetForward(FFTsize); FFT.SetSineWindow(Window, FFTsize, (FloatType)(1.0/sqrt(FFTsize)) );

  printf("Frequency = %5.3fMHz..%5.3fMHz %5.3fMHz step, %d scans\n",
                       1e-6*LowerFreq, 1e-6*UpperFreq, 1e-6*FreqStep, Scans);
  printf("Sampling rate    = %10dHz = %8.3fMHz\n", SampleRate, 1e-6*SampleRate);
  printf("%5.3fsec per scan\n", (double)SamplesPerScan/SampleRate);
  printf("FFT: %d bins, %3.1fHz/bin, %5.3fms/slide\n",
          FFTsize, (double)SampleRate/FFTsize, 0.5e3*FFTsize/SampleRate);

  if(SDR.Open(RxDevice, CenterFreq, SampleRate)<0)        // open the RTLSDR device
  { printf("Can't open RTLSDR device #%d\n", RxDevice);  return 0; }
  printf("Open(%d, %d, %d) OK\n", RxDevice, CenterFreq, SampleRate);

  SDR.setFreqCorrection(RxCrystalCorr);          // [PPM] correct the crystal frequencies
  printf("Tuner crystal correction set to %d ppm\n", RxCrystalCorr);
  if(RxGain>=0)
  { SDR.setTunerGainManual(); SDR.setTunerGain(RxGain); printf("Tuner gain set to %3.1f dB (device reports %3.1f dB)\n", 0.1*RxGain, 0.1*SDR.getTunerGain() ); }
  else
  { SDR.setTunerGainAuto(); printf("Tuner gain set to automatic\n"); }  // automatic gain control

  SDR.setOffsetTuning(RxOffsetTuning); if(RxOffsetTuning) printf("Offset tuning activated\n");
  SDR.FreqRaster=FreqRaster;

  printf("\n");

  SampleBuffer<uint8_t>                   Input;
  SampleBuffer< std::complex<FloatType> > Spectra;
  SampleBuffer<FloatType>                 Power;
  std::vector<FloatType> PPM_Values;

  FloatType BinWidth     = SampleRate/FFTsize;

  int Freq=LowerFreq+GuardBand+FreqStep/2;
  for(int Scan=0; Scan<Scans; Scan++, Freq+=FreqStep)
  { SDR.setCenterFreq(Freq);
    int ActualFreq=SDR.getCenterFreq();
    SDR.ResetBuffer();
    int Samples=SDR.Read(Input, SamplesPerScan);                                    // acquire RF I/Q data
    // printf("SDR.Read(, %5.3fMHz) => %d samples\n", 1e-6*Freq, Samples);
    if(Samples<=0) { printf("SDR.Read(%5.1fMHz) failed\n", 1e-6*ActualFreq); continue; }
    SlidingFFT(Spectra, Input, FFT, Window);                                        // process with sliding FFT
    SpectraPower(Power, Spectra);                                                   // we only want the amplitudes (power)
#ifdef WRITE_SPECTROGRAM
    char FileName[64]; sprintf(FileName, "gsm_scan_%7.3fMHz-%7.3fMHz.jpg", 1e-6*(Power.Freq-FreqStep/2), 1e-6*(Power.Freq+FreqStep/2));
    MonoImage<FloatType> Spectrogram; Spectrogram.setExternal(Power.Data, FFTsize, Power.Samples() );
    Spectrogram.WriteJPG_8bpp(FileName, 80, 0.33, 32.0, 32.0);                      // write spectrogram file
#endif
    // double EstPPM=0;
    // EstimatePPM(EstPPM, Power);

    FloatType FirstBinFreq = ActualFreq-BinWidth*FFTsize2;    // [Hz] center frequency of the first FFT bin
    FloatType LastBinFreq  = ActualFreq+BinWidth*FFTsize2;    // [Hz] center frequency of the one-after-the-last FFT bin
    int Chan = (int)ceil(FirstBinFreq/GSM_ChannelWidth);      // integer channel number corr. to the first FFT bin (GSM channels are on multiples of 200kHz)
    for( ; ; Chan++)                                          // loop over (possible) channels in this scan
    { FloatType CenterFreq=Chan*GSM_ChannelWidth; if(CenterFreq>=LastBinFreq) break; // center frequency of the channel
      FloatType LowFreq = CenterFreq-0.45*GSM_ChannelWidth;    // [Hz] lower frequency to measure the channel
      FloatType UppFreq = CenterFreq+0.45*GSM_ChannelWidth;    // [Hz] upper frequency to measure the channel
      int LowBin=(int)floor((LowFreq-FirstBinFreq)/BinWidth+0.5); // FFT bins corresponding to the channel frequency range
      int UppBin=(int)floor((UppFreq-FirstBinFreq)/BinWidth+0.5);
      if( (LowBin<0) || (LowBin>=FFTsize) ) continue;          // skip this channel if range to measure
      if( (UppBin<0) || (UppBin>=FFTsize) ) continue;          // not contained completely in this scan
      FloatType AverPower; int Marks;
      Marks=ProcessChan(PPM_Values, AverPower,                 // measure the channel, add measured points to PPM_Values
                        LowBin, UppBin, (CenterFreq-FirstBinFreq)/BinWidth, BinWidth, CenterFreq,
                        Power);
      if(Marks==1) PPM_Values.pop_back();
      if(Marks>1)
      {  printf("%7.3fMHz: %+6.1fdB:",
                1e-6*CenterFreq, 10*log10(AverPower/0.33));
         for(size_t Mark=PPM_Values.size()-Marks; Mark<PPM_Values.size(); Mark++)
           printf(" %+6.2f", PPM_Values[Mark]);
         printf(" [ppm]\n");
        // printf("%7.3fMHz %2d:[%4d-%4d] %+6.1fdB %d marks\n",
        //         1e-6*CenterFreq, Scan, LowBin, UppBin, 10*log10(AverPower), Marks);
      }
    }
  }

  SDR.Close();

  std::sort(PPM_Values.begin(), PPM_Values.end());
  if(PPM_Values.size()>=16)
  { FloatType Aver, RMS; int Margin=PPM_Values.size()/8;
    AverRMS(Aver, RMS, PPM_Values.data()+Margin, PPM_Values.size()-2*Margin);
    if(RMS>0.3)
    { Margin=PPM_Values.size()/4; AverRMS(Aver, RMS, PPM_Values.data()+Margin, PPM_Values.size()-2*Margin); }
    printf("Receiver Xtal correction = %d%+6.3f = %+7.3f (%5.3f) ppm [%d]\n", RxCrystalCorr, Aver, RxCrystalCorr+Aver, RMS, (int)PPM_Values.size()-2*Margin);
    if(RMS>0.3)
      printf("Warning: measurements appear inconsistent:\nplease retry with better initial estimate or with lower gain to reduce distortions\n");
    printf("Note: when the receiver warms up the Xtal may drift 5-10ppm\n");
  } else
  { printf("Not enough data was collected: please retry with higher gain to catch more GSM signals\n"); }

  return 0; }

// ==================================================================================================================
