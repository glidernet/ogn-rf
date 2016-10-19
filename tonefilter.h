#include <string.h>

#include "buffer.h"
#include "boxfilter.h"

template <class Float>
 class ToneFilter
{ public:
   int              FFTsize;
   double           Threshold;

   DFT1d<Float>     FwdFFT;
   DFT1d<Float>     BwdFFT;
   Float           *Window;
   Float           *Sort;

   SampleBuffer< std::complex<Float> > SpectraBuffer;
   SampleBuffer<Float>                 SpectraPwr;

   const static int PulseBoxRadius = 17;
   const static int PulseBoxSize = 2*PulseBoxRadius+1;
   BoxPeakSum<Float> PulseBox;

   int Pulses;
   Float Duty;

  public:

   ToneFilter() { PulseBox.Preset(PulseBoxSize);
                  FFTsize=32768; Window=0; Sort=0; Duty=0; }

  ~ToneFilter() { if(Window) free(Window);
                  if(Sort)   free(Sort); }

   int Preset(void)
   { FwdFFT.PresetForward(FFTsize);
     BwdFFT.PresetBackward(FFTsize);
     Window=(Float *)realloc(Window, FFTsize*sizeof(Float));
     Sort  =(Float *)realloc(Sort,   FFTsize*sizeof(Float));
     FwdFFT.SetSineWindow(Window, FFTsize, (Float)(1.0/sqrt(FFTsize)) );
     return 1; }

   int Process(SampleBuffer< std::complex<Float> > *OutBuffer, SampleBuffer<uint8_t> *InpBuffer)
   { Pulses=0;
     SlidingFFT(SpectraBuffer, *InpBuffer, FwdFFT, Window);   // Process input samples, produce FFT spectra
     SpectraPower(SpectraPwr, SpectraBuffer);                 // calculate spectra power
                                                                // process spectra: remove coherent signals
     int FFTslides = SpectraBuffer.Samples();                 // number of FFT slides in the input spectra
     std::complex<Float> *Spectra = SpectraBuffer.Data;
                  Float  *Pwr     = SpectraPwr.Data;
     for(int Slide=0; Slide<FFTslides; Slide++)               // loop over FFT slides (or time)
     { Pulses+=Process(Spectra, Pwr); Spectra+=FFTsize; Pwr+=FFTsize; }

     ReconstrFFT(*OutBuffer, SpectraBuffer, BwdFFT, Window);  // reconstruct input samples
     OutBuffer->Crop(FFTsize/2, FFTsize/2);
     return Pulses; }
/*
     int Process(std::complex<Float> *Spectra, Float *Pwr)
     { int Pulses=0;
       memcpy(Sort, Pwr, FFTsize*sizeof(Float));
       std::nth_element(Sort, Sort+FFTsize/2, Sort+FFTsize);
       Float Median = Sort[FFTsize/2];
       Float Thres = Threshold*Median;
       for(int Bin=0; Bin<FFTsize; Bin++)                     // loop over frequency bins
       { Float Power = Pwr[Bin];
         if(Power>Thres) Spectra[Bin]*=sqrt(Thres/Power);
         Pulses++; }
       return Pulses; }
*/
     int Process(std::complex<Float> *Spectra, Float *Pwr)
     { PulseBox.Clear();
       int Pulses=0;
       int Bin;
       for(Bin=0; Bin<PulseBoxSize; Bin++)
       { PulseBox.Process(Pwr[Bin]); }
       for(     ; Bin<FFTsize;      Bin++)
       { if(PulseBox.isAtPeak())
         { Float PeakAmpl=PulseBox.PeakSum(1);
           Float BkgNoise=(PulseBox.Sum-PeakAmpl)/(PulseBoxSize-3);
           if(PeakAmpl>(Threshold*BkgNoise))
           { int PeakBin=Bin-PulseBoxRadius-1;
             Spectra[PeakBin]=0;
             Spectra[PeakBin+1]*=0.5;
             Spectra[PeakBin-1]*=0.5; }
         }
         PulseBox.Process(Pwr[Bin]); }
       return Pulses; }


} ;

