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

#ifndef __FFT_H__
#define __FFT_H__

#include <stdint.h>
#include <math.h>

// #include <cmath> // for M_PI in C++11 - no, does not work
#include <complex>
#include <new>

#include <fftw3.h>

// ===========================================================================================

template <class Float>
 class DFT1d
{ public:
   std::complex<Float> *Buffer; // input and output buffer
   fftw_plan            Plan;   // FFTW specific
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or backward (inverse)

  public:
   DFT1d() { Buffer=0; Plan=0; Size=0; Sign=0; }

  ~DFT1d() { Free(); }

  void Free(void)
   { if(Buffer) { fftw_destroy_plan(Plan); fftw_free(Buffer); Buffer=0; Size=0; Sign=0; } }

  int Preset(int Size, int Sign)
  { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
    Free();
    Buffer = (std::complex<Float> *)fftw_malloc(Size*sizeof(std::complex<Float>)); if(Buffer==0) return -1;
    Plan = fftw_plan_dft_1d(Size, (fftw_complex *)Buffer, (fftw_complex *)Buffer, Sign, FFTW_MEASURE);
    this->Size=Size; this->Sign=Sign; return Size; }

  int PresetForward(int Size) { return Preset(Size, FFTW_FORWARD); }
  int PresetBackward(int Size) { return Preset(Size, FFTW_BACKWARD); }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }

  std::complex<Float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(void) { return fftw_execute(Plan); }

  void PrintPlan(void) { fftw_print_plan(Plan); printf("\n"); }

} ;

// ----------------------------------------------------------------------------------------------

template <>
 class DFT1d <float>
{ public:
   std::complex<float> *Buffer;
   fftwf_plan           Plan;
   int                  Size;
   int                  Sign;

  public:
   DFT1d() { Buffer=0; Plan=0; Size=0; Sign=0; }

  ~DFT1d() { Free(); }

  void Free(void)
   { if(Buffer) { fftwf_destroy_plan(Plan); fftwf_free(Buffer); Buffer=0; Size=0; Sign=0; } }

  int Preset(int Size, int Sign)
  { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
    Free();
    Buffer = (std::complex<float> *)fftwf_malloc(Size*sizeof(std::complex<float>)); if(Buffer==0) return -1;
    Plan = fftwf_plan_dft_1d(Size, (fftwf_complex *)Buffer, (fftwf_complex *)Buffer, Sign, FFTW_MEASURE);
    this->Size=Size; this->Sign=Sign; return Size; }

  int PresetForward(int Size) { return Preset(Size, FFTW_FORWARD); }
  int PresetBackward(int Size) { return Preset(Size, FFTW_BACKWARD); }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }

  std::complex<float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(void) { return fftwf_execute(Plan); }

  void PrintPlan(void) { fftwf_print_plan(Plan); printf("\n"); }

} ;

// ===========================================================================================

template <class Float=double>
 class InpSlideFFT
{ public:
   DFT1d<Float>         FwdFFT;       // forward FFT
   int                  WindowSize;   // Window size = FFT size
   int                  SlideSize;    // slide step for sliding-window FFT
   Float               *Window;       // Window shape (Hanning)
   std::complex<Float> *Pipe;         // input circular buffer
   int                  Ptr;          // wrap-around input buffer pointer
   std::complex<Float> *Output;       // pointer to FFT spectra

  public:
   InpSlideFFT() { WindowSize=0; Window=0; Pipe=0; }
  ~InpSlideFFT() { Free(); }
   void Free(void)  { delete [] Window; delete [] Pipe; Window=0; Pipe=0; WindowSize=0; }

   int Size(void) const { return FwdFFT.Size; }
   int Preset(int Size)
   { // if(Size==WindowSize) return Size;
     Free();                                             // deallocate everything
     if(FwdFFT.PresetForward(Size)<0) return -1;         // setup forward FFT
     WindowSize=Size;
     Window = new (std::nothrow) Float               [WindowSize]; if(Window==0) return -1;
     Pipe   = new (std::nothrow) std::complex<Float> [WindowSize]; if(Pipe==0) return -1;
     SetSineWindow(); return Size; }                     // return FFT size (or negative when allocations failed)

   void Clear(void) { for(int Idx=0; Idx<WindowSize; Idx++) { Pipe[Idx]=0; } Ptr=WindowSize-SlideSize; }

   void SetHannWindow(int Slide=0)
   { Float Scale=1.0/sqrt(WindowSize);                  // scale factor (forward+backward FFT scale data up by size)
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*(1.0-cos((2*M_PI*Idx)/WindowSize)); }
     if(Slide==0) Slide=WindowSize/4;
     SlideSize=Slide; Clear(); }

   void SetSineWindow(int Slide=0)
   { Float Scale=1.0/sqrt(WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   if(Slide==0) Slide=WindowSize/2;
   SlideSize=Slide; Clear(); }

   void SetGaussWindow(double Sigma, int Slide)
   { int WindowSize2 = WindowSize/2;
     for(int Idx=0; Idx<WindowSize; Idx++)
     { double D=Idx-WindowSize2;
       Window[Idx]=exp(-(D*D)/(2*Sigma*Sigma)); }
   SlideSize=Slide; Clear(); }

   void PrintWindow(void)
   { printf("InpSlideFFT::Window[%d] =", WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
       printf(" %+5.3f", Window[Idx]);
     printf("\n"); }

   int Process(const uint8_t *Input, Float Bias=127.38)       // process exactly one slide [SlideSize] of samples
   { int Idx;
     if(Input)                                                    //
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter new samples into the Pipe
       { std::complex<Float> CmpxInput(Input[0]-Bias, Input[1]-Bias);
         Pipe[Ptr++] = CmpxInput; Input+=2;
         if(Ptr>=WindowSize) Ptr=0; }
     } else                                                       // if no Input given
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter zeros into the pipe
       { Pipe[Ptr++] = 0; if(Ptr>=WindowSize) Ptr=0; }
     }
     return ProcessWindow(); }

  template <class InpFloat>
   int Process(std::complex<InpFloat> *Input)                      // process exactly one slide [SlideSize] of samples
   { int Idx;
     if(Input)
     { for(Idx=0; Idx<SlideSize; Idx++)                            // enter new samples into the Pipe
       { Pipe[Ptr++] = Input[Idx]; if(Ptr>=WindowSize) Ptr=0; }
     } else
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter zeros into the pipe
       { Pipe[Ptr++] = 0; if(Ptr>=WindowSize) Ptr=0; }
     }
     return ProcessWindow(); }

   int ProcessWindow(void)
   { int Idx;
     for(Idx=0; Ptr<WindowSize; Idx++)                            // multiply by the Window and copy to FwdFFT buffer
     { FwdFFT[Idx] = Window[Idx]*Pipe[Ptr++]; }
     Ptr=0;
     for(     ; Idx<WindowSize; Idx++)
     { FwdFFT[Idx] = Window[Idx]*Pipe[Ptr++]; }
     FwdFFT.Execute();                                             // execute forward FFT
     Output = FwdFFT.Buffer;                                       // spectra in now in FwdFFT.Buffer
     return SlideSize; }

} ;

// ===========================================================================================

template <class Float=double>
 class OutSlideFFT
{ public:
   DFT1d<Float>         BwdFFT;       // backward FFT
   int                  WindowSize;   // Window size = FFT size
   int                  SlideSize;    // slide step for sliding-window FFT
   Float               *Window;       // Window shape (Hanning)
   std::complex<Float> *Pipe;         // output circular buffer
   int                  Ptr;          // wrap-around input buffer pointer
   std::complex<Float> *Input;        // here the input spectra are to be placed
   std::complex<Float> *Output;       // the output samples (beware of circular buffering)

  public:
   OutSlideFFT() { WindowSize=0; Window=0; Pipe=0; Input=0; Output=0; }
  ~OutSlideFFT() { Free(); }
   void Free(void)  { delete [] Window; delete [] Pipe; Window=0; Pipe=0; WindowSize=0; }

   int Size(void) const { return BwdFFT.Size; }
   int Preset(int Size)
   { // if(Size==WindowSize) return Size;                   // to avoid reallocations
     Free();                                             // deallocate everything
     if(BwdFFT.PresetBackward(Size)<0) return -1;        // setup forward FFT
     WindowSize=Size;
     Input = BwdFFT.Buffer;                              // here the input spectra is to be place
     Window = new (std::nothrow) Float               [WindowSize]; if(Window==0) return -1;
     Pipe   = new (std::nothrow) std::complex<Float> [WindowSize]; if(Pipe==0) return -1;
     SetSineWindow(); return Size; }                     // return FFT size (or negative when allocations failed)

   void Clear(void) { for(int Idx=0; Idx<WindowSize; Idx++) { Pipe[Idx]=0; } Ptr=WindowSize-SlideSize; Output=Pipe+Ptr; }

   void SetHannWindow(int Slide=0)
   { double Scale=0.5/sqrt(WindowSize);                  // scale factor (forward+backward FFT scale data up by size)
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*(1.0-cos((2*M_PI*Idx)/WindowSize)); }
     if(Slide==0) Slide=WindowSize/4;
     SlideSize=Slide; Clear(); }

   void SetSineWindow(int Slide=0)
   { double Scale=0.5/sqrt(WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   if(Slide==0) Slide=WindowSize/2;
   SlideSize=Slide; Clear(); }

   void SetGaussWindow(double Sigma, int Slide)
   { int WindowSize2 = WindowSize/2;
     for(int Idx=0; Idx<WindowSize; Idx++)
     { double D=Idx-WindowSize2;
       Window[Idx]=exp(-(D*D)/(2*Sigma*Sigma)); }
   SlideSize=Slide; Clear(); }

   void PrintWindow(void)
   { printf("OutSlideFFT::Window[%d] =", WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
       printf(" %+5.3f", Window[Idx]);
     printf("\n"); }

   int Process(void)                                     // spectra to be processed must be in Input
   { int Idx;
     BwdFFT.Execute();

     for(Idx=0; Idx<SlideSize; Idx++)
     { Pipe[Ptr++] = 0;
       if(Ptr>=WindowSize) Ptr=0; }

     Output = Pipe+Ptr;

     for(Idx=0; Ptr<WindowSize; Idx++)
     { Pipe[Ptr++] += Input[Idx]*Window[Idx]; }
     Ptr=0;
     for(     ; Idx<WindowSize; Idx++)
     { Pipe[Ptr++] += Input[Idx]*Window[Idx]; }

     return SlideSize; }

   template <class SpectraType>
    int Process(std::complex<SpectraType> *Spectra)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Input[Idx] = Spectra[Idx]; }
     return Process(); }

   template <class SpectraType, class MaskType>
    int Process(std::complex<SpectraType> *Spectra, MaskType *Mask)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Input[Idx] = Spectra[Idx]*Mask[Idx]; }
     return Process(); }

   template <class OutType>
    int GetOutput(std::complex<OutType> *Output, int Decimate=1) // Decimate must be a 1,2,4,8,16,...
   { int Idx, OutPtr = Ptr;
     for(Idx=0; Idx<SlideSize; Idx+=Decimate)
     { (*Output++)=Pipe[OutPtr]; OutPtr+=Decimate; if(OutPtr>=WindowSize) OutPtr-=WindowSize; }
     return SlideSize/Decimate; }

} ; 

// ===========================================================================================

#ifdef USE_RPI_GPU_FFT     // the following code is Raspberry PI specific

#include "mailbox.h"
#include "gpu_fft.h"

class RPI_GPU_FFT
{ public:

   struct GPU_FFT *FFT;
   int MailBox;
   int Size;
   int Sign;
   int Jobs;

  public:
   RPI_GPU_FFT()
   { MailBox=mbox_open(); FFT=0; Size=0; Sign=0; Jobs=0; }

  ~RPI_GPU_FFT()
   { Free(); mbox_close(MailBox); }

   void Free(void)
   { if(FFT==0) return;
     gpu_fft_release(FFT);
     FFT=0; Size=0; Sign=0; Jobs=0; }

   int Preset(int Size, int Sign, int Jobs=32)
   { if( FFT && (Size==this->Size) && (Sign==this->Sign) && (Jobs==this->Jobs) ) return Size;
     Free(); if(Size<256) return -1;
     int LogN;
     for(LogN=8; LogN<=22; LogN++)
     { if(Size==(1<<LogN)) break; }
     if(LogN>22) return -1;
     int Err=gpu_fft_prepare(MailBox, LogN, Sign, Jobs, &FFT);
     if(Err<0) { FFT=0; Size=0; return Err; } // -1 => firmware up todate ?, -2 => Size not supported ?, -3 => not enough GPU memory
     this->Size=Size; this->Sign=Sign; this->Jobs=Jobs; return Size; }

   int PresetForward (int Size, int Jobs=32) { return Preset(Size, GPU_FFT_FWD, Jobs); }
   int PresetBackward(int Size, int Jobs=32) { return Preset(Size, GPU_FFT_REV, Jobs); }

   std::complex<float> *Input (int Job=0) { return (std::complex<float> *)(FFT->in  + Job*FFT->step); }
   void Execute(void) { gpu_fft_execute(FFT); }
   std::complex<float> *Output(int Job=0) { return (std::complex<float> *)(FFT->out + Job*FFT->step); }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }

} ;

#endif

// ===========================================================================================

#endif // of  __FFT_H__
