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

#ifndef __R2FFT_H__
#define __R2FFT_H__

#include <stdint.h>
#include <math.h>
#include <complex>
#include <new>

template <class Float>
 class r2FFT // radix-2 FFT
{ public:    // size must a power of 2: 2,4,8,16,32,64,128,256,...

   int                  Size;          // FFT size (needs to be power of 2)
   int                 *BitRevIdx;     // Bit-reverse indexing table for data (un)scrambling
   std::complex<Float> *Twiddle;       // Twiddle factors (sine/cos values)

   r2FFT(int MaxSize)
   { BitRevIdx=0; Twiddle=0; Preset(MaxSize); }

   r2FFT()
   { BitRevIdx=0; Twiddle=0; Size=0; }

   ~r2FFT()
   { Free(); }

   void Free(void)
   { delete [] BitRevIdx; BitRevIdx=0;
     delete [] Twiddle; Twiddle=0; 
     Size=0; }

   // preset tables for given (maximum) processing size
   int Preset(int MaxSize)
   { Free();
     if(MaxSize<4) { return 0; }
     Size=MaxSize;
     while((MaxSize&1)==0) MaxSize>>=1;
     if(MaxSize!=1) { Size=0; return 0; }
     BitRevIdx = new (std::nothrow) int                 [Size]; if(BitRevIdx==0) { Free(); return 0; }
     Twiddle   = new (std::nothrow) std::complex<Float> [Size]; if(Twiddle==0)   { Free(); return 0; }
     // int Size4=Size/4;
     int Idx, rIdx, Mask, rMask;
     for(Idx=0; Idx<Size; Idx++)
     { double Phase=(2*M_PI*Idx)/Size;
       Twiddle[Idx] = std::complex<Float> ( cos(Phase), -sin(Phase) ); }
     // for(     ; Idx<Size; Idx++)
     // { Twiddle[Idx].Re=(-Twiddle[Idx-Size4].Im);
     //   Twiddle[Idx].Im=  Twiddle[Idx-Size4].Re ; }
     for(rIdx=0, Idx=0; Idx<Size; Idx++)
     { for(rIdx=0, Mask=Size/2,rMask=1; Mask; Mask>>=1,rMask<<=1)
       { if(Idx&Mask) rIdx|=rMask; }
       BitRevIdx[Idx]=rIdx; }
     return Size; }

   // scramble/unscramble (I)FFT input
   template <class Type>
    void Scramble(Type Data[])
     { for(int Idx=0; Idx<Size; Idx++)
       { int rIdx=BitRevIdx[Idx];
         if(rIdx>Idx)
         { Type Tmp=Data[Idx]; Data[Idx]=Data[rIdx]; Data[rIdx]=Tmp; }
       }
     }

   template <class Type>
    void Scramble(Type Data[], int ShrinkShift)
     { int Len=Size>>ShrinkShift;
       for(int Idx=0; Idx<Len; Idx++)
       { int rIdx=BitRevIdx[Idx]>>=ShrinkShift;
         if(rIdx>Idx)
         { Type Tmp=Data[Idx]; Data[Idx]=Data[rIdx]; Data[rIdx]=Tmp; }
       }
     }

   // core process: the classic tripple loop of butterflies
   // radix-2 FFT: the first and the second pass are by hand
   // looks like there is no gain by separating the second pass
   // and even the first pass is in question ?
   template <class Type>
    void CoreProc(std::complex<Type> Data[])
     { int Groups,GroupSize2,Group,Bf,TwidIdx;
       int Size2=Size/2;
       for(Bf=0; Bf<Size; Bf+=2) FFT2(Data[Bf],Data[Bf+1]); // first pass
       // for(Bf=0; Bf<Size; Bf+=4) FFT4(Data[Bf], Data[Bf+1], Data[Bf+2], Data[Bf+3]); // second
       // for(Groups=Size2/4,GroupSize2=4; Groups; Groups>>=1, GroupSize2<<=1)
       for(Groups=Size2/2,GroupSize2=2; Groups; Groups>>=1, GroupSize2<<=1)
         for(Group=0,Bf=0; Group<Groups; Group++,Bf+=GroupSize2)
           for(TwidIdx=0; TwidIdx<Size2; TwidIdx+=Groups,Bf++)
           { FFTbf(Data[Bf],Data[Bf+GroupSize2],Twiddle[TwidIdx]); }
     }

   // radix-2 FFT with a "shrink" factor
   template <class Type>
    void CoreProc(std::complex<Type> Data[], int ShrinkShift)
     { int Groups,GroupSize2,Group,Bf,TwidIdx,TwidIncr;
       int Len=Size>>ShrinkShift;
       int Size2=Size/2;
       int Len2=Len/2;
       for(Bf=0; Bf<Len; Bf+=2) FFT2(Data[Bf],Data[Bf+1]); // first pass
       // for(Bf=0; Bf<Len; Bf+=4) FFT4(Data[Bf],Data[Bf+1],Data[Bf+2],Data[Bf+3]); // second
       for(Groups=Len2/2,TwidIncr=Size2/2,GroupSize2=2;
           Groups;
           Groups>>=1, TwidIncr>>=1, GroupSize2<<=1)
         for(Group=0,Bf=0; Group<Groups; Group++,Bf+=GroupSize2)
           for(TwidIdx=0; TwidIdx<Size2; TwidIdx+=TwidIncr,Bf++)
           { FFTbf(Data[Bf],Data[Bf+GroupSize2],Twiddle[TwidIdx]); }
     }

   // complex FFT process in place, includes unscrambling
   template <class Type>
    int Process(std::complex<Type> Data[])
     { Scramble(Data); CoreProc(Data); return 0; }

   // find the "shrink" factor for processing batches smaller than declared by Preset()
   int FindShrinkShift(int Len)
     { int Shift;
       for(Shift=0; Len<Size; Shift++)
         Len<<=1;
       if (Len!=Size) return -1;
       return Shift; }

   // process data with length smaller than requested by Preset() (but still a power of 2)
   template <class Type>
    int Process(std::complex<Type> Data[], int Len)
     { if(Len<4) return -1;
       if(Len==Size) { Scramble(Data); CoreProc(Data); return 0; }
       int ShrinkShift=FindShrinkShift(Len); if(ShrinkShift<0) return -1;
       Scramble(Data,ShrinkShift); CoreProc(Data,ShrinkShift); return 0; }

   // classic radix-2 butterflies
   template <class Type>
    inline void FFTbf(std::complex<Type> &x0, std::complex<Type> &x1, std::complex<Float> &W)
     { std::complex<Float> x1W;
       x1W = x1 * W;
       x1 = x0 - x1W;
       x0 = x0 + x1W; }

   // special 2-point FFT for the first pass
   template <class Type>
    inline void FFT2(std::complex<Type> &x0, std::complex<Type> &x1)
     { std::complex<Type> x1W;
       x1W = x1;
       x1 = x0 - x1;
       x0 += x1W; }
/*
   // special 4-point FFT for the second pass
   template <class Type>
    inline void FFT4(std::complex<Type> &x0, std::complex<Type> &x1, std::complex<Type> &x2, std::complex<Type> &x3)
     { Type x1W;
       x1W.Re=x2.Re;
       x1W.Im=x2.Im;
       x2.Re=x0.Re-x1W.Re;
       x2.Im=x0.Im-x1W.Im;
       x0.Re=x0.Re+x1W.Re;
       x0.Im=x0.Im+x1W.Im;
       x1W.Re=x3.Im;
       x1W.Im=(-x3.Re);
       x3.Re=x1.Re-x1W.Re;
       x3.Im=x1.Im-x1W.Im;
       x1.Re=x1.Re+x1W.Re;
       x1.Im=x1.Im+x1W.Im; }
*/
  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }


} ;

#endif // __R2FFT_H__
