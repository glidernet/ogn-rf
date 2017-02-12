#ifndef __BOXFILTER_H__
#define __BOXFILTER_H__

#include <stdlib.h>
#include <math.h>

#include "alloc.h"

// ===========================================================================================

template <class Type>
 class BoxPeakSum
{ public:
   int   Size;       // How many samples to remember
   Type *Pipe;       // storage for Size most recent signals
   int   Ptr;        // most recent signal position
   Type  Peak;       // max. signal value
   int   PeakPtr;    // position of max. signal
   Type  Sum;        // sum of all signals in the Pipe

  public:
   BoxPeakSum()  { Size=0; Pipe=0; Clear(); }
  ~BoxPeakSum()  { Free(); }
   void Free(void) { free(Pipe); Size=0; Pipe=0; Clear(); }

   void Print(void)
   { printf("BoxPeakSum[%d]", Size);
     for(int Idx=0; Idx<Size; Idx++)
     { printf(" %3.1f", Pipe[Idx]); }
     printf(" %3.1f/%3.1f %d/%d\n", Peak, Sum, PeakPtr, Ptr); }

   int Preset(int Size)
   { Free();
     this->Size=Size;
     if(Malloc(Pipe, Size)<=0) return -1;
     Clear(); return Size; }

   void Clear(void)
   { Ptr=0; Peak=0; PeakPtr=0; Sum=0; for(int Idx=0; Idx<Size; Idx++) Pipe[Idx]=0; }

   void ReCalc(void)
   { Sum=0; Peak=Pipe[0]; PeakPtr=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Type Sig=Pipe[Idx]; Sum+=Sig;
       if(Sig>Peak) { Peak=Sig; PeakPtr=Idx; }
     }
   }

   // Type operator[](int Idx)
   // { Idx+=PeakPtr; if(Idx>=Size) Idx-=Size; return Pipe[Idx]; }

   Type GetPipe(int Delay=0)
   { int Idx = Ptr-Delay; if(Idx<0) Idx+=Size; return Pipe[Idx]; }

   void Process(Type Input)
   { Ptr+=1; if(Ptr>=Size) Ptr=0;
     Sum -=  Pipe[Ptr];
     Sum += (Pipe[Ptr]=Input);
     if(Input>Peak) { Peak=Input; PeakPtr=Ptr; }
     if((Ptr==0) || (Ptr==PeakPtr) ) ReCalc(); }

   int isAtPeak(void)
   { int AntiPtr=Ptr-Size/2; if(AntiPtr<0) AntiPtr+=Size;
     return AntiPtr==PeakPtr; } // return 1 when peak is in the center of the pipeline

   Type AtPeak(int Ofs=0)
   { int Idx=PeakPtr+Ofs;
     if(Idx>=Size) Idx-=Size;
     else if(Idx<0) Idx+=Size;
     return Pipe[Idx]; }

   Type PeakSum(int Radius=0)
   { Type Sum=0; if(Radius>(Size/2)) Radius=Size/2;
     int Count=2*Radius+1;
     int Idx=PeakPtr-Radius; if(Idx<0) Idx+=Size;
     for( ; Count>0; Count--)
     { Sum+=Pipe[Idx++]; if(Idx>=Size) Idx-=Size; }
     return Sum; }

   template <class FitType>
     int FitGauss(FitType &Peak, FitType &Pos, FitType &Sigma, Type Bias=0, int LogFit=0)
   { int Idx = Ptr-PeakPtr; if(Idx<0) Idx+=Size; // Idx = distance from the current sample to the peak
     Peak=this->Peak; Pos=(-Idx); Sigma=0; if( (Idx==0) || (Idx==(Size-1)) ) return -1;
     Idx=PeakPtr;                              Type C=Pipe[Idx]-Bias; if(C<=0) return -2; // center
     Idx=(PeakPtr-1); if(Idx<0) Idx+=Size;     Type L=Pipe[Idx]-Bias; if(L<=0) return -2; // left (before center)
     Idx=(PeakPtr+1); if(Idx>=Size) Idx-=Size; Type R=Pipe[Idx]-Bias; if(R<=0) return -2; // right (after center)
     if(LogFit) { C=log(C); R=log(R); L=log(L); }
     FitType A = (R+L)/2-C; if(A>=0) return -3;     // parabole coeff: A, B, C
     FitType B = (R-L)/2;
     FitType D = B*B-4*A*C;                         // Delta
     Peak = (-D/(4*A)); if(LogFit) Peak=exp(Peak); // interpolated maximum value
     Pos += -B/(2*A);                              // interpolated position
     if(LogFit) Sigma = sqrt(-0.5/A);              // interpolated width
           else Sigma = sqrt(-0.5*Peak/A);
     return 3; }

   int FitTriangle(double &Peak, double &Pos)
   { int Idx = Ptr-PeakPtr; if(Idx<0) Idx+=Size; // Idx = distance from the current sample to the peak
     Peak=this->Peak; Pos=(-Idx); if( (Idx<2) || (Idx>=(Size-2)) ) return -1;
     Idx=PeakPtr;                              Type C=Pipe[Idx]; // center
     Idx=(PeakPtr-1); if(Idx<0) Idx+=Size;     Type L=Pipe[Idx]; // left (before center)
     Idx=(PeakPtr+1); if(Idx>=Size) Idx-=Size; Type R=Pipe[Idx]; // right (after center)
     if(R==L) return 2;
     if(R>L)
     { Idx=(PeakPtr+2); if(Idx>=Size) Idx-=Size; Type RR=Pipe[Idx]; // next right
       double X = (-C+2*R-RR)/(-L+C+R-RR); Peak=C+X*(C-L); Pos+=X; }
     else
     { Idx=(PeakPtr-2); if(Idx<0) Idx+=Size;     Type LL=Pipe[Idx]; // next left
       double X = (-LL+2*L-C)/(LL-L-C+R); Peak=C+X*(R-C); Pos+=X; }
     return 2; }

} ;

// ===========================================================================================

template <class Type, class SumType=Type>
 class BoxSumFilter
{ public:
   int   Size;            // box size
   Type *Pipe;            // box storage
   int   Ptr;             // pipe pointer
   SumType Output;        // filter output = sum of all elements in the box
   int   RoundsPerRecalc; // how often to recalculate
   int   RoundsToRecalc;  // recalc. counter

  public:
   BoxSumFilter() { Size=0; Pipe=0; Clear(); }
  ~BoxSumFilter() { Free(); }
   void Free(void) { free(Pipe); Size=0; Pipe=0; }

   int Preset(int Size, int RoundsPerRecalc=4)
   { Free();
     this->Size=Size; this->RoundsPerRecalc=RoundsPerRecalc;
     if(Malloc(Pipe, Size)<=0) return -1;
     Clear(); return Size; }

   void Clear(Type Zero=0)
   { Ptr=0; Output=Size*(SumType)Zero; for(int Idx=0; Idx<Size; Idx++) Pipe[Idx]=Zero; RoundsToRecalc=RoundsPerRecalc; }

   void ReCalc(void)
   { Output=0; for(int Idx=0; Idx<Size; Idx++) Output+=Pipe[Idx]; }

   Type Process(Type Input)
   { Ptr+=1; if(Ptr>=Size) Ptr=0;
     Output-=Pipe[Ptr];
     Output+=(Pipe[Ptr]=Input);
     if( (Ptr==0) && RoundsPerRecalc )
     { RoundsToRecalc--;
       if(RoundsToRecalc==0)
       { ReCalc(); RoundsToRecalc=RoundsPerRecalc; }
     }
     return Output; }

   SumType Average(void) { return Output/Size; }
   Type operator[](int Delay) { int Idx=Ptr-Delay; if(Idx<0) Idx+=Size; return Pipe[Idx]; }
} ;


// ===========================================================================================

#endif // __BOXFILTER_H__
