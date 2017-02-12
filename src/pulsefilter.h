#include <stdint.h>

#include "buffer.h"
#include "boxfilter.h"

class PulseFilter
{ public:
   int Threshold;                                        // apply pulse filter to the the RF samples to remove wideband pulses like radar
   const static int PulseBoxRadius = 33;
   const static int PulseBoxSize = 2*PulseBoxRadius+1;
   BoxPeakSum<int32_t> PulseBox;
   int Pulses;
   float Duty;

  public:
   PulseFilter() { PulseBox.Preset(PulseBoxSize); Threshold=0; Pulses=0; Duty=0; }

   int Process(SampleBuffer<uint8_t> &Buffer, uint8_t Bias=127)
   { PulseBox.Clear(); Pulses=0;
     if(Threshold<=0) return 0;
     int Samples = Buffer.Samples();
     if(Samples<PulseBoxSize) return 0;
     // printf("PulseFilter::Process(Buffer[%d]) (%d)\n", Samples, Threshold);
     uint8_t *Data = Buffer.Data;
     int Idx;
     for(Idx=0; Idx<(2*PulseBoxSize); Idx+=2)
     { int32_t I = Data[Idx  ]-Bias;
       int32_t Q = Data[Idx+1]-Bias;
       int32_t Pwr = I*I + Q*Q;
       PulseBox.Process(Pwr); }
     for(    ; Idx<(2*Samples); Idx+=2)
     { int32_t I = Data[Idx  ]-Bias;
       int32_t Q = Data[Idx+1]-Bias;
       int32_t Pwr = I*I + Q*Q;
       PulseBox.Process(Pwr);
       if(PulseBox.isAtPeak())
       { int32_t PeakAmpl = PulseBox.PeakSum(1);
         int32_t BkgNoise = (PulseBox.Sum-PeakAmpl)/(PulseBoxSize-3);
         if(PeakAmpl>(Threshold*BkgNoise))
         { // printf("PulseFilter::Process() %06d: %4d+%4d+%4d+%4d+%4d=%5d/%2d\n",
           //         Idx/2, PulseBox.AtPeak(-2), PulseBox.AtPeak(-1), PulseBox.AtPeak(0), PulseBox.AtPeak(1), PulseBox.AtPeak(2), PeakAmpl, BkgNoise);
           int32_t Thres = PeakAmpl/2;
           int PeakIdx = Idx-2*PulseBoxRadius;
           // printf("PulseFilter::Process() [%+3d,%+3d] [%+3d,%+3d] [%+3d,%+3d]\n", Data[PeakIdx-2]-Bias, Data[PeakIdx-1]-Bias, Data[PeakIdx]-Bias, Data[PeakIdx+1]-Bias, Data[PeakIdx+2]-Bias, Data[PeakIdx+3]-Bias);
           SetZero(Data+PeakIdx, Bias);
           if(PulseBox.AtPeak(-1)>Thres)
           { SetZero(Data+(PeakIdx-2), Bias); SetHalf(Data+(PeakIdx-4), Bias); }
           else SetHalf(Data+(PeakIdx-2), Bias);
           if(PulseBox.AtPeak( 1)>Thres)
           { SetZero(Data+(PeakIdx+2), Bias); SetHalf(Data+(PeakIdx+4), Bias); }
           else SetHalf(Data+(PeakIdx+2), Bias);
           // printf("PulseFilter::Process() [%+3d,%+3d] [%+3d,%+3d] [%+3d,%+3d]\n", Data[PeakIdx-2]-Bias, Data[PeakIdx-1]-Bias, Data[PeakIdx]-Bias, Data[PeakIdx+1]-Bias, Data[PeakIdx+2]-Bias, Data[PeakIdx+3]-Bias);
           Pulses++; }
       }
     }
     // printf("PulseFilter::Process(Buffer[%d]) (%d)  => %d pulses\n", Samples, Threshold, Pulses);
     Duty = (float)Pulses/Samples;
     return Pulses; }

   static void SetZero(uint8_t *Data, uint8_t Bias=127)
   { Data[0]=Bias; Data[1]=Bias; }

   static void SetHalf(uint8_t *Data, uint8_t Bias=127)
   { int8_t I = Data[0]-Bias; Data[0] = Bias + (I>>1);
     int8_t Q = Data[1]-Bias; Data[1] = Bias + (Q>>1); }

} ;


