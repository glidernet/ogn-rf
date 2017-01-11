#include <stdio.h>
#include <stdlib.h>

#include "r2fft.h"
#include "fft.h"

#define Float double

template <class Type>
 double Power(std::complex<Type> X)
{ return real(X)*real(X)+imag(X)*imag(X); }

int main(int argc, char *argv[])
{
  r2FFT<Float> FFT;
  DFT1d<Float> RefFFT;

  int FFTsize = 1024;
  std::complex<Float> Buffer[FFTsize];

  FFT.Preset(FFTsize);
  RefFFT.PresetForward(FFTsize);

  srand(123456);
  for(int Idx=0; Idx<FFTsize; Idx++)
  { Buffer[Idx] = std::complex<Float>((rand()&3)-1.5, (rand()&3)-1.5 );
    RefFFT.Buffer[Idx]=Buffer[Idx]; }

  FFT.Process(Buffer);
  RefFFT.Execute();

  double AverPower=0;
  for(int Idx=0; Idx<FFTsize; Idx++)
  { AverPower += Power(Buffer[Idx]); }
  AverPower/=FFTsize;
  printf("FFT: <Power> = %8.3f\n", AverPower);

  double RefAverPower=0;
  for(int Idx=0; Idx<FFTsize; Idx++)
  { RefAverPower += Power(RefFFT.Buffer[Idx]); }
  RefAverPower/=FFTsize;
  printf("RefFFT: <Power> = %8.3f\n", AverPower);

  double DiffAverPower=0;
  for(int Idx=0; Idx<FFTsize; Idx++)
  { DiffAverPower += Power(Buffer[Idx]-RefFFT.Buffer[Idx]); }
  DiffAverPower/=FFTsize;
  printf("FFT-RefFFT: <Power> = %8.3f\n", DiffAverPower);

  return 0; }


