VERSION = $(shell cat VERSION.txt)

FLAGS = -Wall -O3 -ffast-math -DVERSION=$(VERSION)
LDLIBS  =  -lpthread -lm -ljpeg -lconfig -lrt -lfftw3f -lrtlsdr

#####

ifneq ("$(wildcard /opt/vc/src/hello_pi/hello_fft)","")
TARGET=RASPBERRYPI
endif

ifeq ($(TARGET),RASPBERRYPI)
USE_RPI_GPU_FFT = 1
FLAGS += -mcpu=arm1176jzf-s -mtune=arm1176jzf-s -march=armv6zk -mfpu=vfp
endif

ifdef USE_RPI_GPU_FFT
GPU_FLAGS = -DUSE_RPI_GPU_FFT
GPU_SRC   = mailbox.c gpu_fft.c gpu_fft_twiddles.c gpu_fft_shaders.c
endif

#####

all: gsm_scan ogn-rf

ogn-rf: src/ogn-rf.cc src/thread.h src/rtlsdr.h src/fft.h src/buffer.h src/image.h
	g++ $(FLAGS) $(GPU_FLAGS) -o $@ $< $(GPU_SRC) $(LDLIBS)

gsm_scan: src/gsm_scan.cc src/fft.h src/buffer.h src/image.h
	g++ $(FLAGS) $(GPU_FLAGS) -o $@ $< $(GPU_SRC) $(LDLIBS)

clean:
	$(RM) gsm_scan ogn-rf

.PHONY: all clean
