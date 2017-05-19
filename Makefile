VERSION = $(shell cat VERSION.txt)

FLAGS = -Wall -O3 -ffast-math -DVERSION=$(VERSION)
LDLIBS  =  -lpthread -lm -ljpeg -lconfig -lrt -lfftw3 -lfftw3f -lrtlsdr -ldl

prefix = /usr/local
#####

TARGETS = X64 ARM

ifeq ($(TARGET),)
  HOST_UNAME="$(shell uname -m)"
  ifeq ($(HOST_UNAME), "x86_64")
    TARGET = X64
  else ifeq ($(HOST_UNAME), "armv7l")
    TARGET = ARM
    ifneq ("$(wildcard /opt/vc/src/hello_pi/hello_fft)","")
      USE_RPI_GPU_FFT = 1
    endif
  else
    $(error Unknown host architecture: $(HOST_UNAME), manually specify TARGET from $(TARGETS))
  endif
else
  ifeq ($(filter $(TARGET),$(TARGETS)),)
    $(error Unknown target: $(TARGET))
  endif
endif


ifeq ($(TARGET),ARM)
  FLAGS += -mcpu=arm1176jzf-s -mtune=arm1176jzf-s -march=armv6zk -mfpu=vfp
  ifdef USE_RPI_GPU_FFT
    OGN_DECODE_PATH=rpi-gpu-bin
  else
    OGN_DECODE_PATH=arm-bin
  endif
else ifeq ($(TARGET), X64)
  OGN_DECODE_PATH=x64-bin
endif

ifdef USE_RPI_GPU_FFT
  LDLIBS += -ldl
  GPU_FLAGS = -DUSE_RPI_GPU_FFT
  GPU_SRC   = src/mailbox.c src/gpu_fft.c src/gpu_fft_base.c src/gpu_fft_twiddles.c src/gpu_fft_shaders.c
endif

#####

all: gsm_scan ogn-rf r2fft_test

ogn-rf: src/ogn-rf.cc src/thread.h src/rtlsdr.h src/fft.h src/buffer.h src/image.h
	g++ $(FLAGS) $(GPU_FLAGS) -o $@ $< $(GPU_SRC) $(LDLIBS)

gsm_scan: src/gsm_scan.cc src/fft.h src/buffer.h src/image.h
	g++ $(FLAGS) $(GPU_FLAGS) -o $@ $< $(GPU_SRC) $(LDLIBS)

r2fft_test:	src/r2fft_test.cc src/r2fft.h src/fft.h
	g++ $(FLAGS) $(GPU_FLAGS) -o $@ $< $(GPU_SRC) $(LDLIBS)

clean:
	$(RM) gsm_scan ogn-rf r2fft_test


install: ogn-rf gsm_scan
	install -D ogn-rf $(DESTDIR)$(prefix)/bin/ogn-rf
	install -D gsm_scan $(DESTDIR)$(prefix)/bin/gsm_scan
	install -D $(OGN_DECODE_PATH)/ogn-decode $(DESTDIR)$(prefix)/bin/ogn-decode

uninstall:
	$(RM) $(DESTDIR)$(prefix)/bin/ogn-rf
	$(RM) $(DESTDIR)$(prefix)/bin/gsm_scan
	$(RM) $(DESTDIR)$(prefix)/bin/ogn-decode

.PHONY: all clean install uninstall
