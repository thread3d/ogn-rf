VERSION = 0.2.6

# USE_RPI_GPU_FFT = 1

FLAGS = -Wall -O3 -ffast-math -DVERSION=$(VERSION)
LIBS  = -lpthread -lm -ljpeg -lconfig -lrt

ifneq ("$(wildcard /opt/vc/src/hello_pi/hello_fft)","")
USE_RPI_GPU_FFT = 1
FLAGS += -mcpu=arm1176jzf-s -mtune=arm1176jzf-s -march=armv6zk -mfpu=vfp
endif

#ifndef __MACH__ # _POSIX_TIMERS  # OSX has no clock_gettime() and thus not -lrt (but _POSIX_TIMERS seem not to be defined in make ?)
#LIBS += -lrt
#endif

ifdef USE_RPI_GPU_FFT
GPU_FLAGS = -DUSE_RPI_GPU_FFT
GPU_SRC   = mailbox.c gpu_fft_base.c gpu_fft.c gpu_fft_twiddles.c gpu_fft_shaders.c
LIBS += -ldl
endif

# LimeSDR support, through SoapySDR + the SoapyLMS7 module (see limesdr.h and sdr.h).
# Auto-detected from the SoapySDR headers, so an ordinary RTL-SDR build is unaffected;
# force it either way with "make USE_LIMESDR=1" or "make USE_LIMESDR=0".
ifndef USE_LIMESDR
ifneq ("$(wildcard /usr/include/SoapySDR/Device.hpp)","")
USE_LIMESDR = 1
endif
endif

ifeq ($(USE_LIMESDR),1)
# SDR_FLAGS is separate from FLAGS on purpose: a "make FLAGS=..." override on the command
# line would otherwise drop the define and produce a binary that cannot select a LimeSDR.
SDR_FLAGS  = -DUSE_LIMESDR
LIME_LIBS  = -lSoapySDR
endif

all:    gsm_scan ogn-rf r2fft_test

ogn-rf:       Makefile ogn-rf.cc sdr.h rtlsdr.h limesdr.h thread.h fft.h buffer.h image.h
	g++ $(FLAGS) $(SDR_FLAGS) $(GPU_FLAGS) -o ogn-rf ogn-rf.cc $(GPU_SRC) $(LIBS) -lrtlsdr -lfftw3 -lfftw3f $(LIME_LIBS)
ifdef USE_RPI_GPU_FFT
	sudo chown root ogn-rf
	sudo chmod a+s  ogn-rf
endif

gsm_scan:       Makefile gsm_scan.cc rtlsdr.h fft.h buffer.h image.h
	g++ $(FLAGS) $(GPU_FLAGS) -o gsm_scan gsm_scan.cc $(GPU_SRC) $(LIBS) -lrtlsdr -lfftw3 -lfftw3f
ifdef USE_RPI_GPU_FFT
	sudo chown root gsm_scan
	sudo chmod a+s gsm_scan
endif

r2fft_test:	Makefile r2fft_test.cc r2fft.h fft.h
	g++ $(FLAGS) -o r2fft_test r2fft_test.cc -lm -lfftw3 -lfftw3f
