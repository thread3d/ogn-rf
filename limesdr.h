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

// ==================================================================================================
//
// LimeSDR (LMS7002M) backend, through SoapySDR + the SoapyLMS7 support module.
//
// This class deliberately mirrors the public interface of RTLSDR (rtlsdr.h) so that the
// acquisition code in ogn-rf.cc can drive either radio without knowing which one it has.
// The one thing that matters to the rest of ogn-rf is the sample format: RTLSDR hands over
// interleaved 8-bit UNSIGNED I/Q with 128 as zero, and this class converts the 12-bit
// LMS7002M samples into exactly that shape, so everything downstream (pulse filter, FFT,
// OGN demodulator) is unchanged.
//
// Notes on the mapping, because several RTL-SDR concepts have no LMS7002M equivalent:
//
//   * Frequency correction (ppm) - the LMS7002M has no tuner crystal to trim, so the ppm
//     correction is applied in software by tuning the LO to freq*(1+ppm*1e-6). The nominal
//     (uncorrected) frequency is what getCenterFreq() reports, so the reported OGN channel
//     stays the nominal one.
//   * Bias tee - the LimeSDR-USB has no on-board bias tee, so setBiasTee() only records the
//     request and reports success; the config option is accepted and ignored.
//   * Offset tuning - an RTL-SDR/E4000 trick that has no meaning here; setOffsetTuning() is
//     accepted and ignored.
//   * getTunerGains() - there is no discrete tuner gain table; a list is synthesised from the
//     device's own gain range so the status page and config validation keep working.
//
// ==================================================================================================

#ifndef __LIMESDR_H__
#define __LIMESDR_H__

#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <time.h>
#include <sys/time.h>

#include <math.h>

#include <string>
#include <vector>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>

#include "asciitime.h"
#include "thread.h"

#include "buffer.h"

class LimeSDR
{ public:
   MutEx         Lock;              // guards the SoapySDR handle against a concurrent Close()

   uint32_t      DeviceIndex;       // index of the LimeSDR among the enumerated LimeSDRs
   char          DeviceSerial[64];  // serial of the device this object holds open
   SoapySDR::Device *Device;        // SoapySDR device handle (0 when closed)
   SoapySDR::Stream *Stream;        // SoapySDR RX stream (0 when closed)

   enum { Channel = 0 };            // which RX channel to use: 0 is RX0 on a LimeSDR-USB

   // The LMS7002M converter tops out at 61.44 MSps (2 x the 30.72 MHz reference). SoapyLMS7
   // advertises a little more than that (0.1-65 MSps on this board) and rejects 80 MSps, but
   // 61.44 is the figure the clocking is built around and the one an oversampling ratio has to
   // respect.
   static double ConverterMaxRate(void) { return 61440000.0; }
   std::string   HardwareKey;       // "LimeSDR-USB", cached: getHardwareKey() returns by value
   std::string   DeviceLabel;       // "LimeSDR-USB [USB 2.0] 9072...", cached for the same reason

   int           Mtu;               // stream MTU [samples] as reported by the driver
   int16_t      *Scratch;           // conversion buffer, 2*Mtu int16 samples
   int           Shift;             // right shift CS16 -> 8-bit (see the note in Read())
   int           ReadTimeoutUs;     // per readStream() timeout [us]
   int           MaxTimeouts;       // give up after this many consecutive timeouts

   int           Gains;             // number of entries in Gain[]
   int           Gain[64];          // [0.1 dB] synthesised gain table

   uint32_t      FreqRaster;        // [Hz] use only multiples of this base value to avoid tuning errors

   int           Oversample;        // requested ADC-to-host rate ratio: 0 = driver decides,
                                    // N = capture N x rate and decimate by N inside the LMS7002M
   int           OversampleApplied; // what the device actually accepted (0 = driver decided)

   uint32_t      Freq;              // [Hz] nominal (uncorrected) centre frequency
   int           FreqCorr;          // [ppm] software frequency correction

   int           GainMode;          // last requested gain mode: 0=Auto, >=1=Manual
   double        GaindB;            // [dB] gain actually applied
   int           BiasTee;           // [bool] requested bias tee state (not supported by the hardware)

   int           Antenna;           // index into AntennaName[] of the selected LNA path
   char          AntennaName[8][32];

   uint64_t      BytesRead;         // counts bytes converted (1 sample = 2 bytes: I/Q)
   double        SamplePeriod;      // [sec] time per sample, as reported by the device

#ifdef __MACH__
   double        RefTime;           // not used, kept so the layout matches RTLSDR
#endif

  public:
   LimeSDR()
   { DeviceIndex=0; DeviceSerial[0]=0; Device=0; Stream=0;
     Mtu=0; Scratch=0; Shift=8; ReadTimeoutUs=250000; MaxTimeouts=8;
     Gains=0; FreqRaster=0; Freq=0; FreqCorr=0; Oversample=32; OversampleApplied=0;
     GainMode=1; GaindB=0; BiasTee=(-1);
     Antenna=(-1); AntennaName[0][0]=0;
     BytesRead=0; SamplePeriod=1e-6; }

  ~LimeSDR()
   { Close(); }

   bool isOpen(void) const { return Device!=0; }

   // -----------------------------------------------------------------------------------------------
   // Open / Close
   // -----------------------------------------------------------------------------------------------

   // List of the LimeSDR devices the driver can see, in SoapySDR enumeration order
   static SoapySDR::KwargsList enumerate(void)
   { try { return SoapySDR::Device::enumerate("driver=lime"); }
     catch(const std::exception &Ex) { printf("LimeSDR::enumerate() => %s\n", Ex.what()); }
     return SoapySDR::KwargsList(); }

   static int getNumberOfDevices(void) { return (int)enumerate().size(); }

   // Find the index of the device with the given serial, -1 when there is no such device
   int getDeviceIndexBySerial(const char *Serial)
   { if(Serial==0) return -1;
     if(isOpen() && (strcmp(DeviceSerial, Serial)==0)) return (int)DeviceIndex;
     SoapySDR::KwargsList List=enumerate();
     for(size_t Idx=0; Idx<List.size(); Idx++)
     { SoapySDR::Kwargs::const_iterator It=List[Idx].find("serial");
       if( (It!=List[Idx].end()) && (strcmp(It->second.c_str(), Serial)==0) ) return (int)Idx; }
     return -1; }

   void Close(void)
   { if(Device)
     { if(Stream)
       { try { Device->deactivateStream(Stream); } catch(const std::exception &Ex) {}
         try { Device->closeStream(Stream); }      catch(const std::exception &Ex) {}
         Stream=0; }
       try { SoapySDR::Device::unmake(Device); }   catch(const std::exception &Ex) {}
       Device=0; }
     if(Scratch) { free(Scratch); Scratch=0; }
     Mtu=0; Gains=0; Antenna=(-1); DeviceSerial[0]=0; }

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868000000, uint32_t SampleRate=1024000)
   { Close();

     SoapySDR::KwargsList List=enumerate();
     if(List.empty())
     { printf("LimeSDR::Open() => no LimeSDR found (is SoapyLMS7 installed and the device plugged in ?)\n"); return -1; }
     if(DeviceIndex>=List.size())
     { printf("LimeSDR::Open() => device #%d requested but only %d present\n", DeviceIndex, (int)List.size()); return -1; }

     SoapySDR::Kwargs Kw=List[DeviceIndex];
     Kw["driver"]="lime";
     try { Device=SoapySDR::Device::make(Kw); }
     catch(const std::exception &Ex)
     { printf("LimeSDR::Open() => cannot open device #%d: %s\n", DeviceIndex, Ex.what()); Device=0; return -1; }

     this->DeviceIndex=DeviceIndex;
     try { HardwareKey=Device->getHardwareKey(); } catch(const std::exception &Ex) { HardwareKey="LimeSDR"; }
     SoapySDR::Kwargs::const_iterator It=Kw.find("serial");
     if(It!=Kw.end()) { strncpy(DeviceSerial, It->second.c_str(), 64); DeviceSerial[63]=0; }
     else DeviceSerial[0]=0;
     It=Kw.find("label");
     DeviceLabel.assign( (It!=Kw.end()) ? It->second : HardwareKey );

     try
     {
       // Sample rate configuration, with the on-chip capture/decimate ratio applied first.
       //
       // Oversampling decides how the LMS7002M reaches the host rate: with a ratio of N the
       // converter runs at N x the host rate and the on-chip CIC/FIR chain decimates by N, so
       // ogn-rf still receives SampleRate while the chip captures N times faster. It has to be
       // set BEFORE setSampleRate(), which is what configures the chain.
       //
       // The default asks for the highest ratio the converter can actually use (see
       // RF.Oversample). If a driver or a gateware build cannot deliver the host rate with that
       // ratio, the open is retried once with the ratio left to the driver rather than failing:
       // a rate the rest of ogn-rf depends on matters more than the ratio.
       double ActualRate = 0;
       int    AppliedRatio = 0;
       bool   Configured = false;

       // Step the requested ratio down to the largest power of two the converter can still run
       // at, given the host rate. At 1 MSps that leaves 32; at 2 MSps it leaves 16, because
       // 32 x 2 MSps would ask for a 64 MSps converter.
       int Wanted=Oversample;
       if(Wanted>0)
       { int MaxRatio=(int)(ConverterMaxRate()/(double)SampleRate);
         if(MaxRatio<1) MaxRatio=1;
         while( (Wanted>1) && (Wanted>MaxRatio) ) Wanted>>=1;
         if(Wanted!=Oversample)
           printf("LimeSDR::Open() => oversampling %d x %d Hz exceeds the %.2f MSps converter, using %d\n",
                  Oversample, SampleRate, 1e-6*ConverterMaxRate(), Wanted); }

       if(Wanted>0)
       { try
         { Device->writeSetting("OVERSAMPLING", std::to_string(Wanted));
           Device->setSampleRate(SOAPY_SDR_RX, Channel, (double)SampleRate);
           ActualRate=Device->getSampleRate(SOAPY_SDR_RX, Channel);
           if(fabs(ActualRate-SampleRate)<=0.5) { AppliedRatio=Wanted; Configured=true; }
           else printf("LimeSDR::Open() => OVERSAMPLING=%d gave %8.1f Hz for %d Hz, retrying with auto\n",
                       Wanted, ActualRate, SampleRate); }
         catch(const std::exception &Ex)
         { printf("LimeSDR::Open() => OVERSAMPLING=%d rejected (%s), retrying with auto\n", Wanted, Ex.what()); } }

       if(!Configured)
       { try { Device->writeSetting("OVERSAMPLING", "0"); } catch(const std::exception &Ex) {}
         Device->setSampleRate(SOAPY_SDR_RX, Channel, (double)SampleRate);
         ActualRate=Device->getSampleRate(SOAPY_SDR_RX, Channel);
         AppliedRatio=0; }

       if(fabs(ActualRate-SampleRate)>0.5)
         printf("LimeSDR::Open() => sample rate %d asked, %8.1f Hz delivered\n", SampleRate, ActualRate);
       SamplePeriod=1.0/ActualRate;
       OversampleApplied=AppliedRatio;

       // Select the LNA path that matches the frequency, when the driver lets us choose
       SelectAntenna((double)Frequency);

       // Gain: manual, at the value the caller asked for through setTunerGain()
       Device->setGainMode(SOAPY_SDR_RX, Channel, false);
       BuildGainTable();

       FreqRaster=0;                                     // caller sets it before setCenterFreq()
       Freq=Frequency;
       ApplyFrequency();

       Stream=Device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, std::vector<size_t>(1, Channel));
       if(Stream==0)
       { printf("LimeSDR::Open() => setupStream(CS16) returned no stream\n"); Close(); return -1; }
       Mtu=(int)Device->getStreamMTU(Stream);
       if(Mtu<=0) Mtu=4096;
       Scratch=(int16_t *)malloc(2*Mtu*sizeof(int16_t));
       if(Scratch==0) { printf("LimeSDR::Open() => cannot allocate %d samples\n", 2*Mtu); Close(); return -1; }

       Device->activateStream(Stream);
     }
     catch(const std::exception &Ex)
     { printf("LimeSDR::Open() => %s\n", Ex.what()); Close(); return -1; }

     printf("LimeSDR::Open(%d,%d,%d) => %s, %8.3f MHz, %5.3f Msps, MTU=%d, oversampling %d\n",
            DeviceIndex, Frequency, SampleRate, getDeviceName(), 1e-6*getCenterFreq(), 1e-6*getSampleRate(), Mtu,
            OversampleApplied>0 ? OversampleApplied : 0);
     PrintGains();
     return 1; }

   // -----------------------------------------------------------------------------------------------
   // Name / identification
   // -----------------------------------------------------------------------------------------------

   const char *getDeviceName(void) const
   { if(Device==0) return "LimeSDR";
     return HardwareKey.c_str(); }

   static const char *getDeviceName(uint32_t DeviceIndex)
   { static std::string Name;                                 // getHardwareKey() returns by value
     SoapySDR::KwargsList List=enumerate();
     if(DeviceIndex>=List.size()) return "LimeSDR";
     SoapySDR::Kwargs::const_iterator It=List[DeviceIndex].find("label");
     Name=(It!=List[DeviceIndex].end()) ? It->second : std::string("LimeSDR");
     return Name.c_str(); }

   int getTunerType(void) const { return 0; }                 // there is no separate tuner chip
   const char *getTunerTypeName(void) const
   { if(Device==0) return "LMS7002M";
     return HardwareKey.c_str(); }

   int getUsbStrings(char *Manufacturer, char *Product, char *Serial)
   { if(Device==0) return -1;
     SoapySDR::Kwargs Info=Device->getHardwareInfo();
     SoapySDR::Kwargs::const_iterator It;
     strcpy(Manufacturer, "Myriad-RF");
     It=Info.find("label");  strncpy(Product, (It!=Info.end()) ? It->second.c_str() : getDeviceName(), 255); Product[255]=0;
     It=Info.find("serial"); strncpy(Serial,  (It!=Info.end()) ? It->second.c_str() : DeviceSerial, 255);    Serial[255]=0;
     return 0; }

   // The LimeSDR-USB is clocked from a 30.72 MHz oscillator (or an external reference)
   int getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq) const
   { RtlFreq=30720000; TunerFreq=30720000; return 0; }

   // -----------------------------------------------------------------------------------------------
   // Frequency
   // -----------------------------------------------------------------------------------------------

   int setCenterFreq(uint32_t Frequency)
   { if(FreqRaster) { uint32_t Div=(Frequency+(FreqRaster>>1))/FreqRaster; Frequency=Div*FreqRaster; }
     Freq=Frequency;
     return ApplyFrequency(); }

   uint32_t getCenterFreq(void) const { return Freq; }        // nominal, i.e. before FreqCorr

   int    setFreqCorrection(int PPM) { FreqCorr=PPM; return ApplyFrequency(); }
   int    getFreqCorrection(void) const { return FreqCorr; }

   int ApplyFrequency(void)
   { if(Device==0) return -1;
     double LO=(double)Freq;
     if(FreqCorr) LO*=1.0+1e-6*FreqCorr;                       // ppm correction, in software
     try { Device->setFrequency(SOAPY_SDR_RX, Channel, LO); }
     catch(const std::exception &Ex) { printf("LimeSDR::setFrequency(%8.3f MHz) => %s\n", 1e-6*LO, Ex.what()); return -1; }
     return 1; }

   // -----------------------------------------------------------------------------------------------
   // Sample rate
   // -----------------------------------------------------------------------------------------------

   int setSampleRate(uint32_t SampleRate)
   { if(Device==0) return -1;
     try { Device->setSampleRate(SOAPY_SDR_RX, Channel, (double)SampleRate); }
     catch(const std::exception &Ex) { printf("LimeSDR::setSampleRate() => %s\n", Ex.what()); return -1; }
     SamplePeriod=1.0/Device->getSampleRate(SOAPY_SDR_RX, Channel);
     return 1; }

   uint32_t getSampleRate(void) const
   { if(Device==0) return (uint32_t)floor(1.0/SamplePeriod+0.5);
     try { return (uint32_t)floor(Device->getSampleRate(SOAPY_SDR_RX, Channel)+0.5); }
     catch(const std::exception &Ex) { return (uint32_t)floor(1.0/SamplePeriod+0.5); } }

   // -----------------------------------------------------------------------------------------------
   // Gain
   // -----------------------------------------------------------------------------------------------

   // ogn-rf gain modes: 0=Auto, 1=Manual, 2=Linearity, 3=Sensitivity (R820T specific)
   int setTunerGainMode(int Mode)
   { GainMode=Mode;
     if(Device==0) return -1;
     try { Device->setGainMode(SOAPY_SDR_RX, Channel, Mode<=0); }   // automatic == (Mode<=0)
     catch(const std::exception &Ex) { printf("LimeSDR::setGainMode(%d) => %s\n", Mode, Ex.what()); return -1; }
     return 1; }

   int setTunerGain(int Gain)                                   // [0.1 dB], as RTLSDR
   { double dB=0.1*Gain;
     if(Device==0) return -1;
     try
     { SoapySDR::Range Range=Device->getGainRange(SOAPY_SDR_RX, Channel);
       if(dB<Range.minimum()) dB=Range.minimum();
       if(dB>Range.maximum()) dB=Range.maximum();
       Device->setGain(SOAPY_SDR_RX, Channel, dB);
       GaindB=Device->getGain(SOAPY_SDR_RX, Channel); }
     catch(const std::exception &Ex) { printf("LimeSDR::setGain(%4.1f dB) => %s\n", dB, Ex.what()); return -1; }
     return (int)floor(10.0*GaindB+0.5); }

   int getTunerGain(void) const { return (int)floor(10.0*GaindB+0.5); }

   // Build a table of the gains this device can actually do, in the 0.1 dB units ogn-rf uses
   int BuildGainTable(void)
   { Gains=0;
     if(Device==0) return 0;
     try
     { SoapySDR::Range Range=Device->getGainRange(SOAPY_SDR_RX, Channel);
       double Min=Range.minimum(), Max=Range.maximum();
       double Step=(Max-Min)/63.0;                            // 64 entries is what RTLSDR uses
       if( (Range.step()>0) && (Range.step()>Step) ) Step=Range.step();
       for(int Idx=0; (Idx<64) && (Min+Idx*Step<=Max+1e-6); Idx++)
         Gain[Gains++]=(int)floor(10.0*(Min+Idx*Step)+0.5); }
     catch(const std::exception &Ex) { printf("LimeSDR::getGainRange() => %s\n", Ex.what()); }
     return Gains; }

   int getTunerGains(int *GainOut=0) const
   { if(GainOut) for(int Idx=0; Idx<Gains; Idx++) GainOut[Idx]=Gain[Idx];
     return Gains; }

   void PrintGains(void) const
   { printf("LimeSDR::Gain[%d] =", Gains);
     for(int Idx=0; Idx<Gains; Idx++) printf(" %+5.1f", 0.1*Gain[Idx]);
     printf(" [dB]\n"); }

#ifdef NEW_RTLSDR_LIB
   // The LMS7002M has no per-stage gain chain to enumerate, so these report nothing.
   int getTunerStageGains(int Stage, int32_t *Gain, char *Description=0) { return 0; }
   int getTunerBandwidths(int *Bandwidth=0) { return 0; }
   int PrintBandwidths(void) const { printf("LimeSDR::Bandwidth[] = (not applicable)\n"); return 0; }
#endif

   // -----------------------------------------------------------------------------------------------
   // Things ogn-rf configures that an LMS7002M does not have
   // -----------------------------------------------------------------------------------------------

   int setOffsetTuning(int ON=1) { return 0; }                  // no offset-tuning mode
   int getOffsetTuning(void) const { return 0; }

   int setBiasTee(int On=1) { BiasTee=On; return 0; }           // no on-board bias tee
   int getBiasTee(void) const { return BiasTee; }

   // Pick the LNA path (antenna port) that suits the frequency, when the driver offers a choice
   int SelectAntenna(double Frequency, bool Verbose=true)
   { if(Device==0) return -1;
     std::vector<std::string> List;
     try { List=Device->listAntennas(SOAPY_SDR_RX, Channel); }
     catch(const std::exception &Ex) { return -1; }
     if(List.empty()) return -1;

     const char *Prefer[3] = { "LNAL", "LNAW", "LNAH" };       // low band first
     if(Frequency>=1500e6) { Prefer[0]="LNAH"; Prefer[1]="LNAW"; Prefer[2]="LNAL"; }

     for(int Pref=0; Pref<3; Pref++)
       for(size_t Idx=0; Idx<List.size(); Idx++)
         if(List[Idx]==Prefer[Pref])
         { try { Device->setAntenna(SOAPY_SDR_RX, Channel, List[Idx]); }
           catch(const std::exception &Ex) { return -1; }
           strncpy(AntennaName[0], List[Idx].c_str(), 32); AntennaName[0][31]=0; Antenna=0;
           if(Verbose) printf("LimeSDR::SelectAntenna() => %s\n", AntennaName[0]);
           return 0; }
     return -1; }

   // -----------------------------------------------------------------------------------------------
   // Time
   // -----------------------------------------------------------------------------------------------

   double getTime(void) const                                     // system time at this very moment
#ifndef __MACH__
   { struct timespec Now; clock_gettime(CLOCK_REALTIME, &Now); return Now.tv_sec + 1e-9*Now.tv_nsec; }
#else
   { struct timeval Now; gettimeofday(&Now, 0); return Now.tv_sec + 1e-6*Now.tv_usec; }
#endif

   // -----------------------------------------------------------------------------------------------
   // Streaming
   // -----------------------------------------------------------------------------------------------

   // Drop whatever the driver has already buffered, so the next Read() starts with fresh samples
   int ResetBuffer(void)
   { if(Stream==0) return -1;
     void *Buffs[1]; Buffs[0]=Scratch;
     int Flags=0; long long TimeNs=0;
     int Drains=0;
     while(Device->readStream(Stream, Buffs, (size_t)(Mtu>0?Mtu:1024), Flags, TimeNs, 0)>0)
     { if(++Drains>10000) break; }
     return 0; }

   // Read the given number of samples as interleaved 8-bit unsigned I/Q (128 = zero), i.e. the
   // same layout librtlsdr produces. 12-bit LMS7002M samples arrive as CS16 with the 12 bits in
   // the upper part of the word, so >>Shift maps them onto the 8-bit range with the same
   // full-scale as an RTL-SDR dongle.
   int Read(uint8_t *Buffer, int Samples)
   { if( (Stream==0) || (Device==0) ) return -1;
     Samples&=0xFFFFFF00;                                        // multiple of 256, as in rtlsdr.h
     int Done=0; int Timeouts=0;
     while(Done<Samples)
     { int Want=Samples-Done;
       if( (Mtu>0) && (Want>Mtu) ) Want=Mtu;
       void *Buffs[1]; Buffs[0]=Scratch;
       int Flags=0; long long TimeNs=0;
       int Ret=Device->readStream(Stream, Buffs, (size_t)Want, Flags, TimeNs, ReadTimeoutUs);
       if(Ret>0)
       { Timeouts=0;
         const int16_t *Src=Scratch; uint8_t *Dst=Buffer+2*Done;
         for(int Idx=0; Idx<2*Ret; Idx++)
         { int Val=(Src[Idx]>>Shift)+128;
           if(Val<0) Val=0; else if(Val>255) Val=255;
           Dst[Idx]=(uint8_t)Val; }
         Done+=Ret; BytesRead+=2*Ret; }
       else if(Ret==SOAPY_SDR_OVERFLOW) continue;                // sample loss, keep going
       else if(Ret==SOAPY_SDR_TIMEOUT)
       { if(++Timeouts>MaxTimeouts)
         { printf("LimeSDR::Read() => %d timeouts, giving up after %d/%d samples\n", Timeouts, Done, Samples); return -1; }
         continue; }
       else
       { printf("LimeSDR::Read() => readStream() returned %d\n", Ret); return -1; } }
     return Done; }

   int Read(SampleBuffer<uint8_t> &Buffer, int Samples)
   { if(Buffer.Allocate(2,Samples)<=0) return 0;
     int ReadSamples=Read(Buffer.Data, Samples);
     double Time=getTime();
     if(ReadSamples>0)
     { Buffer.Full=ReadSamples*2;
       Buffer.Rate=getSampleRate();
       Buffer.Freq=getCenterFreq();
       Buffer.Time=Time-(double)ReadSamples/Buffer.Rate; }
     return ReadSamples; }

   int CancelAsync(void) { return 0; }                            // acquisition is synchronous here

} ;

// ==================================================================================================

#endif // __LIMESDR_H__
