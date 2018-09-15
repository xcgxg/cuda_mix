// Released on July 9, 2004
#include "../stdafx.h"

#include  <stdio.h>
#include  <stdlib.h>
#include  <conio.h>
#include  <math.h>
#include  <string.h>
#include  <io.h>
#include  <Dos.h>
#include  "gp2021.h"
#include  "gpsconst.h"
#include  "gpsstruct.h"
#include  "correlatorProcess.h"



extern CHANNEL		chan[12];
extern CORRELATOR	correlator[chmax+1]; 
extern long			TIC_CNTR;

extern double  deltaPhaseConst;   
extern double deltaCodePhaseConst;

inline void to_gps(int add,int data)
{
	// outpw(0x304,add);
	// outpw(0x308,data);
	// The above code is remmed by Ning LUO, when the real hardware is ready,
	// this function is used to write data to the CAC board

}

inline int from_gps(int add)
{
	// outpw(0x304,add);
	// return(inpw(0x308));
	// The above code is remmed by Ning LUO, when the real hardware is ready,
	// this function is used to write data to the CAC board
}


inline int accum_status(void)
{
	// return(from_gps(0x82));
	// The above code is remmed by Ning Luo, it is used to read "ACCUM_STATUS_A"
	// from GP2021. This register means if new accumulated data is ready to read
	// details can be referred in page 31 of GP2021 manual
	// in new CAC board, the address can be different
}

void all_accum_reset(void)
{
	// This is a NULL function originally
}

void data_tst(int data)
{
	// to_gps(0xf2,data);
	// the above code is remmed by Ning LUo, it is used to set to check the 16bit wide
	// data bus of GP2021, it should not be used in new CAC board
}


// 读取相关器ch通道的ms计数器(锁存的)
unsigned int  readEpoch(char ch)
{
	// return(from_gps((ch<<3)+4));
	// the above code is remmed by Ning Luo, It is used to return the epoch counter
	// the following is referred to the manual
	// Bits 15, 14, 7, 6 and 5: Not used. Read gives low.
	// Bits 13 to 8: CHx_20MS_EPOCH The 20ms epoch
	// counter value that was sampled at the last TIC event, with
	// a valid range from 0 to 49.
	// Bits4 to 0: CHx_1 MS_EPOCH The 1ms epoch counter value that was sampled 
	// at the last TIC event, with a valid range from 0 to 19.

#ifndef REAL_TIME
	return correlator[ch].epochCounter;
#endif
}


unsigned int readEpochCheck(char ch)
{
	// return(from_gps((ch<<3)+7));

	// the above code is remmed by Ning LUO.
	/*CHx_EPOCH_CHECK (Read Address)
	Bits 15 to 14: Not used.
	Bits 13 to 8: Instantaneous value of CHx_20MS_EPOCH.
	Bits 7 to 5: Not used.
	Bits 4 to 0: Instantaneous value of CHx_1MS_EPOCH.
	Reading this address gives the instantaneous value of the
	CHx_1MS_EPOCH and the CHx_20MS_EPOCH
	counters. It can be used to verify if the Epoch counters
	have been properly initialised by the software. Its value is
	not latched and is incremented on each DUMP. To ensure
	the correct result, this register should be read only when
	there is no possibility of getting a DUMP during the read
	cycle, by synchronising the read to NEW_ACCUM_DATA.
	The ranges of these values are the same as those seen in
	the CHx_EPOCH register.*/

#ifndef REAL_TIME
	return correlator[ch].epochCounterCheck;	
#endif
}

void writeEpoch(char ch, unsigned data)
{
#ifndef REAL_TIME
	correlator[ch].epochCounterCheck = data;
#endif
}

double readCodePhase(char ch)
{
	// return(from_gps((ch<<3)+5));		返回code_DCO_phase
	// return(from_gps((ch<<3)+0x1));	返回code_phase

	// the above code is remmed by Ning LUO. 
	/* CHx_CODE_DCO_PHASE (Read Address)
	Bits 15 to 10: Not used (low when read).
	Bits 9 to 0: CHx_CODE_DCO_PHASE: Contains the ten
	more significant bits (25 to 16) of the Code DCO phase
	accumulator sampled at a TIC event. It is an unsigned
	integer valid from 0 to 1023. The weight of the least
	significant bit is 2pi/1024 radians, 2pi being half of a code
	chip, so the pseudorange resolution is 1/2048 of a chip,
	(equivalent to 0.15 metre or 0.5ns).
	The CHx_CODE_DCO_PHASE content is not protected
	by any overwrite protection mechanism. */

	/* CHx_CODE_PHASE 
	Bits 15 to 11: Not used, Low when read.
	Bits 10 to 0: CHx_CODE_PHASE (Read) This is the state
	of the Code Phase Counter, (an 11-bit binary up counter
	clocked by the Code Generator Clock), stored on TIC. The
	phase is expressed as a number of half code chips and
	ranges from 0 to 2046 half chips. A reading of 2046 is very
	rare and can only occur if the TIC captures the Code phase
	just after the counter reaches 2046 and before it is reset
	by a DUMP from the C/A Code Generator. DUMP also
	increments the Epoch counter, so the meaning of a phase
	value of 2046 + the previous Epoch value is the same as
	a phase value of 0 + the incremented Epoch value, and
	either is valid. If a TIC occurs during a Code Slew the
	reading will be 0, and that channel’s Measurement Data is
	of no use. */

	// pls notice the resolution may be lower for high-performance receiver
	// In our design, the resolution must be better than 1cm

#ifndef REAL_TIME
	return (double)correlator[ch].tauLatch/(1.023e6*d_2p40);
#endif
	// 改写为实时函数时可能会分两次读入码相位的整数部分和小数部分，
	// 在拼装为一个double数

}

// 写入CA的速率控制

void writeCodeFreq(char ch, double data)
{
	// to_gps((ch<<3)+0x5,data);	写入码速率高位控制字
	// to_gps((ch<<3)+0x6,data);	写入码速率低位控制字

	/* _INCR_HIGH bits 15 to 9: Not used in this operation.
	_INCR_HIGH bits 8 to 0: More significant bits (24 to 16)
	of the Code DCO phase increment when used before a
	CODE_DCO_INCR_LOW.
	_INCR_LOW bits 15 to 0: Less significant bits (15 to 0) of
	the Code DCO phase increment.
	The contents of registers _INCR_HIGH and _INCR_LOW
	are combined to form the 25 bits of the
	CHx_CODE_DCO_INCR register, the Code DCO phase
	increment number. In order to write successfully, the top
	9 bits must be written first, to any of the _HIGH addresses.
	They will be stored in a buffer and only be transferred into
	the increment register of the DCO together with the _LOW 
	word. A 25-bit increment number is adequate for a 26-bit
	accumulator DCO as the increment to the MSB is always zero.
	The LSB of the INCR register represents a step given by:
	Min. step frequency, Real_Input mode = (40MHz/7)/2^26
	= 85.14949mHz
	Min. step frequency, Complex_Input mode = (35MHz/6)/2^26
	= 86·92344mHz
	Output frequency = CHx_CARRIER_DCO_INCR (min. step frequency).
	Note: The Code DCO drives the Code Generator to give
	half chip time steps and so must be programmed to twice
	the required chip rate. This means that the chip rate
	resolution is 42·57475mHz in Real_lnput mode or
	43·46172mHz in Complex_Input mode.
	The nominal frequency is 1.023000000MHz before
	allowing for Doppler shift or crystal error. Writing
	016EA4A8H into the CHx_CODE_DCO_INCR register will
	generate a chip rate of 1.022999968MHz in Real_lnput
	mode. In Complex_mode, 01672922H will generate a chip
	rate of 1.022999970 MHz.*/

#ifndef REAL_TIME
    __int64 fc = (__int64)(data*deltaCodePhaseConst + 0.5);
    if (fc < 0) {
        OutputDebugString(TEXT("Error!\n"));
        fc = (__int64)(data*deltaCodePhaseConst + 0.5);
    }
	correlator[ch].fCode = fc;

#endif
	// 当进行硬件实时模式时，该函数必须重写
	// 输入的double型数，可能会参考GP2021的模式分解为两个long型整数
}

// 设置CA码发生器的初始相位
// 范围：[0，0.001)，单位：秒
void writeCodePhase(char ch, double data)
{
#ifndef REAL_TIME
	correlator[ch].tau = (__int64)(data*tauConst+0.5);
#endif
	// 当进行硬件实时模式时，该函数必须重写
	// 输入的double型数，可能会参考GP2021的模式分解为两个long型整数
}
	

double readCarrierPhase(char ch)
{
	/*long result;
	result=from_gps((ch<<3)+6);
	result=result<<16;
	result=result+from_gps((ch<<3)+2);
	return(result);*/

	// The above code is remmed by Ning LUO
	/*CHx_CARRIER_CYCLE_HIGH, CHx_CARRIER_CYCLE_LOW(Read Address)
	_HIGH bits 15 to 4: not used - low when read.
	_HIGH bits 3 to 0: Carrier Cycle Count bits 19 to 16.
	_LOW bits 15 to 0: Carrier Cycle Count bits 15 to 0.
	The Correlator tracking channel hardware allows for
	measurement of integrated carrier phase through the
	CHx_CARRIER_CYCLE_HIGH and _LOW and the	CHx_CARRIER_DCO_PHASE registers, 
	which are part of the Measurement Data sampled at every TIC. 
	The	CHx_CARRIER_CYCLE_HIGH and _LOW registers
	contain the 20- bit number of positive going zero crossings
	of the Carrier DCO (4 bits are in _HIGH and 16 in _LOW).
	The cycle fraction can be read from the CHx_Carrier_DCO_Phase register.
	In the CHx_CARRIER_CYCLE counter, a TIC generates two consecutive actions. 
	First it latches the 4 more significant bits of the cycle counter into
	CHx_CARRIER_CYCLE_HIGH and the 16 less significant bits into CHx_CARRIER_CYCLE_LOW. 
	Then it resets the cycle counter.  ******
	After each TIC, every time the Carrier DCO accumulator generates an overflow 
	as a result of a carrier cycle being completed, the cycle counter increments by one.
	In Real_lnput mode the nominal CARRIER DCO frequency with no Doppler and 
	no oscillator drift compensation is 1.405396825 MHz, so in 100ms, 
	there will be about 140540 cycles.
	In almost all applications the number of Carrier DCO cycles does not vary much 
	from one TIC interval to another so it is possible to predict the MSBs of the value
	and then only read the CHx_CARRIER_CYCLE_LOW register.

	CHx_CARRIER_CYCLE_HIGH and_LOWcontents are not protected by an overwrite protection 
	mechanism and so must be read before the next TIC. */

#ifndef REAL_TIME
	return (correlator[ch].carrierCycleLatch
		+(double)correlator[ch].phaseLocalLatch*carrierNCORes);
#endif
	// 当进行硬件实时模式时，该函数必须重写
	// 输入的double型数，可能会参考GP2021的模式分解为两个long型整数
}

// 写入载波频率控制字，其中data的标称值为载波中频，单位：Hz
void writeCarrierFreq(char ch, double data)//writeCarrierFreq(ch,chan[ch].carrier_freq);
{
	// to_gps((ch<<3)+0x3,data);
	
	// this function is remmed by Ning
	/* _INCR_HIGH bits 15 to 10: Not used in this operation.
	_INCR_HIGH bits 9 to 0: More significant bits (25 to 16)
	of the Carrier DCO phase increment when used before a
	write to _CARRIER_DCO_INCR_LOW.
	_INCR_LOW bits 15 to 0: Less significant bits (15 to 0) of
	the Carrier_DCO (phase increment.
	The contents of registers _INCR_HIGH and_lNCR_LOW
	are combined to form the 26 bits of the
	CHx_C,ARRIER_DCO_INCR register, the carrier DCO
	phase increment number. In order to write successfully,
	the top 10 bits must be written first, to any of the _HIGH
	addresses. They will be stored in a buffer and only be
	transferred into the increment register of the DCO together
	with the _LOW word. A 26-bit increment number is
	adequate for a 27-bit accumulator DCO, as the increment
	to the MSB is always zero.
	The LSB of the INCR register represents a step given by:
	Min. step frequency, Real_Input mode = (40MHz/7)/2^27
	= 42·57475mHz
	Min. step frequency, Complex_Input mode = (35MHz/6)/2^27
	= 43·46172mHz
	Output frequency = CHx_CARRIER_DCO_INCR (min. step
	frequency).
	With a GP2015/GP2010 style front end, the nominal value
	of the IF is 1405396.826Hz before allowing for Doppler
	shift or crystal error. Writing 01F7B1B9H into the
	CHx_CARRIER_DCO_INCR register will generate a local
	oscillator frequency of 1405396.845 MHz. */

#ifndef REAL_TIME
    long long carr = (long long)(data*deltaPhaseConst + 0.5);
	correlator[ch].fCarr = (long)(carr & 0xffffffffUL);
#endif

}

// 设置相关器通道的卫星标号
void ch_cntl(char ch,int data)
{
	/*to_gps(ch<<3,data); */

	// remmed by Ning LUO
	// pls refer to page 36 - 37
	// This function needs modifications
#ifndef REAL_TIME
	correlator[ch].sv = data;
#endif
}


void ReadAccumData(char ch)		// new function written by Ning LUO
{
#ifndef REAL_TIME
	chan[ch].i_prompt = correlator[ch].latchedSPR;
	chan[ch].q_prompt = correlator[ch].latchedSPI;
	chan[ch].i_early = correlator[ch].latchedSER;
	chan[ch].q_early = correlator[ch].latchedSEI;
	chan[ch].i_late = correlator[ch].latchedSLR;
	chan[ch].q_late = correlator[ch].latchedSLI;
#endif
}

void ch_accum_reset(char ch)
{
	/* to_gps((ch<<2)+0x85,0); */
	// remmed by Ning LUO
	// refer to page 30 -31. This means clear the flag of all channel output
}

void ch_code_slew(char ch,int data)
{
	/* to_gps((ch<<2)+0x84,data); */
	// remmed by Ning LUO
	/* CHx_CODE_SLEW (Read Address)
	CHx_CODE_SLEW_COUNTER,
	MULTI_CODE_SLEW_COUNTER,
	ALL_CODE_SLEW_COUNTER
	(Write Address)
	Bits 15 to 11: Not used.
	Bits 10 to 0: An unsigned integer ranging from 0 to 2047
	representing the number of code half chips to be slewed
	immediately after the next DUMP if in UPDATE mode
	orafter the next TIC, if in PRESET mode. Since there are
	only 2046 half chips in a GPS C/A code, a programmed
	value of 2047 is equivalent to a programmed value of 1,
	but the next DUMP event will take place 1ms later. In
	PRESET mode, the slew timing is set only by TIC, which
	will also reset the code generator, (no DUMP is needed).
	A non-zero slew must always be programmed when using
	PRESET mode.
	The CHx_CODE_SLEW register can be written to at any
	time. If two accesses have taken place before a DUMP in
	U PDATE mode or before a TIC when in PRESET mode,
	the latest value will be used at the next slew operation.
	During the time a slew process is being executed, any
	further write access to the CHx_CODE_SLEW register will
	be stored until the following DUMP and then cause the
	transfer of this new value into the counter. This situation
	may be avoided by synchronising the access with the
	associated CHx_NEW_ACCUM_DATA status bit.
	If a channel is inactive, a non-zero slew value should be
	written into CHx_CODE_SLEW before the channel is*/
	// To be redesigned

#ifndef REAL_TIME
//	correlator[ch].codeSlewCounter = data;
#endif

}

void all_code_slew(int data)
{
	/*to_gps(0x70,data);*/
	// remmed by Ning LUO
	// refer to above
}

//void data_retent_w(int data)
//{
	/* to_gps(0xe4,data); */
	// remmed by Ning LUO
	/* DATA_RETENT
	(Write/Read Address)
	This is a byte wide Read/Write register which can be used
	to store a predetermined value, which can be interrogated
	in order to determine whether a total power loss (below
	the data retention level ) has occurred. */
	// may not be used
//}

//int data_retent_r(void)
//{
	/* return(from_gps(0xe4)); */
	// remmed by Ning LUO
	// may not be used
//}

//void data_bus_test_w(int data)
//{
	/* to_gps(0xf2,data); */
	// remmed by Ning LUO
	// may not be used
//}

//int data_bus_test_r(void)
//{
	/* return(from_gps(0xf2)); */
	// remmed by Ning LUO
	// may not be used
//}


//inline int meas_status(void)
//{
	/* return(from_gps(0x81));*/
	// remmed by Ning LUO
	/* CHx_MISSED_MEAS_DATA status bit, when high,
	indicates that one or more sets of measurement data have
	been missed since the last read from this register. It is set
	high by a read from the Code Phase Counter of the same
	channel, when the previous value in the Code Phase
	Counter has not been read, and is reset by a read from
	the MEAS_STATUS_A register or by disabling the channel.
	If this register is always read after the Code Phase Counter,
	it indicates whether measurement data has been missed
	before the last read of the Code Phase Counter. All
	CHx_MISSED_MEAS_DATA bits are set low by a
	hardware (NRESET) or software (MRB reset.
	The MEAS_INT bit is set high at each TIC and 50ms before
	each TIC (if TIC period is greater then 50ms), and is cleared
	by reading this register. This bit is used as a flag to the
	microprocessor to time software module swapping and is
	reset by a hardware master reset (NRESET low) but not
	by an MRB software reset.
	The TIC bit is set high at every TIC and is cleared by
	reading this register. The purpose of the bit is to tell the
	microprocessor that new Measurement Data is available.
	This bit is reset by a hardware master reset (NRESET at
	low) but not by an MRB in RESET_CONTROL */

	// not sure if this function is useful
//}

void programTIC(long data)
{
	/*unsigned int high,low;
	high=int(data>>16);
	low =int(data & 0xffff);
	to_gps(0x6d,high);
	to_gps(0x6f,low);*/

	// remmed by Ning LUO
	/* TIC period (Real Input mode)
			= ((PROG_TIC_HIGH*65536)+
			PROG_TIC_LOW+1)*7/40MHZ) */
#ifndef REAL_TIME
	TIC_CNTR = data;
#endif

}

//void reset_cntl(int data)
//{
	/*to_gps(0x7f,data);
	//  fprintf(out,"reset data=%x\n",data); */
//}



//void ch_on(char ch)
//{
/*ch_status=ch_status | bit_pat[ch];
reset_cntl(ch_status);*/

// The above code is remmed by Ning LUO
// To be redesigned
//}

//void ch_off(char ch)
//{
/*ch_status=ch_status & !bit_pat[ch];
reset_cntl(ch_status); */

// remmed by Ning LUO
// To be redesigned
//}

//void system_setup(int data)
//{
/* to_gps(0x7e,data); */
// remmed by Ning LUO
// to be redesigned
//}

//void test_control(int data)
//{
/* to_gps(0x7c,data); */
// remmed by Ning LUO
//}

//void status_latch(void)
//{
/* to_gps(0x80,0); */
// remmed by Ning LUO
//}

//void io_config(int data)
//{
/* to_gps(0xf0,data); */
// remmed by Ning LUO
//}

//void self_test(void)
//{
/*unsigned int indataaax,indata55x,indataaay,indata55y,error;
error=0;
data_retent_w(0x5500);
data_bus_test_w(0xaa55);
indata55x=data_retent_r();
indataaax=data_bus_test_r();
data_retent_w(0xaa00);
data_bus_test_w(0x55aa);
indataaay=data_retent_r();
indata55y=data_bus_test_r();
if ((indata55x != 0x5500) || (indataaax != 0xaa55)
|| (indataaay != 0xaa00) || (indata55y != 0x55aa))
{
error=1;
printf("data line error\n");
printf("indata55x=%x , indataaax=%x\n",indata55x,indataaax);
printf("indataaay=%x , indata55y=%x\n",indataaay,indata55y);
}
if (error==1) exit(0);*/

// remmed by Ning LUO
// this is to test GP2021, redesign may not be necessary
//}

