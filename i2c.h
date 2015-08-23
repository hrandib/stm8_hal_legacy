/*************************************************
*
*		Module based on ST AN3281
*		STM8 I2C optimized examples
*
* ***********************************************/

#pragma once
#ifndef I2C_H
#define I2C_H

#include "stm8s.h"
#include "gpio.h"
#include "clock.h"
#include "timers.h"
#include "delay.h"

//#define FAST_I2C_MODE

namespace Mcudrv
{
namespace Twis
{
	enum
	{
		BaseAddrLM75 = 0x48,
		BaseAddr24C = 0x50,
		BaseAddrBH1750 = 0x23,
		BaseAddrBMP180 = 0x77
	};
	enum Mode
	{
		Standard,
		Fast
	};
	enum AddrType
	{
		Addr7bit,
		Addr10bit
	};
	enum StopMode
	{
		Stop,
		NoStop
	};
	enum AckState
	{
		NoAck, Ack
	};

	template<Mode mode = Standard, typename Scl = Pe1, typename Sda = Pe2>
	class SoftTwi
	{
	private:
		static void Delay()
		{
			if(mode == Standard)
				delay_us<3>();
			else
			{
				__no_operation();
				__no_operation();
			}
		}
		#pragma inline=forced
		static bool Release()
		{
			for(uint8_t scl = 0; scl < 10; ++scl)
			{
				Scl::Clear();
				Delay();
				Scl::Set();
				Delay();
				if(Sda::IsSet())
				{
					Stop();
					return true;	//Sda released
				}
			}
			return false;	// Line is still busy
		}
	protected:
		#pragma inline=forced
		static void Start()
		{
			Sda::Clear();
			Delay();
			Scl::Clear();
			Delay();
		}
		#pragma inline=forced
		static void Stop()
		{
			Scl::Clear();
			Delay();
			Sda::Clear();
			Delay();
			Scl::Set();
			Delay();
			Sda::Set();
		}
		static AckState WriteByte(uint8_t data)
		{
			AckState ack = Ack;
			for(uint8_t i = 0; i < 8; ++i)
			{
				if((data & 0x80) == 0)
					Sda::Clear();
				else
					Sda::Set();
				Delay();
				Scl::Set();
				Delay();
				Scl::Clear();
				data <<= 1U;
			}
			Sda::Set();
			Delay();
			Scl::Set();
			Delay();
			if(Sda::IsSet())
				ack = NoAck;
			else
				ack = Ack;
			Scl::Clear();
			return ack;
		}
		static uint8_t ReadByte(AckState ackstate = Ack)
		{
			uint8_t data = 0;
			Sda::Set();
			for(uint8_t i = 0; i < 8; ++i)
			{
				data = (data << 1U);
				Scl::Set();
				Delay();
				if(Sda::IsSet()) data |= 0x01;
				Scl::Clear();
				Delay();
			}
			if(ackstate == Ack)
				Sda::Clear();
			else
				Sda::Set();
			Delay();
			Scl::Set();
			Delay();
			Scl::Clear();
			Delay();
			Sda::Set();
			return data;
		}

	public:
		static bool Init()
		{
			if((uint8_t)Scl::port_id == (uint8_t)Sda::port_id)
			{
				Scl::Port::Set((uint8_t)Scl::mask | (uint8_t)Sda::mask);
				Scl::Port::template SetConfig<(uint8_t)Scl::mask | (uint8_t)Sda::mask, GpioBase::Out_OpenDrain_fast>();
			}
			else
			{
				Scl::Set();
				Scl::template SetConfig<GpioBase::Out_OpenDrain_fast>();
				Sda::Set();
				Sda::template SetConfig<GpioBase::Out_OpenDrain_fast>();
			}
			if(!Sda::IsSet())
				return Release();			//Reset slave devices
			return true;	//Bus Ready
		}
		#pragma inline=forced
		static void Restart()
		{
			Sda::Set();
			Delay();
			Scl::Set();
			Delay();
		}
		static AckState Write(uint8_t addr, uint8_t* buf, uint8_t length, bool noStop = false)
		{
			Start();
			AckState state = WriteByte(addr << 1);
			if(state == NoAck) goto End;
			while(length--)
			{
				if(WriteByte(*buf++) == NoAck)
					break;
			}
			if(!length) state = Ack;
		End:
			if(!noStop) Stop();
			return state;
		}
		static AckState Write(const uint8_t *buf, uint8_t length, bool noStop = false) //length of data (except address)
		{
			return Write(*buf, buf + 1, length, noStop);
		}
		static AckState Write(uint8_t addr, uint8_t data, bool noStop = false)
		{
			Start();
			AckState state = NoAck;
			if(WriteByte(addr << 1U) == Ack && WriteByte(data) == Ack)
				state = Ack;
			if(!noStop) Stop();
			return state;
		}
		static bool Read(uint8_t addr, uint8_t* buf, uint8_t length)
		{
			Start();
			bool result = false;
			if(WriteByte((addr << 1U) | 0x01))
			{
				while(--length)
				{
					*buf++ = ReadByte();
				}
				*buf = ReadByte(NoAck);
				result = true;
			}
			Stop();
			return result;
		}
	};

	template<typename Twi = SoftTwi<> >
	class Lm75
	{
	public:
		enum { BaseAddr = 0x48 };
		static int16_t Read(uint8_t devAddr = 0)
		{
			int16_t result;
			Twi::Read(BaseAddr | devAddr, (uint8_t*)&result, 2);
			return result / 128;
		}
	};

	template<typename Twi = SoftTwi<> >
	class Eeprom24c
	{
	public:
		enum { BaseAddr = 0x50 };

	};

	template<typename Twi = SoftTwi<> >
	class Bh1750
	{
	public:
		enum { DevAddr = 0x23 };
		enum
		{
			BhPowerDown,
			BhPowerOn,
			BhReset = 0x07
		};
		enum Mode
		{
			ContHres = 0x10,// 1lx res. typ. 120ms (max 180ms)
			ContHres2,		// 0.5lx res. typ. 120ms
			ContLres		// 4lx res. typ. 16ms (max 24ms)
		};
		static AckState Init(Mode mode = ContHres)
		{
			return AckState(Twi::Write(DevAddr, BhPowerOn) && Twi::Write(DevAddr, mode));
		}
		static void SetMode(Mode mode)
		{
			return Twi::Write(DevAddr, mode);
		}

		static uint16_t Read()
		{
			uint8_t buf[2];
			if(!Twi::Read(DevAddr, buf, 2)) return 0;
			return uint16_t(buf[0] << 8U) | buf[1];
		}
	};

	template<typename Twi, uint8_t OversamplingFactor = 0>
	class Bmp180
	{
		static_assert(OversamplingFactor < 4, "Oversampling Factor must be in range 0..3");
	private:
		enum
		{
			BaseAddr = 0x77,
			Oss = OversamplingFactor,
			PMeasureDelay = Oss == 1 ? 8 :
							Oss == 2 ? 14 :
							Oss == 3 ? 26 : 5
		};
		enum RegMap
		{
			RegAC1 = 0xAA,
			RegTestID = 0xD0,
			RegControl = 0xF4,
			RegData = 0xF6,
			RegXlsb = 0xF8
		};
		enum CalValues
		{
			AC1, AC2, AC3, AC4, AC5, AC6,
			B1, B2,
			MB, MC, MD
		};
		enum ControlValue
		{
			CmdTemperature = 0x2E,
			CmdPressure = 0x34 | (Oss << 6),
		};

		static uint16_t calArr[11];

		static AckState SetCommand(ControlValue ctrl)
		{
			uint8_t data[2] = { RegControl, ctrl };
			return Twi::Write(BaseAddr, data, 2);
		}
		static uint16_t GetReg(RegMap reg)
		{
			using namespace Twis;
			uint16_t result;
			Twi::Write(BaseAddr, (uint8_t)reg, NoStop);
			Twi::Restart();
			Twi::Read(BaseAddr, (uint8_t*)&result, 2);
			return result;
		}
		static void GetCalValues()
		{
			for(uint8_t x = 0; x < 11; ++x)
			{
				calArr[x] = GetReg(RegMap(RegAC1 + x * 2));
			}
		}

	public:
		static void Init()
		{
			GetCalValues();
		}
	/*	static void PrintCalArray()
		{
			const uint8_t* const names[] = {
				"AC1", "AC2", "AC3", "AC4", "AC5", "AC6",
				"B1", "B2",
				"MB", "MC", "MD"
			};

			for(uint8_t i = 0; i < 11; ++i)
			{
				Uart::Puts(names[i]);
				Uart::Puts(": ");
				if(i < 3 || i > 5) Uart::Puts(int16_t(calArr[i]));
				else Uart::Puts(calArr[i]);
				Uart::Newline();
			}
		}
	*/
		static uint32_t GetPressure()
		{
			if(!SetCommand(CmdTemperature)) return 0;
			delay_ms(5);
			uint16_t rawvalueT = GetReg(RegData);
			int32_t x1 = ((int32_t)rawvalueT - calArr[AC6]) * calArr[AC5] / (1U << 15);
			int32_t x2 = (int32_t)((int16_t)calArr[MC]) * (1U << 11) / (x1 + calArr[MD]);
			int32_t b5 = x1 + x2;

			if(!SetCommand(CmdPressure)) return 0;
			delay_ms(PMeasureDelay);
			int32_t rawvalueP = GetReg(RegData);
			if(Oss) rawvalueP = (rawvalueP << 8 | (GetReg(RegXlsb)) & 0xFF) >> (8 - Oss);
			int32_t b6 = b5 - 4000;
			x1 = ((int32_t)((int16_t)calArr[B2]) * (b6 * b6 / (1U << 12))) / (1U << 11);
			x2 = (int32_t)((int16_t)calArr[AC2]) * b6 / (1U << 11);
			int32_t x3 = x1 + x2;
			int32_t b3 = ((((int32_t)((int16_t)calArr[AC1]) * 4 + x3) << Oss) + 2) / 4;
			x1 = (int32_t)((int16_t)calArr[AC3]) * b6 / (1U << 13);
			x2 = ((int32_t)((int16_t)calArr[B1]) * ((b6 * b6) / (1U << 12))) / (1UL << 16);
			x3 = ((x1 + x2) + 2) / 4;
			uint32_t b4 = (int32_t)calArr[AC4] * (x3 + 32768U) >> 15U;
			uint32_t b7 = (rawvalueP - b3) * (50000U >> Oss);
			uint32_t p;
			if(b7 < 0x80000000UL)
			{
				p = (b7 * 2) / b4;
			}
			else
			{
				p = (b7 / b4) * 2;
			}
			x1 = (p >> 8) * (p >> 8);
			x1 = (x1 * 3038) >> 16;
			x2 = (-7357 * (int32_t)p) >> 16;
			p = p + ((x1 + x2 + 3791) >> 4);
			return p;
		}
		static int16_t GetTemperature()
		{
			SetCommand(CmdTemperature);
			delay_ms(5);
			uint16_t rawvalue = GetReg(RegData);
			int32_t x1 = ((int32_t)rawvalue - calArr[AC6]) * calArr[AC5] / (1U << 15);
			int32_t x2 = (int32_t)((int16_t)calArr[MC]) * (1U << 11) / (x1 + calArr[MD]);
			return (x1 + x2 + 8) / (1U << 4);
		}
	};
	template<typename Twi, uint8_t Oss>
	uint16_t Bmp180<Twi, Oss>::calArr[11];


	//Based on hardware, is still buggy
/*	template<Mode mode = Standard, AddrType addrType = Addr7bit>
	class Twi
	{
	private:
		enum { I2C_TOUT = 40 };
		enum States
		{
			INI_00 = 0,		//Initial
			SB_01 = 01,		//Write - 0x
			ADD10_02 = 02,
			ADDR_03 = 03,
			BTF_04 = 04,
			SB_11 = 11,		//Read - 1x
			ADD10_12 = 12,
			ADDR_13 = 13,
			BTF_14 = 14,
			BTF_15 = 15,
			RXNE_16 = 16,
			BTF_17 = 17,
			RXNE_18 = 18
		};
		enum Direction
		{
			DirWrite,
			DirRead
		};
		typedef typename stdx::conditional<addrType == Addr7bit, uint8_t, uint16_t>::type SlaveAddr_t;
		static volatile States state_;
		static volatile uint8_t err_state_;  	// error state
		static volatile uint8_t err_save_;   	// I2C->SR2 copy in case of error
		static volatile uint8_t TIM4_tout_;  // Timeout Value

		static Direction direction_;
		static uint8_t numByte_;
		static uint8_t* dataBuffer_;
		static SlaveAddr_t slaveAddr_;
		static StopMode noStop_;
		#pragma inline=forced
		static void set_tout_ms(uint16_t a)
		{
			TIM4_tout_ = a;
		}
	public:
		static void Init()
		{
			//define SDA, SCL outputs, HiZ, Open drain, Fast
			GpioE::Set(P1 | P2);
			GpioE::SetConfig<P1 | P2, GpioBase::Out_OpenDrain_fast>();
			enum
			{
				freqr = (uint8_t)(F_CPU / 1e6),		//Clock frequency in MHz
				tcpu_ns = 1000 / freqr				//CPU cycle in ns
			};
			I2C->FREQR = freqr;               // input clock to I2C = F_CPU
			if(mode == Fast)
			{
				I2C->CCRL = 900 / tcpu_ns + 1;		// 900/62.5= 15, (SCLhi must be at least 600+300=900ns!)
				I2C->CCRH = 0x80;					// fast mode, duty 2/1 (bus speed 62.5*3*15~356kHz)
				I2C->TRISER = 300 / tcpu_ns + 1;	// 300/62.5 + 1 = 5  (maximum 300ns)
			}
			else //if(mode == Standard)
			{
				I2C->CCRL = 5000 / tcpu_ns + 1;		// CCR = (SCLhi must be at least 4000+1000=5000ns!)
				I2C->CCRH = 0;						// standard mode, duty 1/1 bus speed 100kHz
				I2C->TRISER = 1000 / tcpu_ns + 1;	// 1000ns/(125ns) + 1  (maximum 1000ns)
			}
//			I2C->OARL = 0xA0;              // own address A0;
			I2C->OARH |= 0x40;
			if(I2C->SR3 & I2C_SR3_BUSY)
				ReleaseBus();			//Reset slave devices
			I2C->ITR = I2C_ITR_ITERREN | I2C_ITR_ITEVTEN;	// enable Event & error interrupts
			I2C->CR2 = I2C_CR2_ACK;		// ACK=1, Ack enable
			I2C->CR1 = I2C_CR1_PE;			// PE=1
//			I2C->CR1 &= ~I2C_CR1_NOSTRETCH;	// Stretch enable

			// Initialise I2C State Machine
//			err_save_ = 0;
//			state_ == INI_00;
//			set_tout_ms(0);

			using namespace T4;
			Timer4::WriteAutoReload(freqr * 8);
			Timer4::EnableInterrupt();
			Timer4::Init(Div_128, CEN);		//1ms interrupts
		}

		static void ReleaseBus()
		{
			typedef Pe1 Scl;
			typedef Pe2 Sda;
			for(uint8_t scl = 0; scl < 10; ++scl)
			{
				Scl::Clear();
				delay_ms(1);
				Scl::Set();
				delay_ms(1);
				if(Sda::IsSet()) break;
			}
			//Stop condition
			Scl::Clear();
			delay_ms(1);
			Sda::Clear();
			delay_ms(1);
			Scl::Set();
			delay_ms(1);
			Sda::Set();
		}

		#pragma inline=forced
		static bool IsBusy()
		{
			return I2C->SR3 & I2C_SR3_BUSY;
		}
		#pragma inline=forced
		static bool IsInitialState()
		{
			return state_ == INI_00;
		}
		static bool IsEnabled()
		{
			return I2C->CR1 & I2C_CR1_PE;
		}

		#pragma inline=forced
		static void ErrProc()
		{
			err_save_ = I2C->SR2;
			err_state_ = state_;
//			I2C->CR2 = I2C_CR2_SWRST;
//			I2C->CR1 = 0; //~I2C_CR1_PE;
			state_ = INI_00;
//			I2C->CR2 = 0;
//			Init();
			set_tout_ms(0);
			Led1::Set();
		}

		static bool Write(SlaveAddr_t slaveAddr, StopMode noStop, const uint8_t numByteToWrite, const uint8_t* dataBuffer)
		{
			// check if communication on going && if STATE MACHINE is in state INI_00
			if(IsBusy() || !IsInitialState())
				return false;
			// set ACK
			I2C->CR2 |= I2C_CR2_ACK;
			// reset POS
			I2C->CR2 &= ~I2C_CR2_POS;
			// setup I2C comm. in write
			direction_ = DirWrite;
			// copy parametters for interrupt routines
			noStop_ = noStop;
			slaveAddr_ = slaveAddr;
			numByte_ = numByteToWrite;
			dataBuffer_  = (uint8_t*)dataBuffer;
			// set comunication Timeout
			set_tout_ms(I2C_TOUT);
			// generate Start
			state_ = SB_01;
			I2C->CR2 |= I2C_CR2_START;
			return true;
		}

		static bool Read(SlaveAddr_t slaveAddr, StopMode noStop, uint8_t numByteToRead, uint8_t* dataBuffer)
		{
			// check if communication on going && if STATE MACHINE is in state INI_00
			if((IsBusy() && noStop == Stop) || !IsInitialState())
				return false;
			// set ACK
			I2C->CR2 |= I2C_CR2_ACK;
			// reset POS
			I2C->CR2 &= ~I2C_CR2_POS;
			// setup I2C comm. in Read
			direction_ = DirRead;
			// copy parameters for interrupt routines
			noStop_ = noStop;
			slaveAddr_ = slaveAddr;
			numByte_ = numByteToRead;
			dataBuffer_  = dataBuffer;
			// set comunication Timeout
			set_tout_ms(I2C_TOUT);
			//generate Start
			I2C->CR2 |= I2C_CR2_START;
			state_ = SB_11;
			I2C->ITR |= I2C_ITR_ITERREN | I2C_ITR_ITEVTEN;  // re-enable interrupt
			return true;
		}

		_Pragma(VECTOR_ID(I2C_SB_vector))
		__interrupt static void I2CIRQ()
		{
			uint8_t sr1, sr2;
			volatile uint8_t dummy;
		//	Get Value of Status registers and Control register 2
			sr1 = I2C->SR1;
			sr2 = I2C->SR2;
			dummy = I2C->CR2;
		//	Check for error in communication
			if (sr2 != 0)
			{
				ErrProc();
				Led2::Set();
			}
		//	Start bit detected
			if(sr1 & I2C_SR1_SB)
			{
				switch(state_)
				{
				case SB_01:
					if (addrType == Addr10bit)
					{
						I2C->DR = (uint8_t)(((slaveAddr_ >> 7) & 6) | 0xF0);  // send header of 10-bit device address (R/W = 0)
						state_ = ADD10_02;
						break;
					} else {
						I2C->DR = (uint8_t)(slaveAddr_ << 1);   // send 7-bit device address & Write (R/W = 0)
						state_ = ADDR_03;
						break;
					}
				case SB_11:
					if (addrType == Addr10bit)
					{
						I2C->DR = (uint8_t)(((slaveAddr_ >> 7) & 6) | 0xF1);// send header of 10-bit device address (R/W = 1)
					} else {
						I2C->DR = (uint8_t)(slaveAddr_ << 1) | 1 ; // send 7-bit device address & Write (R/W = 1)
					}
					state_ = ADDR_13;
					break;
				default: ErrProc();
					break;
				}
			}
		//	 ADD10
			if (I2C->SR1 & I2C_SR1_ADD10) {
				switch(state_)
				{
				case ADD10_02:
					I2C->DR = (uint8_t)(slaveAddr_);                // send lower 8-bit device address & Write
					state_ = ADDR_03;
					break;
				default: ErrProc();
					break;
				}
			}
		//	ADDR
			if (sr1 & I2C_SR1_ADDR)
			{
				switch (state_)
				{
				case ADDR_13 :

					if (numByte_ == 3)
					{
						dummy = I2C->SR3;
						state_ = BTF_15;
						break;
					}
					if (numByte_ == 2)
					{
					// set POS bit
						I2C->CR2 |= I2C_CR2_POS;
					//	Clear Add Ack Flag
						dummy = I2C->SR3;
						// set No ACK
						I2C->CR2 &= ~I2C_CR2_ACK;
						state_ = BTF_17;
						break;
					}
					if (numByte_ == 1)
					{
						I2C->CR2 &= ~I2C_CR2_ACK;
				//	Clear Add Ack Flag
						dummy = I2C->SR3;
						I2C->CR2 |= I2C_CR2_STOP;
						I2C->ITR |= I2C_ITR_ITBUFEN;
						state_ = RXNE_18;
						break;
					}
					if (numByte_ > 3)
					{
						dummy = I2C->SR3;
						state_ = BTF_14;
						break;
					}
					ErrProc();
					break;
				case ADDR_03 :
				// Clear Add Ack Flag
					dummy = I2C->SR3;
					I2C->DR = *dataBuffer_++;
					numByte_--;
					state_ = BTF_04;
					break;
				default : ErrProc();
					break;
				}
			}
			if (sr1 & I2C_SR1_RXNE)
			{
				switch (state_)
				{
				case RXNE_18 :
					*dataBuffer_++ = I2C->DR;			// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				case RXNE_16 :
					*dataBuffer_++ = I2C->DR;	// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				}
				I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
			}
		// BTF
			if (sr1 & I2C_SR1_BTF)
			{
				switch (state_)
				{
				case BTF_17 :
					I2C->CR2 |= I2C_CR2_STOP;                   				// generate stop request here (STOP=1)
					*dataBuffer_++ = I2C->DR;											// Read next data byte
					*dataBuffer_++ = I2C->DR;											// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				case BTF_14 :
					*dataBuffer_++ = I2C->DR;
					numByte_--;
					if(numByte_ <= 3)
						state_ = BTF_15;
					break;
				case BTF_15 :
					I2C->CR2 &= ~I2C_CR2_ACK;                     		// Set NACK (ACK=0)
					*dataBuffer_++ = I2C->DR;                    // Read next data byte
					I2C->CR2 |= I2C_CR2_STOP;                        // Generate stop here (STOP=1)
					*dataBuffer_++ = I2C->DR;                    // Read next data byte
					I2C->ITR |= I2C_ITR_ITBUFEN; 										// Enable Buffer interrupts (errata)
					state_ = RXNE_16;
					break;
				case BTF_04 :
					if (numByte_ && (I2C->SR1 & I2C_SR1_TXE))
					{
						I2C->DR = *dataBuffer_++;												// Write next data byte
						numByte_--;
						break;
					}
					else
					{
						if (noStop_ == Stop)
						{
							I2C->CR2 |= I2C_CR2_STOP;                   			// Generate stop here (STOP=1)
						}
						else
						{
							I2C->ITR = 0;                  // disable interrupt
						}
						state_ = INI_00;
						set_tout_ms(0);
						break;
					}
				}
			}
		}
		_Pragma(VECTOR_ID(TIM4_OVR_UIF_vector))
		__interrupt static void TimerIRQ()
		{
			T4::Timer4::ClearIntFlag();		//BUG: Check if busy flag set too long, reinit module in this case
			if(TIM4_tout_)
				if(--TIM4_tout_ == 0)
				{
					Led3::Set();
					I2C->CR1 = 0;
					I2C->CR2 = I2C_CR2_SWRST;
					__no_operation();
					if(!Pe1::IsSet() || !Pe2::IsSet())
						ReleaseBus();
					I2C->CR2 = 0;
					I2C->CR1 = I2C_CR1_PE;
					ErrProc();
				}
		}
	};

	template<Mode mode, AddrType addrType>
	volatile typename Twi<mode, addrType>::States Twi<mode, addrType>::state_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t Twi<mode, addrType>::err_state_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t Twi<mode, addrType>::err_save_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t Twi<mode, addrType>::TIM4_tout_;

	template<Mode mode, AddrType addrType>
	typename Twi<mode, addrType>::Direction Twi<mode, addrType>::direction_;

	template<Mode mode, AddrType addrType>
	uint8_t Twi<mode, addrType>::numByte_;

	template<Mode mode, AddrType addrType>
	uint8_t* Twi<mode, addrType>::dataBuffer_;

	template<Mode mode, AddrType addrType>
	typename Twi<mode, addrType>::SlaveAddr_t Twi<mode, addrType>::slaveAddr_;

	template<Mode mode, AddrType addrType>
	StopMode Twi<mode, addrType>::noStop_;
	*/


}//i2c
}//Mcudrv

#endif // I2C_H

