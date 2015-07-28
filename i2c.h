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

//#define FAST_I2C_MODE

namespace Mcudrv
{

namespace n_i2c
{
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

	template<Mode mode = Standard, AddrType addrType = Addr7bit>
	class i2c
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
			I2C->OARL = 0xA0;              // own address A0;
			I2C->OARH |= 0x40;
			I2C->ITR = I2C_ITR_ITERREN | I2C_ITR_ITEVTEN;	// enable Event & error interrupts
			I2C->CR2 |= I2C_CR2_ACK;		// ACK=1, Ack enable
			I2C->CR1 |= I2C_CR1_PE;			// PE=1
			I2C->CR1 &= ~I2C_CR1_NOSTRETCH;	// Stretch enable

			// Initialise I2C State Machine
//			err_save_ = 0;
//			state_ == INI_00;
//			set_tout_ms(0);

			using namespace T4;
			Timer4::WriteAutoReload(freqr * 8);
			Timer4::EnableInterrupt();
			Timer4::Init(Div_128, CEN);		//1ms interrupts
		}

		static void ErrProc()
		{
			err_save_ = I2C->SR2;
			err_state_ = state_;
			I2C->SR2 = 0;
			state_ = INI_00;
			set_tout_ms(0);
		}

		static bool Write(SlaveAddr_t slaveAddr, StopMode noStop, uint8_t numByteToWrite, uint8_t* dataBuffer)
		{
			// check if communication on going
			if (I2C->SR3 & I2C_SR3_BUSY)
				return false;
			// check if STATE MACHINE is in state INI_00
			if (state_ != INI_00)
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
			dataBuffer_  = dataBuffer;
			// set comunication Timeout
			set_tout_ms(I2C_TOUT);
			// generate Start
			I2C->CR2 |= I2C_CR2_START;
			state_ = SB_01;
			return true;
		}

		static bool Read(SlaveAddr_t slaveAddr, uint8_t numByteToRead, uint8_t* dataBuffer)
		{
			// check if communication on going
			if (I2C->SR3 & I2C_SR3_BUSY)
				return false;
			// check if STATE MACHINE is in state INI_00
			if (state_ != INI_00)
				return false;
			// set ACK
			I2C->CR2 |= I2C_CR2_ACK;
			// reset POS
			I2C->CR2 &= ~I2C_CR2_POS;
			// setup I2C comm. in Read
			direction_ = DirRead;
			// copy parameters for interrupt routines
			noStop_ = NoStop;
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
			/* Get Value of Status registers and Control register 2 */
			sr1 = I2C->SR1;
			sr2 = I2C->SR2;
			/* Check for error in communication */
			if (sr2 != 0)
			{
				ErrProc();
			}

			/* Start bit detected */
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
			/* ADD10*/
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
			/* ADDR*/
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
						/* Clear Add Ack Flag */
						dummy = I2C->SR3;
						// set No ACK
						I2C->CR2 &= ~I2C_CR2_ACK;
						state_ = BTF_17;
						break;
					}
					if (numByte_ == 1)
					{
						I2C->CR2 &= ~I2C_CR2_ACK;
						/* Clear Add Ack Flag */
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
					/* Clear Add Ack Flag */
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
			/* BTF */
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
			T4::Timer4::ClearIntFlag();
			if(TIM4_tout_)
				if(--TIM4_tout_ == 0)
				{
					ErrProc();
				}
		}
	};
	template<Mode mode, AddrType addrType>
	volatile i2c<mode, addrType>::States i2c<mode, addrType>::state_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t i2c<mode, addrType>::err_state_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t i2c<mode, addrType>::err_save_;

	template<Mode mode, AddrType addrType>
	volatile uint8_t i2c<mode, addrType>::TIM4_tout_;

	template<Mode mode, AddrType addrType>
	i2c<mode, addrType>::Direction i2c<mode, addrType>::direction_;

	template<Mode mode, AddrType addrType>
	uint8_t i2c<mode, addrType>::numByte_;

	template<Mode mode, AddrType addrType>
	uint8_t* i2c<mode, addrType>::dataBuffer_;

	template<Mode mode, AddrType addrType>
	i2c<mode, addrType>::SlaveAddr_t i2c<mode, addrType>::slaveAddr_;

	template<Mode mode, AddrType addrType>
	StopMode i2c<mode, addrType>::noStop_;



}//i2c
}//Mcudrv

#endif // I2C_H

