#pragma once
#ifndef I2C_H
#define I2C_H

#include "stm8s.h"
#include "gpio.h"

//#define FAST_I2C_MODE

namespace Mcudrv
{

typedef Pd2 TestLed;

namespace n_i2c
{
	enum
	{
		MAX_DUMMY = 10,
		I2C_TOUT = 40,
		WRITE = 0,
		READ = 1,
		SEV_BIT_ADDRESS = 0,
		TEN_BIT_ADDRESS = 1,
		STOP = 0,
		NOSTOP = 1,
	};
	enum States
	{
		INI_00 = 0,
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

	enum Mode
	{
		Standard,
		Fast
	};

	class i2c
	{
	private:
		static volatile States state_;
		static volatile uint8_t err_state;  	// error state
		static volatile uint8_t err_save;   	// I2C->SR2 copy in case of error
		static volatile uint16_t TIM4_tout;  // Timout Value

		static u8  u8_regAddr ;
		static u8  u8_Direction;

		static u8  u8_NumByte_cpy;
		static u8* pu8_DataBuffer_cpy ;
		static u16 u16_SlaveAdd_cpy;
		static u8  u8_AddType_cpy;
		static u8  u8_NoStop_cpy;
		/* flag clearing sequence - uncoment next for peripheral clock under 2MHz */
		#pragma inline=forced
		static void dead_time()
		{
			__no_operation();
			__no_operation();
		}
		#pragma inline=forced
		static void delay(uint16_t a)
		{
			TIM4_tout = a;
			while(TIM4_tout)
				;
		}
		#pragma inline=forced
		static uint16_t tout()
		{
			return TIM4_tout;
		}
		#pragma inline=forced
		static void set_tout_ms(uint16_t a)
		{
			TIM4_tout = a;
		}

	public:
		static void Init()
		{
			//define SDA, SCL outputs, HiZ, Open drain, Fast
			//GpioD::SetConfig<P1 | P2, GpioBase::>();
			TestLed::Clear();
			GPIOE->ODR |= 0x06;
		  GPIOE->DDR |= 0x06;
		  GPIOE->CR2 |= 0x06;
#ifdef FAST_I2C_MODE
			I2C->FREQR = 16;               // input clock to I2C - 16MHz
			I2C->CCRL = 15;                // 900/62.5= 15, (SCLhi must be at least 600+300=900ns!)
			I2C->CCRH = 0x80;              // fast mode, duty 2/1 (bus speed 62.5*3*15~356kHz)
			I2C->TRISER = 5;               // 300/62.5 + 1= 5  (maximum 300ns)
#else
			I2C->FREQR = 16;                // input clock to I2C - 8MHz
			I2C->CCRL = 80;                // CCR= 40 - (SCLhi must be at least 4000+1000=5000ns!)
			I2C->CCRH = 0;                 // standard mode, duty 1/1 bus speed 100kHz
			I2C->TRISER = 17;               // 1000ns/(125ns) + 1  (maximum 1000ns)
#endif
			I2C->OARL = 0xA0;              // own address A0;
			I2C->OARH |= 0x40;
			I2C->ITR = 3;                  // enable Event & error interrupts
			I2C->CR2 |= 0x04;              // ACK=1, Ack enable
			I2C->CR1 |= 0x01;              // PE=1
			I2C->CR1 &= ~0x80;             // Stretch enable

			//TIM4_Init
			CLK->PCKENR1 |= 4;               // TIM4 clock enable
			TIM4->ARR = 0x80;                // init timer4 1ms interrupts
			TIM4->PSCR= 7;
			TIM4->IER = 1;
			TIM4->CR1 |= 1;

			// Initialise I2C State Machine
			err_save= 0;
//			state_ == INI_00;
			set_tout_ms(0);
		}

		static void ErrProc()
		{
			err_save = I2C->SR2 ;
			err_state = state_;
			I2C->SR2= 0;
			state_ = INI_00;
			set_tout_ms(0);
		}

		static u8 I2C_WriteRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop, u8 u8_NumByteToWrite, u8 *pu8_DataBuffer)
		{
			// check if communication on going
			if ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
				return 0 ;
			// check if STATE MACHINE is in state INI_00
			if (state_ != INI_00)
				return 0 ;
			// set ACK
			I2C->CR2 |= I2C_CR2_ACK;
			// reset POS
			I2C->CR2 &= ~I2C_CR2_POS;
			// setup I2C comm. in write
			u8_Direction = WRITE;
			// copy parametters for interrupt routines
			u8_NoStop_cpy = u8_NoStop;
			u16_SlaveAdd_cpy = u16_SlaveAdd;
			u8_NumByte_cpy = u8_NumByteToWrite;
			pu8_DataBuffer_cpy  = pu8_DataBuffer;
			u8_AddType_cpy = u8_AddType;
			// set comunication Timeout
			set_tout_ms(I2C_TOUT);
			// generate Start
			I2C->CR2 |= I2C_CR2_START;
			state_ = SB_01;
			return 1;
		}

		static u8 I2C_ReadRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop, u8 u8_NumByteToRead, u8 *u8_DataBuffer)
		{
			// check if communication on going
			if (((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY) && (u8_NoStop == 0))
				return 0 ;
			// check if STATE MACHINE is in state INI_00
			if (state_ != INI_00)
				return 0 ;
			// set ACK
			I2C->CR2 |= I2C_CR2_ACK;
			// reset POS
			I2C->CR2 &= ~I2C_CR2_POS;
			// setup I2C comm. in Read
			u8_Direction = READ;
			// copy parametters for interrupt routines
			u8_NoStop_cpy = u8_NoStop;
			u8_AddType_cpy = u8_AddType;
			u16_SlaveAdd_cpy = u16_SlaveAdd;
			u8_NumByte_cpy = u8_NumByteToRead;
			pu8_DataBuffer_cpy = u8_DataBuffer;
			// set comunication Timeout
			set_tout_ms(I2C_TOUT);
			//generate Start
			I2C->CR2 |= I2C_CR2_START;
			state_ = SB_11;
			I2C->ITR |= 3;                  // re-enable interrupt
			return 1;
		}

		_Pragma(VECTOR_ID(I2C_SB_vector))
		__interrupt static void I2CInterruptHandle()
		{
			u8 sr1, sr2;//, cr2;
			volatile uint8_t dummy, cr2;
			TestLed::Toggle();
			Green::Set();
			/* Get Value of Status registers and Control register 2 */
			sr1 = I2C->SR1;
			sr2 = I2C->SR2;
			cr2 = I2C->CR2;

			/* Check for error in communication */
			if (sr2 != 0)
			{
				ErrProc();
			}

			/* Start bit detected */
			if ((sr1 & I2C_SR1_SB) == 1)
			{
				switch(state_)
				{
				case SB_01:
					if (u8_AddType_cpy == TEN_BIT_ADDRESS)
					{
						I2C->DR = (u8)(((u16_SlaveAdd_cpy >> 7) & 6) | 0xF0);  // send header of 10-bit device address (R/W = 0)
						state_ = ADD10_02;
						break;
					} else {
						I2C->DR = (u8)(u16_SlaveAdd_cpy << 1);   // send 7-bit device address & Write (R/W = 0)
						state_ = ADDR_03;
						break;
					}
				case SB_11:
					if (u8_AddType_cpy == TEN_BIT_ADDRESS)
					{
						I2C->DR = (u8)(((u16_SlaveAdd_cpy >> 7) & 6) | 0xF1);// send header of 10-bit device address (R/W = 1)
					} else {
						I2C->DR = (u8)(u16_SlaveAdd_cpy << 1)|1 ; // send 7-bit device address & Write (R/W = 1)
					}
					state_ = ADDR_13;
					break;
				default : ErrProc();
					break;
				}
			}
			/* ADD10*/
			if (I2C->SR1 & I2C_SR1_ADD10) {
				switch(state_)
				{
				case ADD10_02:
					I2C->DR = (u8)(u16_SlaveAdd_cpy);                // send lower 8-bit device address & Write
					state_ = ADDR_03;
					break;

				default : ErrProc();
					break;
				}
			}
			/* ADDR*/
			if ((sr1 & I2C_SR1_ADDR) == I2C_SR1_ADDR)
			{
				switch (state_)
				{
				case ADDR_13 :

					if (u8_NumByte_cpy == 3)
					{
						dummy = I2C->SR3;
						state_ = BTF_15;
						break;
					}
					if (u8_NumByte_cpy == 2)
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
					if (u8_NumByte_cpy == 1)
					{
						I2C->CR2 &= ~I2C_CR2_ACK;
						/* Clear Add Ack Flag */
						dummy = I2C->SR3;
						I2C->CR2 |= I2C_CR2_STOP;
						I2C->ITR |= I2C_ITR_ITBUFEN;
						state_ = RXNE_18;
						break;
					}
					if (u8_NumByte_cpy > 3)
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
					I2C->DR = *pu8_DataBuffer_cpy++ ;
					u8_NumByte_cpy -- ;
					state_ = BTF_04;
					break;
				default : ErrProc();
					break;
				}
			}
			if ((sr1  & I2C_SR1_RXNE)==I2C_SR1_RXNE)
			{
				switch (state_)
				{
				case RXNE_18 :
					*(pu8_DataBuffer_cpy++) = I2C->DR;													// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				case RXNE_16 :
					*(pu8_DataBuffer_cpy++) = I2C->DR;                     			// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				}
				I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
			}
			/* BTF */
			if ((sr1 & I2C_SR1_BTF) == I2C_SR1_BTF)
			{
				switch (state_)
				{
				case BTF_17 :
					I2C->CR2 |= I2C_CR2_STOP;                   				// generate stop request here (STOP=1)
					*(pu8_DataBuffer_cpy++) = I2C->DR;											// Read next data byte
					*(pu8_DataBuffer_cpy++) = I2C->DR;											// Read next data byte
					state_ = INI_00;
					set_tout_ms(0);
					break;
				case BTF_14 :
					*(pu8_DataBuffer_cpy++) = I2C->DR;
					u8_NumByte_cpy --;
					if (u8_NumByte_cpy <= 3)
						state_ = BTF_15;
					break;
				case BTF_15 :
					I2C->CR2 &= ~I2C_CR2_ACK;                     		// Set NACK (ACK=0)
					*(pu8_DataBuffer_cpy++) = I2C->DR;                    // Read next data byte
					I2C->CR2 |= I2C_CR2_STOP;                        // Generate stop here (STOP=1)
					*(pu8_DataBuffer_cpy++) = I2C->DR;                    // Read next data byte
					I2C->ITR |= I2C_ITR_ITBUFEN; 										// Enable Buffer interrupts (errata)
					state_ = RXNE_16;
					break;
				case BTF_04 :
					if ((u8_NumByte_cpy) && ((I2C->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE))
					{
						I2C->DR = *pu8_DataBuffer_cpy++ ;												// Write next data byte
						u8_NumByte_cpy-- ;
						break;
					}
					else
					{
						if (u8_NoStop_cpy == 0)
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
			TestLed::Clear();	//test purpose
		}
		_Pragma(VECTOR_ID(TIM4_OVR_UIF_vector))
		__interrupt static void TIM4InterruptHandle()
		{
			u8 dly= 10;
			TIM4->SR1= 0;
			if(TIM4_tout)
				if(--TIM4_tout == 0)
				{
					__no_operation();
					ErrProc();
				}
			while(dly--)
				;
		}
	};

	volatile States i2c::state_;

	volatile uint8_t i2c::err_state;
	volatile uint8_t i2c::err_save;
	volatile uint16_t i2c::TIM4_tout;

	uint8_t i2c::u8_regAddr;
	uint8_t i2c::u8_Direction;
	uint8_t i2c::u8_NumByte_cpy;
	uint8_t* i2c::pu8_DataBuffer_cpy;
	uint16_t i2c::u16_SlaveAdd_cpy;
	uint8_t i2c::u8_AddType_cpy;
	uint8_t i2c::u8_NoStop_cpy;



}//i2c
}//Mcudrv

#endif // I2C_H

