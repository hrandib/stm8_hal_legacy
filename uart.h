#pragma once
#include "stm8s.h"
#include "static_assert.h"
#include "gpio.h"
#include "cstring"

//void utoa(uint16_t value, unsigned char* ptr, uint8_t base = 10);

namespace Mcudrv
{

namespace Uarts
{
	typedef uint16_t BaudRate;

	enum Cfg
	{

//		---=== UART CR1 ===---			

//		ParityIntEnable = UART1_CR1_PIEN,
//		ParityControl = UART1_CR1_PCEN,
		WakeIdle = 0,
		WakeAddressMark = UART1_CR1_WAKE,
		
		DataBits8 = 0,
		DataBits9 = UART1_CR1_M,

		UartDisable = UART1_CR1_UARTD,

		T8 = UART1_CR1_T8,
		R8 = UART1_CR1_R8,
	
		EvenParity = 0,
		OddParity  = UART1_CR1_PS,

//		---=== UART CR2 ===---			

		RxEnable = UART1_CR2_REN << 8,
		TxEnable = UART1_CR2_TEN << 8,
		RxTxEnable  = RxEnable | TxEnable,

//		RxWakeup = UART1_CR2_RWU,
//		SendBreak = UART1_CR2_SBK,

//		---=== UART CR3 ===---			

		LinEnable = static_cast<uint32_t>(UART1_CR3_LINEN) << 16UL ,
		OneStopBit         = 0,
		TwoStopBits        = static_cast<uint32_t>(0x02 << 4U) << 16UL,
		OneAndHalfStopBits = static_cast<uint32_t>(0x03 << 4U) << 16UL,

		SclkEnable = static_cast<uint32_t>(UART1_CR3_CKEN) << 16UL,
	// Should not be written while the transmitter is enabled
		CPOL_0_Idle = 0,				// Clock Polarity - 0 when Idle
		CPOL_1_Idle = static_cast<uint32_t>(UART1_CR3_CPOL) << 16UL,	// Clock Polarity - 1 when Idle
		CPHA_0 = 0,						// The first clock transition is the first data capture edge
		CPHA_1 = static_cast<uint32_t>(UART1_CR3_CPHA) << 16UL,		// The second clock transition is the first data capture edge
		LastBitPulseOn = static_cast<uint32_t>(UART1_CR3_LBCL) << 16UL,

//		---=== UART CR5 ====---			

		SingleWireMode = static_cast<uint32_t>(UART1_CR5_HDSEL) << 24UL,

		DefaultCfg = RxTxEnable | DataBits8 | OneStopBit,
	};

	enum Events
	{
		ParityErr = UART1_SR_PE,
		FrameErr = UART1_SR_FE,
		NoiseErr = UART1_SR_NF,
		OverrunErr = UART1_SR_OR,
		Idle = UART1_SR_IDLE,
		Rxne = UART1_SR_RXNE,
		TxComplete = UART1_SR_TC,
		TxEmpty = UART1_SR_TXE
	};

	enum Ints
	{
		ParityIntEnable = UART1_CR1_PIEN,
//		---================---			
		TxEmptyInt = UART1_CR2_TIEN,
		TxCompleteInt = UART1_CR2_TCIEN,
		RxneInt = UART1_CR2_RIEN,
		IdleInt = UART1_CR2_ILIEN,
		DefaultInts = UART1_CR2_TCIEN | UART1_CR2_RIEN	//TX complete and RX not empty
	};
	
	template<uint8_t size = 24>
	class UartBuf
	{
	private:
		static uint8_t buf[size];
		static uint8_t labelc;
		static uint8_t count;
	public:
		#pragma inline=forced
		static void Init()
		{
			buf[0] = 0;
		}
		#pragma inline=forced
		template<typename T>
		static void Push(T byte)
		{
			BOOST_STATIC_ASSERT(sizeof(T) == 1);
			if (byte == '\r') return;
			if (byte == '\n')
			{
				buf[count] = 0;
				labelc++;
			}
			else 
				buf[count] = byte;
			if (++count == 24)
				count = 0;
		}
		
		static uint8_t* Pop()	//TODO: Rewrite, not work properly
		{
			uint8_t countl = count - 1;
			while (buf[--countl])
			{
				if (!countl) countl = size;
			}
			if (labelc) labelc--;
			return &buf[++countl];			//Bug at the bound
		}

		static bool IsNotEmpty()
		{
			return labelc;
		}
	};

	template<uint8_t size>
	uint8_t UartBuf<size>::buf[];
	template<uint8_t size>
	uint8_t UartBuf<size>::labelc = 0;
	template<uint8_t size>
	uint8_t UartBuf<size>::count = 1;

	typedef UartBuf<> Rxbuf;

	template<typename Pin>
	class UartTraits
	{
	public:
		#pragma inline=forced
		static void Set()
		{
			Pin::Set();
		}
		#pragma inline=forced
		static void Clear()
		{
			Pin::Clear();
		}
		#pragma inline=forced
		static void SetConfig()
		{
			Pin::template SetConfig<GpioBase::Out_PushPull_fast>();
			Pin::Clear();
		}
	};

	template<>
	class UartTraits<Nullpin>
	{
	public:
		#pragma inline=forced
		static void Set(){}
		#pragma inline=forced
		static void Clear(){}
		#pragma inline=forced
		static void SetConfig(){}
	};
	
	typedef void (*cbRxFunc)(uint8_t);

 	#pragma inline=forced
 	void EmptyRxFunc(uint8_t){}

	template<uint16_t BaseAddr, typename DEpin = Nullpin>
	class Uart
	{
	private:
		static const uint8_t *pBuf_;
		static uint8_t size_;
	public:
		#pragma inline=forced
		static UART1_TypeDef* GetBaseAddr()
		{
			return reinterpret_cast<UART1_TypeDef*>(BaseAddr);
		}
//		static cbRxFunc GetChar;
		
		#pragma inline=forced
		template<Cfg config, /*cbRxFunc Getch = EmptyRxFunc,*/ BaudRate baud = 9600UL>
		static void Init()
		{
			enum{Div = F_CPU/baud};
			BOOST_STATIC_ASSERT(Div <= __UINT16_T_MAX__ && Div > 0x0F);		//Divider in Range 16...65535 
			BOOST_STATIC_ASSERT(!(BaseAddr == UART2_BaseAddress && (static_cast<uint32_t>(config) >> 24) & UART1_CR5_HDSEL)); // UART2 doesn't have SingleWire mode
//			GetChar = Getch;
			GetBaseAddr()->BRR2 = ((Div >> 8U) & 0xF0) | (Div & 0x0F);		
			GetBaseAddr()->BRR1 = (Div >> 4U) & 0xFF;
			GetBaseAddr()->CR1 = static_cast<uint32_t>(config) & 0xFF;
			GetBaseAddr()->CR3 = (static_cast<uint32_t>(config) >> 16) & 0xFF;
			GetBaseAddr()->CR5 = (static_cast<uint32_t>(config) >> 24) & 0xFF;
			GetBaseAddr()->CR2 = (static_cast<uint32_t>(config) >> 8) & 0xFF;
//			EnableInterrupt<DefaultInts>();
			UartTraits<DEpin>::SetConfig();
//			Rxbuf::Init();
		}

		#pragma inline=forced
		template<uint8_t addr>
		static void SetNodeAddress()	// Incompatible With LIN mode
		{
			GetBaseAddr()->CR4 = addr;				
		}

		#pragma inline=forced
		static bool IsEvent(const Events event)
		{
			return GetBaseAddr()->SR & event;
		}
		
		#pragma inline=forced
		static bool IsBusy()
		{
			return GetBaseAddr()->CR2 & TxEmptyInt;
		}		
			
		#pragma inline=forced
		static void ClearEvent(const Events event)
		{
			GetBaseAddr()->SR &= ~event;
		}

		#pragma inline=forced
		static void EnableInterrupt(const Ints mask)
		{
			GetBaseAddr()->CR2 |= mask;
		}
		#pragma inline=forced
		static void DisableInterrupt(Ints mask)
		{
			GetBaseAddr()->CR2 &= ~mask;
		}

// Int driven functions
		static void Putbuf(const uint8_t *buf, uint8_t size)
		{
			while (IsBusy());
			UartTraits<DEpin>::Set();
			size_ = size;
			pBuf_ = buf;
			GetBaseAddr()->DR = buf[0];
			EnableInterrupt(TxEmptyInt);
		}
		#pragma inline=forced
		template<typename T>
		static void Putbuf(T *buf, uint8_t size)
		{
			STATIC_ASSERT(sizeof(T) == 1);
			Putbuf(reinterpret_cast<const uint8_t* >(buf), size);
		}

		template<typename T>
		static void Puts(T *str)
		{
			STATIC_ASSERT(sizeof(T) == 1);
			Putbuf(reinterpret_cast<const char*>(str), strlen(reinterpret_cast<const char*>(str)));
		}

//		static void Puts(uint16_t value)
//		{
//			uint8_t buf[6];
//			utoa(value, buf);
//			Puts(buf);
//		}

		template<typename T>
		static void Putbyte(T byte)
		{
			STATIC_ASSERT(sizeof(byte) == 1);
			Putbuf(&byte, 1);
		}

		static void Newline()
		{
			Puts("\r\n");
		}

		#pragma inline=forced
		static void TxIRQ()
		{		
			static uint8_t count;
			if (IsEvent(TxComplete))
			{
				ClearEvent(TxComplete);
				UartTraits<DEpin>::Clear();
			}
			else //if (IsEvent(TxEmpty))
			{
				if (++count < size_)
				{
					GetBaseAddr()->DR = pBuf_[count];
				}
				else
				{
					DisableInterrupt(TxEmptyInt);
					count = 0;
				}
			}

		}

		#pragma inline=forced
		static void RxIRQ()
		{
			bool error = IsEvent(static_cast<Events>(ParityErr | FrameErr | NoiseErr | OverrunErr)); //чтение флагов ошибок
			if(error) return;
			uint8_t temp = GetBaseAddr()->DR;
//			if (GetChar) GetChar(temp);
			//Rxbuf::Push(temp);
			GetBaseAddr()->DR = temp;			//echo
		}

		// Functions with polling
//		static void Putch(char ch)
// 		{
// 			while(!(GetBaseAddr()->SR & UART1_SR_TXE));
// 			GetBaseAddr()->DR = ch;
// 		}
// 		static void Puts(char *s)
// 		{
// 			while(*s)
// 			{
// 				Putch(*s++);
// 			}
// 
// 		}
// 		static void Putbuf(uint8_t *buf, uint16_t size)
// 		{
// 			while(size--)
// 			{
// 				Putch(*buf++);
// 			}
// 		}
// 	
// 		static uint8_t Getch()
// 		{
// 			uint8_t x = GetBaseAddr()->DR;
// 			while (!(GetBaseAddr()->SR & UART1_SR_RXNE));
// 			return GetBaseAddr()->DR;
// 		}
	};
	
 	template<uint16_t BaseAddr, typename DEpin>
 	uint8_t Uart<BaseAddr, DEpin>::size_;
 	template<uint16_t BaseAddr, typename DEpin>
	uint8_t const *Uart<BaseAddr, DEpin>::pBuf_;
//	template<uint16_t BaseAddr, typename DEpin>
//	cbRxFunc Uart<BaseAddr, DEpin>::GetChar = EmptyRxFunc;

#ifndef USE_CUSTOM_UART_IRQ
#if defined (STM8S103) || defined (STM8S003)
	
	typedef Uart<UART1_BaseAddress> Uart1;
	INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
	{
		Uart1::TxIRQ();
	}
	INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
	{
		Uart1::RxIRQ();
	}
#endif

#if defined (STM8S105)
	
	typedef Uart<UART2_BaseAddress> Uart2;
	INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
	{
		 Uart2::TxIRQ();
	}
	INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
	{
		 Uart2::RxIRQ();
 	}

#endif
#endif
// 	class Rs485: public Uart1
// 	{
// 	private:	
// 		typedef Pd5 RxTx;
// 		typedef Pd6 TxModePin;
// 	public:
// 		#pragma inline=forced
// 		static void Init()
// 		{
// 			TxModePin::SetConfig<GpioBase::Out_PushPull>();
// 			RxTx::SetConfig<GpioBase::In_float>();
// 			Uart1::Init<static_cast<Cfg>(RxTxEnable | SingleWireMode)>();
// 		}
// 		static void Putch(char ch)
// 		{
// 			//				Uart1::ReceiverDisable();
// 			TxModePin::Set();
// 			Uart1::Putch(ch);
// 			while (!isEvent<TxComplete>());
// 			TxModePin::Clear();
// 			//				Uart1::ReceiverEnable();
// 		}
// 		static void Puts(char *s)
// 		{
// 			TxModePin::Set();
// 			Uart1::Puts(s);
// 			while (!isEvent<TxComplete>());
// 			TxModePin::Clear();
// 		}
// 		static void Putbuf(uint8_t *buf, uint16_t size)
// 		{
// 			TxModePin::Set();
// 			Uart1::Putbuf(buf, size);
// 			while (!isEvent<TxComplete>());
// 			TxModePin::Clear();
// 		}
// 	};
}//Uarts
}//Mcudrv
/*
void utoa(uint16_t value, unsigned char* ptr, uint8_t base)
{
	uint16_t tmp_value;
	unsigned char *ptr1 = ptr, tmp_char;
	// check that the base if valid
	if (base < 2 || base > 36) { *ptr = '\0'; return; }
	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "0123456789abcdefghijklmnopqrstuvwxyz"[tmp_value - value * base]; //TODO: Eliminate string
	} while (value);
	*ptr-- = '\0';
	while (ptr1 < ptr)
	{
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return;
}
*/
