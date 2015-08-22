#pragma once
#include "stm8s.h"
#include "gpio.h"
#include "cstring"
#include "circularBuffer.h"
#include "string_utils.h"



namespace Mcudrv
{

namespace Uarts
{
	typedef uint32_t BaudRate;

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
		EvParityErr = UART1_SR_PE,
		EvFrameErr = UART1_SR_FE,
		EvNoiseErr = UART1_SR_NF,
		EvOverrunErr = UART1_SR_OR,
		EvIdle = UART1_SR_IDLE,
		EvRxne = UART1_SR_RXNE,
		EvTxComplete = UART1_SR_TC,
		EvTxEmpty = UART1_SR_TXE
	};

	enum Irqs
	{
		IrqParityEnable = UART1_CR1_PIEN,
//		---================---			
		IrqTxEmpty = UART1_CR2_TIEN,
		IrqTxComplete = UART1_CR2_TCIEN,
		IrqRxne = UART1_CR2_RIEN,
		IrqIdle = UART1_CR2_ILIEN,
		IrqDefault = UART1_CR2_TCIEN | UART1_CR2_RIEN	//TX complete and RX not empty
	};
	namespace Internal
	{
		template<uint16_t base>	struct Base;
		template<> struct Base<UART1_BaseAddress>
		{
			typedef UART1_TypeDef type;
			typedef Pd5 TxPin;
			typedef Pd6 RxPin;
		};
		template<> struct Base<UART2_BaseAddress>
		{
			typedef UART2_TypeDef type;
			typedef Pd5 TxPin;
			typedef Pd6 RxPin;
		};
	}//Internal

	//class based on polling routines
	class Uart
	{
	public:
		static const uint16_t BaseAddr =
#if defined (STM8S103) || defined (STM8S003)
			   UART1_BaseAddress
#elif defined (STM8S105)
			   UART2_BaseAddress
#endif
				;
		typedef Internal::Base<BaseAddr>::type BaseType;
		typedef Internal::Base<BaseAddr>::TxPin TxPin;
		typedef Internal::Base<BaseAddr>::RxPin RxPin;

		#pragma inline=forced
		static BaseType* Regs()
		{
			return reinterpret_cast<BaseType*>(BaseAddr);
		}
		#pragma inline=forced
		template<Cfg config, BaudRate baud = 9600UL>
		static void Init()
		{
			enum { Div = F_CPU / baud };
			static_assert(Div <= __UINT16_T_MAX__ && Div > 0x0F, "UART divider not in range 16...65535");
			static_assert(!(BaseAddr == UART2_BaseAddress && (static_cast<uint32_t>(config) >> 24) & UART1_CR5_HDSEL),
						  "Single wire Halfduplex mode not available for UART2");
			Regs()->BRR2 = ((Div >> 8U) & 0xF0) | (Div & 0x0F);
			Regs()->BRR1 = (Div >> 4U) & 0xFF;
			Regs()->CR1 = static_cast<uint32_t>(config) & 0xFF;
		//	Regs()->CR3 = (static_cast<uint32_t>(config) >> 16) & 0xFF; //Need for synchronuos communication and LIN
			Regs()->CR5 = (static_cast<uint32_t>(config) >> 24) & 0xFF;
			Regs()->CR2 = (static_cast<uint32_t>(config) >> 8) & 0xFF;

			TxPin::SetConfig<GpioBase::Out_PushPull_fast>();
			RxPin::SetConfig<GpioBase::In_Pullup>();
		}

		#pragma inline=forced
		static void SetNodeAddress(const uint8_t addr)	// Incompatible With LIN mode
		{
			Regs()->CR4 = addr;
		}

		#pragma inline=forced
		static bool IsEvent(const Events event)
		{
			return Regs()->SR & event;
		}
		
//		#pragma inline=forced
//		static bool IsBusy()
//		{
//			return Regs()->CR2 & IrqTxEmpty;
//		}

		#pragma inline=forced
		static void ClearEvent(const Events event)
		{
			Regs()->SR &= ~event;
		}

		#pragma inline=forced
		static void EnableInterrupt(const Irqs mask)
		{
			Regs()->CR2 |= mask;
		}
		#pragma inline=forced
		static void DisableInterrupt(const Irqs mask)
		{
			Regs()->CR2 &= ~mask;
		}

		static void Putch(const uint8_t ch)
		{
			while(!IsEvent(EvTxEmpty))
				;
			Regs()->DR = ch;
		}
		static void Puts(const uint8_t* s)
		{
			while(*s)
			{
				Putch(*s++);
			}
		}
		template<typename T>
		static void Puts(const T* s)
		{
			static_assert(sizeof(T) == 1, "Wrong type for Puts");
			Puts((const uint8_t*)s);
		}
		static void Putbuf(const uint8_t *buf, uint16_t size)
		{
			while(size--)
			{
				Putch(*buf++);
			}
		}
		static void Putbuf(const uint8_t *buf, uint8_t size)
		{
			while(size--)
			{
				Putch(*buf++);
			}
		}
		static uint8_t Getch()
		{
			while (!IsEvent(EvRxne));
			uint8_t ch = Regs()->DR;
#ifdef UARTECHO
			Regs()->DR = ch;
#endif
			return ch;
		}
	};

	//class based on interrupts and circular buffer
	template<uint16_t TxBufSize = 16, uint16_t RxBufSize = TxBufSize, typename DEpin = Nullpin>
	class UartIrq : public Uart
	{
	protected:
		typedef Uart Base;
		typedef UartIrq<TxBufSize, RxBufSize, DEpin> Self;
		static CircularBuffer<TxBufSize> txbuf_;
		static CircularBuffer<RxBufSize> rxbuf_;
	public:
		enum
		{
			TXBUFSIZE = TxBufSize,
			RXBUFSIZE = RxBufSize
		};

		#pragma inline=forced
		template<Cfg config, BaudRate baud = 9600UL>
		static void Init()
		{
			Base::Init<config, baud>();
			DEpin::template SetConfig<GpioBase::Out_PushPull_fast>();
			EnableInterrupt(Irqs(IrqRxne | IrqTxComplete));
		}

		static bool Putch(const uint8_t c)
		{
			bool st = txbuf_.Write(c);
			EnableInterrupt(IrqTxEmpty);
			return st;
		}
		static bool Puts(const uint8_t* s)
		{
			while(*s)
			{
				if(!txbuf_.Write(*s++)) return false;
			}
			EnableInterrupt(IrqTxEmpty);
			return true;
		}
		static bool Puts(const char* s)
		{
			return Puts((const uint8_t*)s);
		}
		template<typename T>
		static bool Puts(T value, uint8_t base = 10)
		{
			uint8_t buf[16];
			return Puts(io::xtoa(value, buf, base));
		}

		static bool Putbuf(const uint8_t* buf, uint16_t size)
		{
			while(size--)
			{
				if(!txbuf_.Write(*buf++)) return false;
			}
			EnableInterrupt(IrqTxEmpty);
			return true;
		}

		#pragma inline=forced
		template<typename T>
		static void Putbuf(T *buf, uint8_t size)
		{
			static_assert(sizeof(T) == 1, "Type size for Putbuf func must be 1");
			Putbuf((const uint8_t*)buf, size);
		}

		static void Newline()
		{
			Puts("\r\n");
		}
		#pragma inline=forced
		static bool Getch(uint8_t &c)
		{
			return rxbuf_.Read(c);
		}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma(VECTOR_ID(UART1_T_TXE_vector))
#elif defined (STM8S105)
			_Pragma(VECTOR_ID(UART2_T_TXE_vector))
#endif
		__interrupt static void TxIRQ()
		{
			if (IsEvent(EvTxComplete))
			{
				ClearEvent(EvTxComplete);
				DEpin::Clear();
			}
			else //if (IsEvent(TxEmpty))
			{
				uint8_t c;
				if(txbuf_.Read(c))
					Regs()->DR = c;
				else
					DisableInterrupt(IrqTxEmpty);
			}
			__no_operation();
		}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma(VECTOR_ID(UART1_R_RXNE_vector))
#elif defined (STM8S105)
			_Pragma(VECTOR_ID(UART2_R_RXNE_vector))
#endif
			__interrupt static void RxIRQ()
		{
//			bool error = IsEvent(Events(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr)); //чтение флагов ошибок
			uint8_t c = Regs()->DR;
//			error |= !					//buffer is full
			rxbuf_.Write(c);
//			if(error) return;
#ifdef UARTECHO
			Regs()->DR = c;			//echo
#endif
		}
	};

	template<uint16_t TxBufSize, uint16_t RxBufSize, typename DEpin>
	CircularBuffer<TxBufSize> UartIrq<TxBufSize, RxBufSize, DEpin>::txbuf_;
	template<uint16_t TxBufSize, uint16_t RxBufSize, typename DEpin>
	CircularBuffer<RxBufSize> UartIrq<TxBufSize, RxBufSize, DEpin>::rxbuf_;


}//Uarts
}//Mcudrv


