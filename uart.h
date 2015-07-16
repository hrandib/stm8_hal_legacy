#pragma once
#include "stm8s.h"
//#include "static_assert.h"
#include "gpio.h"
#include "cstring"

void utoa(uint16_t value, unsigned char* ptr, uint8_t base = 10);

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
		IrqTxCompletet = UART1_CR2_TCIEN,
		IrqRxne = UART1_CR2_RIEN,
		IrqIdle = UART1_CR2_ILIEN,
		IrqDefault = UART1_CR2_TCIEN | UART1_CR2_RIEN	//TX complete and RX not empty
	};
	namespace Internal
	{

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

	template<uint16_t base>
	struct Base;
	template<> struct Base<UART1_BaseAddress>
	{
		typedef UART1_TypeDef type;
	};
	template<> struct Base<UART2_BaseAddress>
	{
		typedef UART2_TypeDef type;
	};

	}//Internal

	//class based on polling routines
	template<typename DEpin = Nullpin>
	class Uart
	{
	protected:
		typedef Internal::UartTraits<DEpin> ControlPin;
	public:
		static const uint16_t BaseAddr =
   #if defined (STM8S103) || defined (STM8S003)
			   UART1_BaseAddress
   #elif defined (STM8S105)
			   UART2_BaseAddress
   #endif
				;
		typedef Internal::Base<BaseAddr>::type BaseType;

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
			Regs()->CR3 = (static_cast<uint32_t>(config) >> 16) & 0xFF;
			Regs()->CR5 = (static_cast<uint32_t>(config) >> 24) & 0xFF;
			Regs()->CR2 = (static_cast<uint32_t>(config) >> 8) & 0xFF;
//			EnableInterrupt<DefaultInts>();
			ControlPin::SetConfig();
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
		
		#pragma inline=forced
		static bool IsBusy()
		{
			return Regs()->CR2 & IrqTxEmpty;
		}		
			
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
			while(!IsEvent(EvTxEmpty));
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
#define USARTECHO
		static uint8_t Getch()
		{
			while (!IsEvent(EvRxne));
			uint8_t ch = Regs()->DR;
#ifdef USARTECHO
			Regs()->DR = ch;
#endif
			return ch;
		}
	};

	//class based on interrupts and circular buffer
	template<typename T>
	class UartIrq : public Uart<T>
	{

	};

// Int driven functions
/*		static void Putbuf(const uint8_t *buf, uint8_t size)
		{
			while (IsBusy());
			ControlPin::Set();
			size_ = size;
			pBuf_ = buf;
			Regs()->DR = buf[0];
			EnableInterrupt(TxEmptyInt);
		}
		#pragma inline=forced
		template<typename T>
		static void Putbuf(T *buf, uint8_t size)
		{
			static_assert(sizeof(T) == 1, "Type size for Putbuf func must be 1");
			Putbuf(reinterpret_cast<const uint8_t* >(buf), size);
		}

		template<typename T>
		static void Puts(T *str)
		{
			static_assert(sizeof(T) == 1, "Type size for Puts func must be 1");
			Putbuf(reinterpret_cast<const char*>(str), strlen(reinterpret_cast<const char*>(str)));
		}

		static void Puts(uint16_t value)
		{
			uint8_t buf[6];
			utoa(value, buf);
			Puts(buf);
		}

		template<typename T>
		static void Putbyte(T byte)
		{
			static_assert(sizeof(byte) == 1, "Type size for Putbyte func must be 1");
			Putbuf(&byte, 1);
		}

		static void Newline()
		{
			Puts("\r\n");
		}
*/
/*#if defined (STM8S103) || defined (STM8S003)
			_Pragma("vector=17")
#elif defined (STM8S105)
			_Pragma("vector=20")
#endif
		__interrupt static void TxIRQ()
		{		
			static uint8_t count;
			if (IsEvent(EvTxComplete))
			{
				ClearEvent(EvTxComplete);
				ControlPin::Clear();
			}
			else //if (IsEvent(TxEmpty))
			{
				if (++count < size_)
				{
					Regs()->DR = pBuf_[count];
				}
				else
				{
					DisableInterrupt(TxEmptyInt);
					count = 0;
				}
			}

		}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma("vector=18")
#elif defined (STM8S105)
			_Pragma("vector=21")
#endif
			__interrupt static void RxIRQ()
		{
			bool error = IsEvent(static_cast<Events>(ParityErr | FrameErr | NoiseErr | OverrunErr)); //чтение флагов ошибок
			if(error) return;
			uint8_t temp = Regs()->DR;
			//Rxbuf::Push(temp);
			Regs()->DR = temp;			//echo
		}
	};
	
	template<typename DEpin>
	uint8_t Uart<DEpin>::size_;
	template<typename DEpin>
	uint8_t const *Uart<DEpin>::pBuf_;
*/
}//Uarts
}//Mcudrv

void utoa(int32_t value, unsigned char* ptr, uint8_t base) //TODO: move to source file
{
	uint32_t quotient = value < 0 ? -value : value;
	unsigned char *ptr1 = ptr, tmp_char;
	// check that the base if valid
	if (base < 2 || base > 36) { *ptr = '\0'; return; }
	do {
		const uint32_t q = quotient / base;
		const uint32_t rem = quotient - q * base;
		quotient = q;
		*ptr++ = (rem < 10 ? '0' : 'a' - 10) + rem;
	} while ( quotient );
	*ptr-- = '\0';
	while (ptr1 < ptr)
	{
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return;
}

