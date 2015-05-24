#pragma once

#include <stdint.h>

namespace Mcudrv
{
	namespace Exti
	{
		enum Port
		{
			Porta,
			Portb,
			Portc,
			Portd,
			Porte
		};

		enum Cfg
		{
			LowLevel,
			RisingEdge,
			FallingEdge,
			RisingFallingEdge
		};
		
		#pragma inline=forced
		template<Port port, Cfg cfg>
		void SetExtIntMode()
		{
			if (port == Porte) EXTI->CR2 |= cfg;
			else EXTI->CR1 |= cfg << (port * 2);
		}
		#pragma inline=forced
		template<Port port>
		void ClearExtIntMode()
		{
			if (port == Porte) EXTI->CR2 &= ~(0x0F);
			else EXTI->CR1 |= ~(0x0F << (port * 2));
		}
	}


	class GpioBase
	{
	private:

	public:
		enum Cfg
		{
			In_float,
			In_float_int,
			In_Pullup,
			In_Pullup_int,
			Out_OpenDrain,
			Out_OpenDrain_fast,
			Out_PushPull,
			Out_PushPull_fast
		};
		enum DontCareConfiguration{ None };
	};


	template <uint16_t baseaddr, uint8_t ID>
	class Gpio: public GpioBase
	{
	private:
		#pragma inline=forced
		static GPIO_TypeDef* GetBase()
		{
			return reinterpret_cast<GPIO_TypeDef*>(baseaddr);
		}
	public:
		typedef Gpio Base;
		typedef uint8_t DataT;
		enum{ Width = 8 };
		static const uint8_t port_id = ID;

		#pragma inline=forced
		template <uint8_t mask, Cfg cfg>
		static void SetConfig()
		{
			if (cfg & 0x01)
			{
				GetBase()->CR2 |= mask;
			}
			else GetBase()->CR2 &= ~mask;
			if ((cfg >> 1) & 0x01)
			{
				GetBase()->CR1 |= mask;
			}
			else GetBase()->CR1 &= ~mask;
			if ((cfg >> 2) & 0x01)
			{
				GetBase()->DDR |= mask;
			}
			else GetBase()->DDR &= ~mask;
		}

		#pragma inline=forced
		template <uint8_t mask, Cfg cfg>
		static void WriteConfig()
		{
			GetBase()->CR2 = ((cfg & 0x01) * 0xff) & mask;
			GetBase()->CR1 = (((cfg >> 1) & 0x01) * 0xff) & mask;
			GetBase()->DDR = (((cfg >> 2) & 0x01) * 0xff) & mask;
		}
		
		#pragma inline=forced
		template <uint8_t value>
		static void Write()
		{
			GetBase()->ODR = value;
		}
		#pragma inline=forced
		static void Write(uint8_t value)
		{
			GetBase()->ODR = value;
		}

		#pragma inline=forced
		template <uint8_t mask>
		static void Set()
		{
			GetBase()->ODR |= mask;
		}
		#pragma inline=forced
		static void Set(uint8_t mask)
		{
			GetBase()->ODR |= mask;
		}

		#pragma inline=forced
		template <uint8_t mask>
		static void Clear()
		{
			GetBase()->ODR &= ~mask;
		}
		#pragma inline=forced
		static void Clear(uint8_t mask)
		{
			GetBase()->ODR &= ~mask;
		}

		#pragma inline=forced
		template <uint8_t mask>
		static void Toggle()
		{
			GetBase()->ODR ^= mask;
		}
		#pragma inline=forced
		static void Toggle(uint8_t mask)
		{
			GetBase()->ODR ^= mask;
		}
		
		#pragma inline=forced
		static uint8_t Read()
		{
			return GetBase()->IDR;
		}
		#pragma inline=forced
		static uint8_t ReadODR()
		{
			return GetBase()->ODR;
		}
		#pragma inline=forced
		template<uint8_t clearmask, uint8_t setmask>
		static void ClearAndSet()
		{
			const DataT value = setmask & ~clearmask;
			GetBase()->ODR = value;
		}
	};

#define PORTDEF(x,y) typedef Gpio<GPIO##x##_BaseAddress, y> Gpio##x	

	PORTDEF(A, 0);
	PORTDEF(B, 1);
	PORTDEF(C, 2);
	PORTDEF(D, 3);
	PORTDEF(E, 4);
	PORTDEF(F, 5);
	PORTDEF(G, 6);
	PORTDEF(H, 7);
	PORTDEF(I, 8);

	#define GPIOZ_BaseAddress 0
	PORTDEF(Z, 0xff);					//nullport

	template <typename PORT, uint8_t MASK>
	class TPin
	{
	public:
		typedef PORT Port;
		enum { Mask = MASK };
		enum { port_id = Port::port_id };
		
		#pragma inline=forced
		template <GpioBase::Cfg cfg>
		static void SetConfig()
		{
			Port::template SetConfig<Mask, cfg>();
		}
		
		#pragma inline=forced
		static void Set()
		{
			Port::template Set<Mask>();
		}
		#pragma inline=forced
		static void SetOrClear(bool cond)
		{
			if (cond) Port::template Set<Mask>();
			else Port::template Clear<Mask>();
		}
		#pragma inline=forced
		static void Clear()
		{
			Port::template Clear<Mask>();
		}
		#pragma inline=forced
		static void Toggle()
		{
			Port::template Toggle<Mask>();
		}
		#pragma inline=forced
		static bool IsSet()
		{
			return Port::Read() & Mask;
		}
		#pragma inline=forced
		static bool IsODRSet()
		{
			return Port::ReadODR() & Mask;
		}
	};

#define PINSDEF(x,y)	typedef TPin<Gpio##x, 0x01> P##y##0;\
						typedef TPin<Gpio##x, 0x02> P##y##1;\
						typedef TPin<Gpio##x, 0x04> P##y##2;\
						typedef TPin<Gpio##x, 0x08> P##y##3;\
						typedef TPin<Gpio##x, 0x10> P##y##4;\
						typedef TPin<Gpio##x, 0x20> P##y##5;\
						typedef TPin<Gpio##x, 0x40> P##y##6;\
						typedef TPin<Gpio##x, 0x80> P##y##7
	PINSDEF(A,a);
	PINSDEF(B,b);
	PINSDEF(C,c);
	PINSDEF(D,d);
	PINSDEF(E,e);
	PINSDEF(F,f);
	PINSDEF(G,g);
	PINSDEF(H,h);
	PINSDEF(I,i);
	typedef TPin<GpioZ, 0x0> Nullpin;

#define	P0	0x01
#define	P1	0x02
#define	P2	0x04
#define	P3	0x08
#define	P4	0x10
#define	P5	0x20
#define	P6	0x40
#define	P7	0x80

} //Mcudrv
