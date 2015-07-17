#pragma once

#include "stdint.h"
#include "stm8s.h"

const uint32_t F_CPU = 16000000UL;

namespace Mcudrv {
namespace SysClock {
	enum RefSource
	{
		HSE = 0xB4,
		HSI = 0xE1, //Default
		LSI = 0xD2
	};
	enum HsiDiv
	{
		Div1, Div2, Div4, Div8,
		DivNoChange = 0xFF
	};

	#pragma inline=forced
	inline static void Init(const RefSource ref, const HsiDiv div = DivNoChange)
	{
		CLK->SWCR |= CLK_SWCR_SWEN;
		CLK->SWR = ref;
		if(div != DivNoChange) CLK->CKDIVR = div << 3U;
		while(CLK->SWCR & CLK_SWCR_SWBSY)
			;
	}

}//Clock
}//Mcudrv
