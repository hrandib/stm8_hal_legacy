#pragma once
#ifndef ITC_H
#define ITC_H

#include "stm8s.h"

namespace Mcudrv
{

	namespace Itc
	{
		enum Priority
		{
			prioLevel_1 = 0x01, //TODO: What the highest and lowest?
			prioLevel_2 = 0x00,
			prioLevel_3 = 0x03
		};
		#pragma inline=forced
		void SetPriority(uint8_t vector, const Priority priority)
		{
			vector -= 2;
			volatile uint8_t* reg = &((volatile uint8_t*)ITC)[vector / 4];
			const uint8_t position = (vector % 4) * 2;
			*reg = (*reg & ~(0x03 << position)) | priority << position;
		}

	}//Itc
}//Mcudrv

#endif // ITC_H

