#pragma once
#ifndef PINLIST_H
#define PINLIST_H

#include "gpio.h"

namespace Mcudrv
{

template<typename S0 = Nullpin, typename S1 = Nullpin, typename S2 = Nullpin, typename S3 = Nullpin,
			typename S4 = Nullpin, typename S5 = Nullpin, typename S6 = Nullpin, typename S7 = Nullpin>
struct Pinlist : GpioBase
{
	static uint8_t ReadODR()
	{
		uint8_t value = 0;
		if (S0::IsODRSet()) value |= 0x01;
		if (S1::IsODRSet()) value |= 0x02;
		if (S2::IsODRSet()) value |= 0x04;
		if (S3::IsODRSet()) value |= 0x08;
		if (S4::IsODRSet()) value |= 0x10;
		if (S5::IsODRSet()) value |= 0x20;
		if (S6::IsODRSet()) value |= 0x40;
		if (S7::IsODRSet()) value |= 0x80;
		return value;
	}
	static uint8_t Read()
	{
		uint8_t value = 0;
		if (S0::IsSet()) value |= 0x01;
		if (S1::IsSet()) value |= 0x02;
		if (S2::IsSet()) value |= 0x04;
		if (S3::IsSet()) value |= 0x08;
		if (S4::IsSet()) value |= 0x10;
		if (S5::IsSet()) value |= 0x20;
		if (S6::IsSet()) value |= 0x40;
		if (S7::IsSet()) value |= 0x80;
		return value;
	}
	static void Toggle(uint8_t mask)
	{
		if (mask & 0x01) S0::Toggle();
		if (mask & 0x02) S1::Toggle();
		if (mask & 0x04) S2::Toggle();
		if (mask & 0x08) S3::Toggle();
		if (mask & 0x10) S4::Toggle();
		if (mask & 0x20) S5::Toggle();
		if (mask & 0x40) S6::Toggle();
		if (mask & 0x80) S7::Toggle();
	}
	static void Set(uint8_t mask)
	{
		if (mask & 0x01) S0::Set();
		if (mask & 0x02) S1::Set();
		if (mask & 0x04) S2::Set();
		if (mask & 0x08) S3::Set();
		if (mask & 0x10) S4::Set();
		if (mask & 0x20) S5::Set();
		if (mask & 0x40) S6::Set();
		if (mask & 0x80) S7::Set();
	}
	static void Clear(uint8_t mask)
	{
		if (mask & 0x01) S0::Clear();
		if (mask & 0x02) S1::Clear();
		if (mask & 0x04) S2::Clear();
		if (mask & 0x08) S3::Clear();
		if (mask & 0x10) S4::Clear();
		if (mask & 0x20) S5::Clear();
		if (mask & 0x40) S6::Clear();
		if (mask & 0x80) S7::Clear();
	}
	static void Write(uint8_t value)
	{
		if (value & 0x01) S0::Set(); else S0::Clear();
		if (value & 0x02) S1::Set(); else S1::Clear();
		if (value & 0x04) S2::Set(); else S2::Clear();
		if (value & 0x08) S3::Set(); else S3::Clear();
		if (value & 0x10) S4::Set(); else S4::Clear();
		if (value & 0x20) S5::Set(); else S5::Clear();
		if (value & 0x40) S6::Set(); else S6::Clear();
		if (value & 0x80) S7::Set(); else S7::Clear();
	}
	#pragma inline=forced
	template<Cfg cfg>
	static void SetConfig()
	{
		S0::template SetConfig<cfg>();
		S1::template SetConfig<cfg>();
		S2::template SetConfig<cfg>();
		S3::template SetConfig<cfg>();
		S4::template SetConfig<cfg>();
		S5::template SetConfig<cfg>();
		S6::template SetConfig<cfg>();
		S7::template SetConfig<cfg>();
	}
	#pragma inline=forced
	template<uint8_t mask_, Cfg cfg>
	static void SetConfig()
	{
		if(mask_ & 0x01) S0::template SetConfig<cfg>();
		if(mask_ & 0x02) S1::template SetConfig<cfg>();
		if(mask_ & 0x04) S2::template SetConfig<cfg>();
		if(mask_ & 0x08) S3::template SetConfig<cfg>();
		if(mask_ & 0x10) S4::template SetConfig<cfg>();
		if(mask_ & 0x20) S5::template SetConfig<cfg>();
		if(mask_ & 0x40) S6::template SetConfig<cfg>();
		if(mask_ & 0x80) S7::template SetConfig<cfg>();
	}
};

template<uint8_t seq>
struct SequenceOf
{
	enum { value = seq };
};

template<typename First, uint8_t Seq>
struct Pinlist<First, SequenceOf<Seq> > : GpioBase
{
	typedef typename First::Port Port;
	enum
	{
		offset = First::position,
		mask = stdx::NumberToMask<Seq>::value << offset
	};
	#pragma inline=forced
	static uint16_t ReadODR()
	{
		return (Port::ReadODR() & mask) >> offset;
	}
	#pragma inline=forced
	static uint16_t Read()
	{
		return (Port::Read() & mask) >> offset;
	}
	#pragma inline=forced
	static void Write(uint16_t value)
	{
		value = (value << offset) & mask;
		Port::ClearAndSet(~value & mask, value);
	}
	#pragma inline=forced
	template<uint8_t mask_, Cfg cfg>
	static void SetConfig()
	{
		Port::template SetConfig<(uint8_t)(mask_ << offset), cfg>();
	}
	#pragma inline=forced
	template<Cfg cfg>
	static void SetConfig()
	{
		Port::template SetConfig<mask, cfg>();
	}
};

}//Mcudrv
#endif // PINLIST_H

