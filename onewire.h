#pragma once
#ifndef ONEWIRE_H
#define ONEWIRE_H

#include "gpio.h"
#include "delay.h"
#include "crc.h"

namespace Mcudrv {

template<typename OwPin>
class OneWire
{
private:
	enum InstructionSet
	{
		CmdSearchRom = 0xF0,
		CmdReadRom = 0x33,
		CmdMatchRom = 0x55,
		CmdSkipRom = 0xCC
	};
	#pragma inline=forced
	static void WriteBit(bool val)
	{
		disableInterrupts();
		OwPin::Clear();
		if(val) delay_us<10>();
		else delay_us<65>();
		OwPin::Set();
		enableInterrupts();
		if(val) delay_us<55>();
		else delay_us<5>();
	}
	#pragma inline=forced
	static bool ReadBit()
	{
		disableInterrupts();
		OwPin::Clear();
		delay_us<5>();
		OwPin::Set();
		delay_us<10>();
		bool result = OwPin::IsSet();
		enableInterrupts();
		delay_us<55>();
		return result;
	}
	static uint8_t Search(uint8_t romArr[][8], const uint8_t romsMaxNumber, const bool checkRom)
	{
		uint8_t lastCollision = (!checkRom ? 0 : 64);
		uint8_t bitIndex, byteIndex, byteMask;
		uint8_t idBit, idBitComp;
		uint8_t  searchDirection = 0;
		Crc::Crc8 crc;
		uint8_t* prevRom = (uint8_t*)romArr;
		uint8_t romIndex = 0;
		for(; romIndex < romsMaxNumber; ++romIndex)
		{
		// initialize for search
			uint8_t* const rom = &romArr[romIndex][0]; //8 byte buffer for current search
			bitIndex = 1;	//Current bit search, start at 1, end at 64 (for convenience)
			byteIndex = 0; //Current byte search in
			byteMask = 1;
			crc.Reset();
			if(!Reset())
			{
				return 0;
			}
			Write(CmdSearchRom);
			bool lastDevice = true;
			do
			{
				// read a bit and its complement
				idBit = ReadBit();
				idBitComp = ReadBit();
				// check for no devices on 1-wire
				if(idBit && idBitComp)
					return 0;
				// all devices coupled have 0 or 1
				if(idBit != idBitComp)
					searchDirection = idBit;  // bit write value for search
				else	//collision here
				{
				// if this discrepancy is before the Last Discrepancy
				// on a previous next then pick the same as last time
					if(bitIndex < lastCollision)
						searchDirection = ((prevRom[byteIndex] & byteMask) > 0);
					else
						searchDirection = (bitIndex == lastCollision);
					if(!searchDirection)
					{
						lastCollision = bitIndex;
						lastDevice = false;
					}
				}

				if(searchDirection)
					rom[byteIndex] |= byteMask;
				else
					rom[byteIndex] &= ~byteMask;

				WriteBit(searchDirection);

				++bitIndex;
				byteMask <<= 1;

				if(!byteMask)
				{
					crc(rom[byteIndex]);  // accumulate the CRC
					++byteIndex;
					byteMask = 1;
				}
			} while(byteIndex < 8);  // loop until through all ROM bytes 0-7
		// if the search was successful then
			if(bitIndex == 65 && !crc.Get() && lastDevice)
			{
				return romIndex + 1;
			}
			prevRom = rom;
		}
		return romsMaxNumber;
	}
public:
	static void Init()
	{
		OwPin::Set();
		OwPin::template SetConfig<GpioBase::Out_OpenDrain_fast>();
	}
	static bool Reset()
	{
		OwPin::Clear();
		delay_us<480>();
		OwPin::Set();
		delay_us<70>();
		bool result = !OwPin::IsSet();
		delay_us<410>();
		return result;
	}
	static void Write(uint8_t val)
	{
		for(uint8_t i = 0; i < 8; ++i)
		{
			WriteBit(val & 0x01);
			val >>= 1;
		}
	}
	static void Write(const uint8_t* buf, uint16_t len)
	{
		for(uint16_t i = 0; i < len; ++i)
		{
			Write(buf[i]);
		}
	}
	static uint8_t Read()
	{
		uint8_t result = 0;
		for(uint8_t i = 0; i < 8; ++i)
		{
			result >>= 1;
			result |= ReadBit() << 7;
		}
		return result;
	}
	static uint8_t* Read(uint8_t* buf, uint16_t len)
	{
		for(uint16_t i = 0; i < len; ++i)
		{
			buf[i] = Read();
		}
		return buf;
	}
	static void Select(const uint8_t rom[8])
	{
		Write(CmdMatchRom);
		Write(rom, 8);
	}
	static void SkipRom()
	{
		Write(CmdSkipRom);
	}
	static void MatchRom(const uint8_t* rom)
	{
		Write(CmdMatchRom);
		Write(rom, 8);
	}
	 /*Input - Empty array[x][8] will be filled with roms of devices found, Capacity of array (x)
	 Output - number of roms found*/
	static uint8_t Search(uint8_t romArr[][8], const uint8_t romsMaxNumber)
	{
		return Search(romArr, romsMaxNumber, false);
	}

	/*Verify the device with the ROM number in rom buffer is present.
	 Return TRUE  : device verified present
			FALSE : device not present*/
	static bool Verify(const uint8_t* const romToCheck)
	{
		uint8_t rom[1][8];
		bool result = true;
		for(uint8_t i = 0; i < 8; ++i)
			rom[0][i] = romToCheck[i];
		if(Search(rom, 1, true))
		{
			for(uint8_t i = 0; i < 8; ++i)
				if(rom[0][i] != romToCheck[i])
					result = false;
		}
		return result;
	}

};
template<typename Ow, uint8_t devMaxNumber>
class Ds18b20
{
private:
	static uint8_t romArr[devMaxNumber][8];
	static uint8_t devNumber;
public:

};


}//Mcudrv


#endif // ONEWIRE_H

