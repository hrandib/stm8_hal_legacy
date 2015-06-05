#pragma once

#include "timers.h"

#include "uart.h"
#include "flash.h"

#ifndef DEPIN
#define DEPIN Mcudrv::Nullpin
#endif

#define INSTRUCTION_SET_VERSION 2
#define WAKEDATABUFSIZE 64

namespace Mcudrv
{
	namespace Wk
	{

	//			---=== Operation time counter ===---
		class OpTime
		{
		private:
			struct EepromBuf_t
			{
				uint16_t Dummy1;
				uint8_t Dummy2;
				uint8_t lvalue;
			};

			#pragma data_alignment=4
			static EepromBuf_t eebuf[16] @ ".eeprom.noinit";
			#pragma data_alignment=4
			static uint16_t hvalue @ ".eeprom.noinit";
			volatile static bool tenMinPassed;
		public:
			#pragma inline=forced
			static void Init()
			{
				using namespace T2;
				Timer2::Init<Div32768, CEN>();
				Timer2::WriteAutoReload(36620UL);		//10 min period
				Timer2::TriggerEvent(UpdEv);			//Need to forcing prescaler settings
				Timer2::EnableInterrupt(UpdInt);		//
				__no_operation();						//Need to GenerateEvent<UpdEv>() take effect,
				Timer2::ClearIntFlag(UpdInt);
			}

			#pragma inline=forced
			static bool GetTenMinitesFlag()
			{
				return tenMinPassed;
			}
			#pragma inline=forced
			static void SetTenMinutesFlag()
			{
				tenMinPassed = true;
			}
			#pragma inline=forced
			static void ClearTenMinutesFlag()
			{
				tenMinPassed = false;
			}
			static uint8_t GetIndex()
			{
				uint8_t i;
				for (i = 0; i < 15; i++)
				{
					if (eebuf[i + 1].lvalue != eebuf[i].lvalue + 1) break;
				}
				return i;
			}
			static void Get(volatile uint8_t *arr)
			{
				uint16_t temp = hvalue;
				arr[0] = eebuf[GetIndex()].lvalue;
				arr[1] = temp & 0xFF;
				arr[2] = temp >> 8UL;
			}

			#pragma inline=forced
			static void CountInc()
			{
				using namespace Mem;
				uint8_t i = GetIndex();
				uint8_t tmp = eebuf[i].lvalue + 1;
				Unlock<Eeprom>();
				if (IsUnlocked<Eeprom>())
				{
					if (i != 15) eebuf[i + 1].lvalue = tmp;
					else eebuf[0].lvalue = tmp;
					if (tmp == 0) hvalue++;
				}
				Lock<Eeprom>();
			}
		};

		OpTime::EepromBuf_t OpTime::eebuf[16];
		uint16_t OpTime::hvalue;
		volatile bool OpTime::tenMinPassed;

		INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
		{
			T2::Timer2::ClearIntFlag(T2::UpdInt);
			OpTime::SetTenMinutesFlag();
		}

	//			---=== Wake main definitions ===---

		enum
		{
			DefaultADDR = 127,
			DefaultGroupADDR = 95,
			CRC_INIT = 0xDE,
			FEND = 0xC0,    //Frame END
			FESC = 0xDB,    //Frame ESCape
			TFEND = 0xDC,    //Transposed Frame END
			TFESC = 0xDD    //Transposed Frame ESCape
		};

		enum Mode
		{
			Master,
			Slave
		};

		enum State
		{
			WAIT_FEND = 0,     //ожидание приема FEND
			SEND_IDLE = 0,											//состояние бездействия
			ADDR,     //ожидание приема адреса						//передача адреса
			CMD,      //ожидание приема команды						//передача команды
			NBT,      //ожидание приема количества байт в пакете	//передача количества байт в пакете
			DATA,     //прием данных								//передача данных
			CRC,      //ожидание окончания приема CRC				//передача CRC
			CARR	   //ожидание несущей							//окончание передачи пакета
		};

		enum Cmd
		{
			C_NOP,    //нет операции
			C_ERR,    //ошибка приема пакета
			C_ECHO,    //передать эхо
			C_GETINFO,
			C_SETNODEADDRESS,
			C_SETGROUPADDRESS,
			C_SAVESETTINGS,
			C_GETOPTIME,
			C_OFF,
			C_ON
		};

		enum Err
		{
			ERR_NO,	//no error
			ERR_TX,	//Rx/Tx error
			ERR_BU,	//device busy error
			ERR_RE,	//device not ready error
			ERR_PA,	//parameters value error
			ERR_NI,	//Command not impl
			ERR_NR,	//no replay
			ERR_NC,	//no carrier
			ERR_ADDRFMT,	//new address is wrong
			ERR_EEPROMUNLOCK //EEPROM didn't unlock
		};

		enum DeviceType
		{
			devNull,
			devLedDriver = 0x01,
			devSwitch = 0x02,
			devRgbDriver = 0x04,
			devGenericIO = 0x08,
			devSensor = 0x10,
			devPowerSupply = 0x20,
//			Reserved = 0x40,
			devCustom = 0x80
		};

		enum AddrType
		{
			addrNode,
			addrGroup
		};

		struct NullModule
		{
			enum { deviceMask = devNull, features = 0 };
			static void Init() { }
			static void Process() { }
			static void SaveState() { }
			static void On() { }
			static void Off() { }
		};

		template<typename Module1, typename Module2 = NullModule, typename Module3 = NullModule,
				 typename Module4 = NullModule, typename Module5 = NullModule, typename Module6 = NullModule>
		struct ModuleList
		{
			enum { devicesMask = (uint8_t)Module1::deviceMask | Module2::deviceMask | Module3::deviceMask |
								Module4::deviceMask | Module5::deviceMask | Module6::deviceMask };
			static void Init()
			{
				Module1::Init();
				Module2::Init();
				Module3::Init();
				Module4::Init();
				Module5::Init();
				Module6::Init();
			}
			static void Process()
			{
				Module1::Process();
				Module2::Process();
				Module3::Process();
				Module4::Process();
				Module5::Process();
				Module6::Process();
			}
			static uint8_t GetDeviceFeatures(uint8_t deviceMask)
			{
				return deviceMask == Module1::deviceMask ? Module1::features :
						deviceMask == Module2::deviceMask ? Module2::features :
						deviceMask == Module3::deviceMask ? Module3::features :
						deviceMask == Module4::deviceMask ? Module4::features :
						deviceMask == Module5::deviceMask ? Module5::features :
						deviceMask == Module6::deviceMask ? Module6::features : 0;
			}
			static void SaveState()		//module should be save only if settings changed
			{
				Module1::SaveState();
				Module2::SaveState();
				Module3::SaveState();
				Module4::SaveState();
				Module5::SaveState();
				Module6::SaveState();
			}
			static void On()
			{
				Module1::On();
				Module2::On();
				Module3::On();
				Module4::On();
				Module5::On();
				Module6::On();
			}
			static void Off()
			{
				Module1::Off();
				Module2::Off();
				Module3::Off();
				Module4::Off();
				Module5::Off();
				Module6::Off();
			}
		};

		class WakeData
		{
		protected:
			struct Packet
			{
				uint8_t addr;
				uint8_t cmd;
				uint8_t dsize;
				uint8_t buf[WAKEDATABUFSIZE];
				uint8_t crc;
			};
			static volatile Packet pdata;
			static uint8_t cmd;
		};
		volatile WakeData::Packet WakeData::pdata;
		uint8_t WakeData::cmd;

		template<typename moduleList = ModuleList<NullModule>,
				 Uarts::BaudRate baud = 9600UL,
				 typename DEpin = DEPIN,
				 Mode mode = Slave>	//TODO: Master mode
		class Wake : WakeData
		{
		private:
			typedef Uarts::Uart<> Uart;
			typedef Uarts::Internal::UartTraits<DEpin> ControlPin;
			enum { SingleWireMode = Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0 };

			static void Crc8(uint8_t b) //TODO: Table computation
			{
				for(char i = 0; i < 8; b = b >> 1, i++)
					if((b ^ pdata.crc) & 1) pdata.crc = ((pdata.crc ^ 0x18) >> 1) | 0x80;
					else pdata.crc = (pdata.crc >> 1) & ~0x80;
			}
			#pragma location=".eeprom.data"
			static uint8_t nodeAddr_nv;// @ ".eeprom.data";
			#pragma location=".eeprom.data"
			static uint8_t groupAddr_nv;// @ ".eeprom.data";
//			static uint8_t addr;
//			static uint8_t groupaddr;
			static uint8_t prev_byte;
			static State state;				//Current tranfer mode
			static uint8_t ptr;				//data pointer in Rx buffer
			static bool activity;			//Transaction activity flag

			static void SetAddress(const AddrType nodeOrGroup)
			{
				if(pdata.dsize == 2 && pdata.addr)
				{
					if(nodeOrGroup ? CheckNodeAddress() : CheckGroupAddress())
					{
						using namespace Mem;
						uint8_t tempAddr = pdata.buf[0];
						Unlock<Eeprom>();
						if (IsUnlocked<Eeprom>())
						{
							if(nodeOrGroup) nodeAddr_nv = tempAddr;
							else groupAddr_nv = tempAddr;
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = tempAddr;
						}
						else pdata.buf[0] = ERR_EEPROMUNLOCK;
						Lock<Eeprom>();
					}
					else
						pdata.buf[0] = ERR_ADDRFMT;
				}
				else
				{
					pdata.buf[0] = ERR_PA;
				}
				if(pdata.buf[0]) pdata.buf[1] = 0;
			}
			#pragma inline=forced
			static bool CheckNodeAddress()
			{
				uint8_t taddr = pdata.buf[0];
				return taddr == (~pdata.buf[1] & 0xFF)
						&& ((taddr && taddr < 80) || (taddr > 111 && taddr < 128));
			}
			#pragma inline=forced
			static bool CheckGroupAddress()
			{
				uint8_t taddr = pdata.buf[0];
				return taddr == (~pdata.buf[1] & 0xFF)
						&& taddr > 79 && taddr < 96;
			}

		public:
			#pragma inline=forced
			static void Init()
			{
				using namespace Uarts;
		//Single Wire mode is default for UART1
				Uart::template Init<Cfg(Uarts::DefaultCfg | (Cfg)SingleWireMode)>();
				ControlPin::SetConfig();
				moduleList::Init();
				OpTime::Init();
				Wdg::Iwdg::Enable(Wdg::P_1s);
				Uart::EnableInterrupt(DefaultInts);
			}

			#pragma inline=forced
			static void Process()
			{
				if (OpTime::GetTenMinitesFlag() && !IsActive())
				{
					OpTime::ClearTenMinutesFlag();
					moduleList::SaveState();		//Save to EEPROM
					OpTime::CountInc();			//Refresh Uptime counter every 10 mins
				}
				Wdg::Iwdg::Refresh();
				switch(cmd)
				{
					case C_NOP:
						break;
					case C_ERR:
					{
						activity = false;
					}
						break;
					case C_ECHO:
						break;
					case C_GETINFO:
					{
						if (!pdata.dsize)	//Common device info
						{
							pdata.buf[0] = moduleList::devicesMask;
							pdata.buf[1] = INSTRUCTION_SET_VERSION;
						}
						else if (pdata.dsize == 1)	//Info for each logical device
						{
							if(pdata.buf[0] < 7)
							{
								uint8_t deviceMask = 1 << pdata.buf[0];
								if(moduleList::devicesMask & deviceMask) //device available
								{
									pdata.buf[0] = ERR_NO;
									pdata.buf[1] = moduleList::GetDeviceFeatures(deviceMask);
								}
								else //device not available
								{
									pdata.buf[0] = ERR_PA;
									pdata.dsize = 1;
									break;
								}
							}
							//else if(pdata.buf[0] == 7) //custom device
						}
						else
						{
							pdata.buf[0] = ERR_PA;
							pdata.dsize = 1;
							break;
						}
						pdata.dsize = 2;
					}
						break;
					case C_SETNODEADDRESS: SetAddress(addrNode);
						break;
					case C_SETGROUPADDRESS: SetAddress(addrGroup);
						break;
					case C_SAVESETTINGS:
					{
						if(!pdata.dsize)
						{
							moduleList::SaveState();
							pdata.buf[0] = ERR_NO;
						}
						else pdata.buf[0] = ERR_PA;
						pdata.dsize = 1;
					}
						break;
					case C_GETOPTIME:
					{
						if (!pdata.dsize)
						{
							pdata.buf[0] = Wk::ERR_NO;
							OpTime::Get(&pdata.buf[1]);
							pdata.dsize = 4;
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
							pdata.dsize = 1;
						}
					}
						break;
					case C_OFF:
					{
						if (!pdata.dsize)
						{
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::Off();
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.dsize = 1;
					}
						break;
					case C_ON:
					{
						if (!pdata.dsize)
						{
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::On();
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.dsize = 1;
					}
						break;
					default: moduleList::Process();
				}
				if (pdata.addr == nodeAddr_nv && cmd != C_NOP) Send();
				cmd = Wk::C_NOP;
			}

			#pragma inline=forced
			static bool IsActive()
			{
				return activity;
			}

			static void Send()
			{
				using namespace Uarts;
				ControlPin::Set(); //переключение RS-485 на передачу
				char data_byte = FEND;
				pdata.crc = CRC_INIT;           //инициализация CRC,
				Crc8(data_byte);		//обновление CRC
				Uart::Regs()->DR = data_byte;
				state = ADDR;
				prev_byte = TFEND;
				Uart::EnableInterrupt(TxEmptyInt);
				Uart::DisableInterrupt(RxneInt);
			}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma("vector=17")
#elif defined (STM8S105)
			_Pragma("vector=20")
#endif
			__interrupt static void TxIRQ()
			{
				using namespace Uarts;
				if (Uart::IsEvent(TxComplete))
				{
					Uart::ClearEvent(TxComplete);
					Uart::ClearEvent(Rxne);
					Uart::EnableInterrupt(RxneInt);
					ControlPin::Clear();		//переключение RS-485 на прием
					activity = false;
				}
				else //if (Uart::template IsEvent<Uarts::TxEmpty>())
				{
					char data_byte;
					if(prev_byte == FEND)               //если производится стаффинг,
					{
						data_byte = TFEND;                //передача TFEND вместо FEND
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					if(prev_byte == FESC)               //если производится стаффинг,
					{
						data_byte = TFESC;                //передача TFESC вместо FESC
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					switch(state)
					{
					case ADDR:                     //-----> передача адреса	
									//TODO: Implement differences master to slave
						{
							state = CMD;
							if(pdata.addr)                   //если адрес не равен нулю,
							{
								data_byte = nodeAddr_nv; //то он передается (бит 7 равен единице)
								break;
							}
								//иначе сразу передаем команду
						}
					case CMD:                      //-----> передача команды
						{
							data_byte = pdata.cmd & 0x7F;
							state = NBT;
							break;
						}
					case NBT:                      //-----> передача количества байт
						{
							data_byte = pdata.dsize;
							state = DATA;
							ptr = 0;                  //обнуление указателя данных для передачи
							break;
						}
					case DATA:                     //-----> передача данных
						{
							if(ptr < pdata.dsize)
								data_byte = pdata.buf[ptr++];
							else
							{
								data_byte = pdata.crc;        //передача CRC
								state = CRC;
							}
							break;
						}
					default:
						{
							state = SEND_IDLE;          //передача пакета завершена
							Uart::DisableInterrupt(TxEmptyInt);
						}
					}
					Crc8(data_byte);     //обновление CRC
					if (state == CMD)			//Метка адреса
						data_byte |= 0x80;
					prev_byte = data_byte;              //сохранение пре-байта
					if(data_byte == FEND)// || data_byte == FESC)
						data_byte = FESC;                 //передача FESC, если нужен стаффинг
					Uart::Regs()->DR = data_byte;
				}
			}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma("vector=18")
#elif defined (STM8S105)
			_Pragma("vector=21")
#endif
			__interrupt static void RxIRQ()
			{
				using namespace Uarts;
				bool error = Uart::IsEvent(static_cast<Events>(ParityErr | FrameErr | NoiseErr | OverrunErr)); //чтение флагов ошибок
				uint8_t data_byte = Uart::Regs()->DR;              //чтение данных

				if(error)     //если обнаружены ошибки при приеме байта
				{	
					state = WAIT_FEND;            //ожидание нового пакета
					cmd = C_ERR;                //рапортуем об ошибке
					return;
				}

				if(data_byte == FEND)               //если обнаружено начало фрейма,
				{
					prev_byte = data_byte;          //то сохранение пре-байта,
					pdata.crc = CRC_INIT;           //инициализация CRC,
					state = ADDR;					//сброс указателя данных,
					Crc8(data_byte);	//обновление CRC,
					activity = true;
					return;                         //выход
				}

				if(state == WAIT_FEND)          //-----> если ожидание FEND,
				{
					return;				//то выход
				}

				char Pre = prev_byte;               //сохранение старого пре-байта
				prev_byte = data_byte;              //обновление пре-байта
				
				if(Pre == FESC)                     //если пре-байт равен FESC,
				{
					if(data_byte == TFESC)            //а байт данных равен TFESC,
						data_byte = FESC;               //то заменить его на FESC
					else if(data_byte == TFEND)       //если байт данных равен TFEND,
						data_byte = FEND;          //то заменить его на FEND
					else
					{
						state = WAIT_FEND;     //для всех других значений байта данных,
						cmd = C_ERR;         //следующего за FESC, ошибка
						return;
					}
				}
				else
				{
					if(data_byte == FESC)             //если байт данных равен FESC, он просто
						return;                         //запоминается в пре-байте
				}

				switch(state)
				{
				case ADDR:                     //-----> ожидание приема адреса
					{
						if(data_byte & 0x80)            //если бит 7 данных не равен нулю, то это адрес
						{
							data_byte = data_byte & 0x7F; //обнуляем бит 7, получаем истинный адрес
							if(data_byte == 0 || data_byte == nodeAddr_nv || data_byte == groupAddr_nv) //если нулевой или верный адрес,
							{
								Crc8(data_byte); //то обновление CRC и
								pdata.addr = data_byte;
								state = CMD;       //переходим к приему команды
								break;
							}
							state = WAIT_FEND;        //адрес не совпал, ожидание нового пакета
							break;
						}
						else pdata.addr = 0;	//если бит 7 данных равен нулю, то
						state = CMD;					//сразу переходим к приему команды
					}
				case CMD:                      //-----> ожидание приема команды
					{
						if(data_byte & 0x80)            //проверка бита 7 данных
						{
							state = WAIT_FEND;        //если бит 7 не равен нулю,
							cmd = C_ERR;            //то ошибка
							break;
						}
						pdata.cmd = data_byte;          //сохранение команды
						Crc8(data_byte); //обновление CRC
						state = NBT;           //переходим к приему количества байт
						break;
					}
				case NBT:                      //-----> ожидание приема количества байт
					{
						if(data_byte >= WAKEDATABUFSIZE)           //если количество байт > bufsize,
						{
							state = WAIT_FEND;
							cmd = C_ERR;//TODO:Флаг ошибки переполнения буфера  //то ошибка
							break;
						}
						pdata.dsize = data_byte;
						Crc8(data_byte); //обновление CRC
						ptr = 0;                  //обнуляем указатель данных
						state = DATA;          //переходим к приему данных
						break;
					}
				case DATA:                     //-----> ожидание приема данных
					{
						if(ptr < pdata.dsize)       //если не все данные приняты,
						{
							pdata.buf[ptr++] = data_byte; //то сохранение байта данных,
							Crc8(data_byte);  //обновление CRC
							break;
						}
						if(data_byte != pdata.crc)      //если приняты все данные, то проверка CRC
						{
							state = WAIT_FEND;        //если CRC не совпадает,
							cmd = C_ERR;//TODO:Флаг ошибки CRC           //то ошибка
							break;
						}
						state = WAIT_FEND;          //прием пакета завершен,
						cmd = pdata.cmd;            //загрузка команды на выполнение
						break;
					}
				}
			}
		};

		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		uint8_t Wake<moduleList, baud, DEpin, mode>::nodeAddr_nv = 127;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		uint8_t Wake<moduleList, baud, DEpin, mode>::groupAddr_nv = 95;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		uint8_t Wake<moduleList, baud, DEpin, mode>::prev_byte;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		State Wake<moduleList, baud, DEpin, mode>::state;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		uint8_t Wake<moduleList, baud, DEpin, mode>::ptr;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		bool Wake<moduleList, baud, DEpin, mode>::activity;
	}


}
