#pragma once

#define USE_CUSTOM_UART_IRQ
#include "uart.h"
#undef USE_CUSTOM_UART_IRQ
#include "flash.h"

#ifndef DEPIN
#define DEPIN Mcudrv::Nullpin
#endif

namespace Mcudrv
{
	namespace Wk
	{
		enum
		{
			DefaultADDR = 0x7F,
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
			C_SETADDRESS,
			C_SAVESETTINGS,
			C_GETUPTIME,
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
			ERR_NR,	//no replay
			ERR_NC,	//no carrier
			ERR_ADDRFMT,	//new address is wrong
			ERR_EEPROMUNLOCK, //EEPROM didn't unlock
			ERR_CNOTIMPL	//Command not impl
		};

		struct NullModule
		{
			static void Process() { }
		};

		template<typename Module1 = NullModule, typename Module2 = NullModule, typename Module3 = NullModule,
				 typename Module4 = NullModule, typename Module5 = NullModule, typename Module6 = NullModule>
		struct ModuleList
		{
			static void Process() {}
		};


		template<typename Uart>
		struct WakeTraits
		{
			static const bool isUart1 = false;
			enum{SingleWireOnlyForUART1 = 0};
		};

		template<>
		struct  WakeTraits<Uarts::Uart<UART1_BaseAddress> >
		{
			static const bool isUart1 = true;
			enum{SingleWireOnlyForUART1 = Uarts::SingleWireMode};
	 	};

		template<typename Uart, typename DEpin = DEPIN, uint8_t bufsize = 50 , Uarts::BaudRate baud = 9600UL, Mode mode = Slave>	//TODO: Master mode
		class Wake
		{
		private:
			static void Crc8(uint8_t b)
			{
				for(char i = 0; i < 8; b = b >> 1, i++)
					if((b ^ pdata.crc) & 1) pdata.crc = ((pdata.crc ^ 0x18) >> 1) | 0x80;
					else pdata.crc = (pdata.crc >> 1) & ~0x80;
			}
			#pragma data_alignment=4
			static uint8_t NVaddr  @ ".eeprom.noinit";
			static uint8_t addr;
			static uint8_t prev_byte;
			static State state;				//Current tranfer mode
			static uint8_t ptr;				//data pointer in Rx buffer
			
		public:
			static bool activity;			//Transaction activity flag
			static uint8_t cmd;
			struct Packet
			{
				uint8_t addr;
				uint8_t cmd;
				uint8_t dsize;
				uint8_t buf[bufsize];
				uint8_t crc;
			};
			static volatile Packet pdata;
			
			#pragma inline=forced
			static void Init()
			{
				using namespace Uarts;
				Uart::template Init<static_cast<Cfg>(Uarts::DefaultCfg | static_cast<Cfg>(WakeTraits<Uart>::SingleWireOnlyForUART1))>();		//Single Wire mode is default for UART1
				Uart::template EnableInterrupt<DefaultInts>();
				UartTraits<DEpin>::SetConfig();
				uint8_t tmp = NVaddr;
				if (tmp && tmp < 127)						//Address valid
					addr = tmp;
			}

			#pragma inline=forced
			static bool IsActive()
			{
				return activity;
			}

			static void SetAddress()
			{
				uint8_t taddr = pdata.buf[0];
				if (pdata.dsize == 2 && pdata.addr)
				{
					if (taddr == (~pdata.buf[1] & 0xFF) && taddr && taddr < 128)
					{
						using namespace Mem;
						Unlock<Eeprom>();
						if (IsUnlocked<Eeprom>())
						{
							NVaddr = addr = taddr;
							pdata.buf[0] = ERR_NO;
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
				pdata.dsize = 1;
			}

			static void Send()
			{
				using namespace Uarts;
				UartTraits<DEpin>::Set(); //переключение RS-485 на передачу
				char data_byte = FEND;
				pdata.crc = CRC_INIT;           //инициализация CRC,
				Crc8(data_byte);		//обновление CRC
				Uart::GetBaseAddr()->DR = data_byte;
				state = ADDR;
				prev_byte = TFEND;
				Uart::template EnableInterrupt<TxEmptyInt>();
				Uart::template DisableInterrupt<RxneInt>();
			}
			
			#pragma inline=forced
			static void TxIRQ()
			{
				using namespace Uarts;
				if (Uart::template IsEvent<TxComplete>())
				{
					Uart::template ClearEvent<TxComplete>();
					Uart::template ClearEvent<Rxne>();
					Uart::template EnableInterrupt<RxneInt>();
					UartTraits<DEpin>::Clear();		//переключение RS-485 на прием
					activity = false;
				}
				else //if (Uart::template IsEvent<Uarts::TxEmpty>())
				{
					char data_byte;
					if(prev_byte == FEND)               //если производится стаффинг,
					{
						data_byte = TFEND;                //передача TFEND вместо FEND
						prev_byte = data_byte;
						Uart::GetBaseAddr()->DR = data_byte;
						return;
					}
					if(prev_byte == FESC)               //если производится стаффинг,
					{
						data_byte = TFESC;                //передача TFESC вместо FESC
						prev_byte = data_byte;
						Uart::GetBaseAddr()->DR = data_byte;
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
								data_byte = addr; //то он передается (бит 7 равен единице)
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
							Uart::template DisableInterrupt<TxEmptyInt>();
						}
					}
					Crc8(data_byte);     //обновление CRC
					if (state == CMD)			//Метка адреса
						data_byte |= 0x80;
					prev_byte = data_byte;              //сохранение пре-байта
					if(data_byte == FEND)// || data_byte == FESC)
						data_byte = FESC;                 //передача FESC, если нужен стаффинг
					Uart::GetBaseAddr()->DR = data_byte;
				}
			}

			#pragma inline=forced
			static void RxIRQ()
			{
				using namespace Uarts;
				bool error = Uart::template IsEvent<static_cast<Events>(ParityErr | FrameErr | NoiseErr | OverrunErr)>(); //чтение флагов ошибок
				uint8_t data_byte = Uart::GetBaseAddr()->DR;              //чтение данных

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
							if(data_byte == 0 || data_byte == addr) //если нулевой или верный адрес,
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
						if(data_byte > bufsize)           //если количество байт > bufsize,
						{
							state = WAIT_FEND;
							cmd = C_ERR;//TODO:Флаг ошибки переполнения буфера            //то ошибка	
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

 		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		uint8_t Wake<Uart, DEpin, bufsize, baud, mode>::addr = DefaultADDR;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		uint8_t Wake<Uart, DEpin, bufsize, baud, mode>::NVaddr;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		uint8_t Wake<Uart, DEpin, bufsize, baud, mode>::cmd;// = CMD_NOP;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		uint8_t Wake<Uart, DEpin, bufsize, baud, mode>::prev_byte;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		State Wake<Uart, DEpin, bufsize, baud, mode>::state;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		uint8_t Wake<Uart, DEpin, bufsize, baud, mode>::ptr;

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		Wake<Uart, DEpin, bufsize, baud, mode>::Packet Wake<Uart, DEpin, bufsize, baud, mode>::pdata;// = Wake<bufsize, baud, mode>::Data();

		template<typename Uart, typename DEpin, uint8_t bufsize, Uarts::BaudRate baud, Mode mode>
		bool Wake<Uart, DEpin, bufsize, baud, mode>::activity;
	}

#ifndef USE_CUSTOM_UART_IRQ

#if defined (STM8S103) || defined (STM8S003)
	typedef Wk::Wake<Uarts::Uart<UART1_BaseAddress> > Wake1;
	INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
	{
		Wake1::TxIRQ();
	}

	INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
	{
		Wake1::RxIRQ();
	}

#endif

#if defined (STM8S105)
	typedef Wk::Wake<Uarts::Uart<UART2_BaseAddress> > Wake1;

	INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
	{
		Wake1::TxIRQ();
	}

	INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
	{
		Wake1::RxIRQ();
	}
#endif

#endif

}
