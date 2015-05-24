#pragma once

#define USE_CUSTOM_UART_IRQ
#include "uart.h"
#undef USE_CUSTOM_UART_IRQ
#include "flash.h"

#ifndef DEPIN
#define DEPIN Nullpin
#endif

#ifdef __VS12
#define __interrupt
#define __enable_interrupt()
#define __weak
#define __no_init
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
			WAIT_FEND = 0,     //�������� ������ FEND
			SEND_IDLE = 0,											//��������� �����������
			ADDR,     //�������� ������ ������						//�������� ������
			CMD,      //�������� ������ �������						//�������� �������
			NBT,      //�������� ������ ���������� ���� � ������	//�������� ���������� ���� � ������
			DATA,     //����� ������								//�������� ������
			CRC,      //�������� ��������� ������ CRC				//�������� CRC
			CARR	   //�������� �������							//��������� �������� ������
		};

		enum Cmd
		{
			C_NOP,    //��� ��������
			C_ERR,    //������ ������ ������
			C_ECHO,    //�������� ���
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

		template<typename Uart, typename DEpin = DEPIN, uint8_t bufsize = 50/*sizeof(DEVICEINFO)*/, Uarts::BaudRate baud = 9600UL, Mode mode = Slave>	//TODO: Master mode
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
			static uint8_t NVaddr @ ".eeprom.noinit";
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
				UartTraits<DEpin>::Set(); //������������ RS-485 �� ��������
				char data_byte = FEND;
				pdata.crc = CRC_INIT;           //������������� CRC,
				Crc8(data_byte);		//���������� CRC
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
					UartTraits<DEpin>::Clear();		//������������ RS-485 �� �����
					activity = false;
				}
				else //if (Uart::template IsEvent<Uarts::TxEmpty>())
				{
					char data_byte;
					if(prev_byte == FEND)               //���� ������������ ��������,
					{
						data_byte = TFEND;                //�������� TFEND ������ FEND
						prev_byte = data_byte;
						Uart::GetBaseAddr()->DR = data_byte;
						return;
					}
					if(prev_byte == FESC)               //���� ������������ ��������,
					{
						data_byte = TFESC;                //�������� TFESC ������ FESC
						prev_byte = data_byte;
						Uart::GetBaseAddr()->DR = data_byte;
						return;
					}
					switch(state)
					{
					case ADDR:                     //-----> �������� ������	
									//TODO: Implement differences master to slave
						{
							state = CMD;
							if(pdata.addr)                   //���� ����� �� ����� ����,
							{
								data_byte = addr; //�� �� ���������� (��� 7 ����� �������)
								break;
							}
								//����� ����� �������� �������
						}
					case CMD:                      //-----> �������� �������
						{
							data_byte = pdata.cmd & 0x7F;
							state = NBT;
							break;
						}
					case NBT:                      //-----> �������� ���������� ����
						{
							data_byte = pdata.dsize;
							state = DATA;
							ptr = 0;                  //��������� ��������� ������ ��� ��������
							break;
						}
					case DATA:                     //-----> �������� ������
						{
							if(ptr < pdata.dsize)
								data_byte = pdata.buf[ptr++];
							else
							{
								data_byte = pdata.crc;        //�������� CRC
								state = CRC;
							}
							break;
						}
					default:
						{
							state = SEND_IDLE;          //�������� ������ ���������
							Uart::template DisableInterrupt<TxEmptyInt>();
						}
					}
					Crc8(data_byte);     //���������� CRC
					if (state == CMD)			//����� ������
						data_byte |= 0x80;
					prev_byte = data_byte;              //���������� ���-�����
					if(data_byte == FEND)// || data_byte == FESC)
						data_byte = FESC;                 //�������� FESC, ���� ����� ��������
					Uart::GetBaseAddr()->DR = data_byte;
				}
			}

			#pragma inline=forced
			static void RxIRQ()
			{
				using namespace Uarts;
				bool error = Uart::template IsEvent<static_cast<Events>(ParityErr | FrameErr | NoiseErr | OverrunErr)>(); //������ ������ ������
				uint8_t data_byte = Uart::GetBaseAddr()->DR;              //������ ������

				if(error)     //���� ���������� ������ ��� ������ �����
				{	
					state = WAIT_FEND;            //�������� ������ ������
					cmd = C_ERR;                //��������� �� ������
					return;
				}

				if(data_byte == FEND)               //���� ���������� ������ ������,
				{
					prev_byte = data_byte;          //�� ���������� ���-�����,
					pdata.crc = CRC_INIT;           //������������� CRC,
					state = ADDR;					//����� ��������� ������,
					Crc8(data_byte);	//���������� CRC,
					activity = true;
					return;                         //�����
				}

				if(state == WAIT_FEND)          //-----> ���� �������� FEND,
				{
					return;				//�� �����
				}

				char Pre = prev_byte;               //���������� ������� ���-�����
				prev_byte = data_byte;              //���������� ���-�����
				
				if(Pre == FESC)                     //���� ���-���� ����� FESC,
				{
					if(data_byte == TFESC)            //� ���� ������ ����� TFESC,
						data_byte = FESC;               //�� �������� ��� �� FESC
					else if(data_byte == TFEND)       //���� ���� ������ ����� TFEND,
						data_byte = FEND;          //�� �������� ��� �� FEND
					else
					{
						state = WAIT_FEND;     //��� ���� ������ �������� ����� ������,
						cmd = C_ERR;         //���������� �� FESC, ������
						return;
					}
				}
				else
				{
					if(data_byte == FESC)             //���� ���� ������ ����� FESC, �� ������
						return;                         //������������ � ���-�����
				}

				switch(state)
				{
				case ADDR:                     //-----> �������� ������ ������
					{
						if(data_byte & 0x80)            //���� ��� 7 ������ �� ����� ����, �� ��� �����
						{
							data_byte = data_byte & 0x7F; //�������� ��� 7, �������� �������� �����
							if(data_byte == 0 || data_byte == addr) //���� ������� ��� ������ �����,
							{
								Crc8(data_byte); //�� ���������� CRC �
								pdata.addr = data_byte;
								state = CMD;       //��������� � ������ �������
								break;
							}
							state = WAIT_FEND;        //����� �� ������, �������� ������ ������
							break;
						}
						else pdata.addr = 0;	//���� ��� 7 ������ ����� ����, ��
						state = CMD;					//����� ��������� � ������ �������
					}
				case CMD:                      //-----> �������� ������ �������
					{
						if(data_byte & 0x80)            //�������� ���� 7 ������
						{
							state = WAIT_FEND;        //���� ��� 7 �� ����� ����,
							cmd = C_ERR;            //�� ������
							break;
						}
						pdata.cmd = data_byte;          //���������� �������
						Crc8(data_byte); //���������� CRC
						state = NBT;           //��������� � ������ ���������� ����
						break;
					}
				case NBT:                      //-----> �������� ������ ���������� ����
					{
						if(data_byte > bufsize)           //���� ���������� ���� > bufsize,
						{
							state = WAIT_FEND;
							cmd = C_ERR;//TODO:���� ������ ������������ ������            //�� ������	
							break;
						}
						pdata.dsize = data_byte;
						Crc8(data_byte); //���������� CRC
						ptr = 0;                  //�������� ��������� ������
						state = DATA;          //��������� � ������ ������
						break;
					}
				case DATA:                     //-----> �������� ������ ������
					{
						if(ptr < pdata.dsize)       //���� �� ��� ������ �������,
						{
							pdata.buf[ptr++] = data_byte; //�� ���������� ����� ������,
							Crc8(data_byte);  //���������� CRC
							break;
						}
						if(data_byte != pdata.crc)      //���� ������� ��� ������, �� �������� CRC
						{
							state = WAIT_FEND;        //���� CRC �� ���������,
							cmd = C_ERR;//TODO:���� ������ CRC           //�� ������
							break;
						}
						state = WAIT_FEND;          //����� ������ ��������,
						cmd = pdata.cmd;            //�������� ������� �� ����������
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