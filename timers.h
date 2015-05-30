#pragma once
#include "stm8s.h"
#include "static_assert.h"

namespace Mcudrv
{
	namespace T1
	{
		typedef uint16_t Div;
//TODO: Implement TIM1_SR2
		enum Cfg
		{
//			---=== TIM1_CR1 ===---
			Default = 0,
			CEN = TIM1_CR1_CEN,			// Counter enable
			UDIS = TIM1_CR1_UDIS,		// A UEV is not generated, shadow registers keep their value (ARR, PSC).
										// The counter and the prescaler are re-initialized if the UG bit is set.
			URS = TIM1_CR1_URS,		// An update interrupt request is sent only when the counter reaches the
									// overflow/underflow.
			OPM = TIM1_CR1_OPM,		// One Pulse Mode. Counter stops counting at the next update event (clearing the CEN bit)
			DIR = TIM1_CR1_DIR,		//Direction 0 - Up, 1 - Down
			ARPE = TIM1_CR1_ARPE,	// TIM4_ARR register is buffered through a preload register	

//			---================---			

//		Counter must be in Stop Mode CEN = 0

			EdgeAlign = 0,				// The counter counts up or down depending on the direction bit (DIR)		

//		Encoder mode must be disable (SMS = 0) in center aligned mode	

			CenterAlign1 = 0x01 << 5u,	// The counter counts up and down alternately. Output compare interrupt 
										// flags of channels configured in output (CCiS = 00 in TIM1_CCMRiregisters) are set only when the 
										// counter is counting down.
			CenterAlign2 = 0x02 << 5u,	// The counter counts up and down alternately. Output compare interrupt 
										// flags of channels configured in output (CCiS = 00 in CCMRiregisters) are set only when the counter 
										// is counting up. 
			CenterAlign3 = 0x03 << 5u,	// The counter counts up and down alternately. Output compare interrupt 
										// flags of channels configured in output (CCiS = 00 in TIM1_CCMRiregisters) are set both when the 
										// counter is counting up and down
			
//			---=== TIM1_CR2 ===---

		// Master mode selection
			TrigReset = 0,				// The UG bit from the TIM1_EGR register is used as trigger output (TRGO). If the reset is 
										// generated by the trigger input (clock/trigger mode controller configured in reset mode), the signal on 
										// TRGO is delayed compared to the actual reset.
			TrigEnable = 0x01 << 12u,	// The counter enable signal is used as trigger output (TRGO). It is used to start several 
										// timers or the ADC to control a window in which a slave timer or the ADC is enabled. The counter 
										// enable signal is generated by a logic OR between the CEN control bit and the trigger input when 
										// configured in trigger gated mode. When the counter enable signal is controlled by the trigger input, 
										// there is a delay on TRGO, except if the master/slave mode is selected (see the MSM bit description 
										// in TIM1_SMCR register).
			TrigUpdateEv = 0x02 << 12u,	// The update event is selected as trigger output (TRGO)
			TrigMatch = 0x03 << 12u,		// The trigger output sends a positive pulse when the CC1IF flag is to 
										// be set (even if it was already high), as soon as a capture or a compare match occurs (TRGO).
			TrigComp1 = 0x04 << 12u,		// OC1REF signal is used as trigger output (TRGO)
			TrigComp2 = 0x05 << 12u,		// OC2REF signal is used as trigger output (TRGO)
			TrigComp3 = 0x06 << 12u,		// OC3REF signal is used as trigger output (TRGO)
			TrigComp4 = 0x07 << 12u,		// OC4REF signal is used as trigger output (TRGO)
//			---================---			
			COMS = TIM1_CR2_COMS << 10u,		// 0: When capture/compare control bits are preloaded (CCPC = 1), they are updated by setting the COMG bit. 
										// 1: When capture/compare control bits are preloaded (CCPC = 1), they are updated by setting the 
										// COMG bit or when an rising edge occurs on TRGI.
//		This bit acts only on channels with complementary outputs
			CCPC = TIM1_CR2_CCPC << 8u,		// 0: The CCiE, CCiNE, CCiP, and CCiNP bits in the TIM1_CCERiregisters and the OCiM bit in the TIM1_CCMRiregisters are not preloaded
										// 1: CCiE, CCiNE, CCiP, C CiNP and OCiM bits are preloaded, after having been written, they are 
										// updated only when COMG bit is set in the TIM1_EGR register.
		};

		enum SlaveModeCtrl
		{
//			Default = 0,
			MSM = TIM1_SMCR_MSM,
			//TODO: implement
		};

		enum Ints
		{
			UpdInt = TIM1_IER_UIE,
			Ch1Int = TIM1_IER_CC1IE,
			Ch2Int = TIM1_IER_CC2IE,
			Ch3Int = TIM1_IER_CC3IE,
			Ch4Int = TIM1_IER_CC4IE,
			ComInt = TIM1_IER_COMIE,
			TrigInt = TIM1_IER_TIE,			
			BreakInt = TIM1_IER_BIE
		};
		
		enum Events
		{
			UpdEv = TIM1_EGR_UG,
			Ch1Ev = TIM1_EGR_CC1G,
			Ch2Ev = TIM1_EGR_CC2G,
			Ch3Ev = TIM1_EGR_CC3G,
			TrigEv = TIM1_EGR_TG
		};
		
		enum Channel
		{
			Ch1,
			Ch2,
			Ch3,
			Ch4,
			All_Ch
		};

		enum ActiveLevel
		{
			ActiveHigh,
			ActiveLow
		};

		enum ChannelType
		{
			Output,
			Input,
			InputMap0 = Input,  //ICx is mapped on TI1FPx
			InputMap1,			//ICx is mapped on TI2FPx
			InputMap2			//ICx is mapped on TRC
		};

		enum ChannelCfgIn
		{
			In_NoFilter,
			In_Filt_n2 = 0x01 << 4u,			// 0001: fSAMPLING= fMASTER, N = 2
			In_Filt_n4 = 0x02 << 4u,			// 0010: fSAMPLING= fMASTER, N = 4
			In_Filt_n8 = 0x03 << 4u,			// 0011: fSAMPLING= fMASTER, N = 8 
			In_Filt_div2_n8 = 0x05 << 4u,		// 0101: fSAMPLING= fMASTER/2, N = 8
			In_Filt_div4_n8 = 0x07 << 4u,		// 0111: fSAMPLING= fMASTER/4, N = 8
			In_Filt_div8_n8 = 0x09 << 4u,		// 1001: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div16_n8 = 0x0C << 4u,		// 1100: fSAMPLING= fMASTER/16, N = 8
			In_Filt_div32_n8 = 0x0F << 4u,		// 1111: fSAMPLING= fMASTER/32, N = 8
			//			---================---			
			In_NoPresc = 0,						// 00: no prescaler, capture is done each time an edge is detected on the capture input
			In_Presc_2ev = 0x01 << 2u,			// 01: Capture is done once every 2 events
			In_Presc_4ev = 0x02 << 2u,			// 10: Capture is done once every 4 events
			In_Presc_8ev = 0x03 << 2u,			// 11: Capture is done once every 8 events
		};

		enum ChannelCfgOut
		{
			OutCompareClear = 0x80,				// OCxCE This bit is used to enable the clearing of the channel x output compare signal (OCxREF) by an 
												// external event on the TIM1_ETR pin (see Section 17.5.9 on page 181).
												// 	0: OC1REF is not affected by the ETRF input signal (derived from the TIM1_ETR pin)
												// 	1: OC1REF is cleared as soon as a high level is detected on ETRF input signal (derived from the 
												// 	TIM1_ETR pin).
//			---================---			
			Out_Frozen = 0,						// 000: Frozen - The comparison between the output compare register TIMx_CCRx and the counter 
			Out_ActiveOnMatch = 0x01 << 4u,		// 001: Set channel x to active level on match. OC1REF signal is forced high when the counter 
												// TIMx_CNT matches the capture/compare register x (TIMx_CCRx).
			Out_InactiveOnMatch = 0x02 << 4u,	// 010: Set channel x to inactive level on match. OC1REF signal is forced low when the counter 
												// TIMx_CNT matches the capture/compare register x (TIMx_CCRx).
			Out_ToggleOnMatch = 0x03 << 4u,		// 011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCRx
			Out_ForceInactive = 0x04 << 4u,		// 100: Force inactive level - OCxREF is forced low
			Out_ForceActive = 0x05 << 4u,		// 101: Force active level - OCxREF is forced high
			Out_PWM_Mode1 = 0x06 << 4u,			// 110: PWM mode 1 - In up-counting, channel x is active as long as TIMx_CNT< TIMx_CCRx. 
												// Otherwise, channel x is inactive. In down-counting,channel x is inactive (OCxREF = 0) as long as 
												// TIMx_CNT> TIMx_CCRx. Otherwise, channel x is active (OCxREF = 1).
			Out_PWM_Mode2 = 0x07 << 4u,			// 111: PWM mode 2 - In up-counting, channel x is inactive as long as TIMx_CNT< TIMx_CCRx. Otherwise, channel x is active
			//		---================---			
			Out_PreloadEnable = TIM1_CCMR_OCxPE,	// Preload register on TIMx_CCRx enabled. Read/write operations access the preload register. 
			//		---================---			
			Out_FastEnable = TIM1_CCMR_OCxFE		// This bit is used to accelerate the effect of an event on the trigger in input on the CC output.
													//	0: CC1 behaves normally depending on the counter and CCR1 values, even when the trigger is on. 
													//	The minimum delay to activate CC1 output when an edge occurs on the trigger input, is 5 clock 
													//	cycles.
													//	1: An active edge on the trigger input acts like a compare match on the CC1 output. If this happens, 
													//	OC is set to the compare level irrespective of the result of the comparison. The delay to sample the 
													//	trigger input and to activate CC1 output is reduced to 3 clock cycles. OCFE acts only if the channel 
													//	is configured in PWM1 or PWM2 mode. 
		};

		class Timer1
		{
		public:
			template <uint16_t divider, Cfg config = Default>
			static void Init()
			{
				TIM1->PSCRH = (divider - 1) >> 8UL;
				TIM1->PSCRL = divider - 1;
				TIM1->CR1 = config;
				TIM1->CR2 = config >> 8u;
				TIM1->BKR |= TIM1_BKR_MOE;
			}

			#pragma inline=forced
			static void Enable()
			{
				TIM1->CR1 |= TIM1_CR1_CEN;
			}
			#pragma inline=forced
			static void Disable()
			{
				TIM1->CR1 &= ~TIM1_CR1_CEN;
			}

			#pragma inline=forced
			template <Ints mask>
			static void EnableInterrupt()
			{
				TIM1->IER |= mask;
			}

			#pragma inline=forced
			template <uint16_t mask>
			static void DisableInterrupt()
			{
				TIM1->IER &= ~mask;
			}

			#pragma inline=forced
			template <Ints flag>
			static bool CheckIntStatus()
			{
				return TIM1->SR1 & flag;
			}

			#pragma inline=forced
			template <Ints flag>
			static void ClearIntFlag()
			{
				TIM1->SR1 &= ~flag;
			}

			#pragma inline=forced
			template <Events ev>
			static void GenerateEvent()
			{
				TIM1->EGR |= ev;
			}
			
			#pragma inline=forced
			static void WriteCounter(uint16_t c)	//Need to stop Timer 
			{
	//			Disable();
				TIM1->CNTRH = c >> 8;
				TIM1->CNTRL = c;
	//			Enable();
			}

			#pragma diag_suppress=Pe940
			static uint16_t ReadCounter()
			{
				__asm("LD A, L:0x525e\n"
						"LD XH, A\n"
						"LD A, L:0x525f\n"
						"LD XL, A\n");
			}
			#pragma diag_default=Pe940
			
			#pragma inline=forced
			static void WriteAutoReload(uint16_t c)
			{
				TIM1->ARRH = c >> 8;
				TIM1->ARRL = c;
			}

			#pragma inline=forced
			template <Channel Ch, ChannelType type, ChannelCfgIn cfg>
			static void SetChannelCfg()
			{
				BOOST_STATIC_ASSERT(type != Output);
				if(Ch == All_Ch)
				{
					TIM1->CCMR1 = TIM1->CCMR2 = TIM1->CCMR3 = TIM1->CCMR4 = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
				}
				else *reinterpret_cast<volatile uint8_t*>(&TIM1->CCMR1 + Ch) = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
			}

			#pragma inline=forced
			template <Channel Ch, ChannelType type, ChannelCfgOut cfg>
			static void SetChannelCfg()
			{
				BOOST_STATIC_ASSERT(type == Output);
				if(Ch == All_Ch)
				{
					TIM1->CCMR1 = TIM1->CCMR2 = TIM1->CCMR3 = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
				}
				else *reinterpret_cast<volatile uint8_t*>(&TIM1->CCMR1 + Ch) = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
			}
			
			#pragma inline=forced
			template <Channel Ch, ActiveLevel level = ActiveHigh>
			static void ChannelEnable()
			{
				if(Ch == All_Ch)
				{
					TIM1->CCER2 |= TIM1_CCER2_CC3E | (level << 1) | TIM1_CCER2_CC4E | ((level << 1) << 4);
					TIM1->CCER1 |= TIM1_CCER1_CC1E | (level << 1) | TIM1_CCER1_CC2E | ((level << 1) << 4);
				}
				if (Ch == Ch4) TIM1->CCER2 |= TIM1_CCER2_CC4E | ((level << 1) << 4);
				if (Ch == Ch3) TIM1->CCER2 |= TIM1_CCER2_CC3E | (level << 1);
				if (Ch == Ch2) TIM1->CCER1 |= TIM1_CCER1_CC2E | ((level << 1) << 4);
				if (Ch == Ch1) TIM1->CCER1 |= TIM1_CCER1_CC1E | (level << 1);
			}

			#pragma inline=forced
			template <Channel Ch>
			static void ChannelDisable()
			{
				BOOST_STATIC_ASSERT(Ch != All_Ch);
				if (Ch & Ch4 == Ch4) TIM1->CCER2 &= ~(0x0F << 4);
				if (Ch & Ch3 == Ch3) TIM1->CCER2 &= ~0x0F;
				if (Ch & Ch2 == Ch2) TIM1->CCER1 &= ~(0x0F << 4);
				if (Ch & Ch1 == Ch1) TIM1->CCER1 &= ~0x0F;
			}
			
			#pragma inline=forced
			template <Channel Ch, ActiveLevel level = ActiveHigh>
			static void ChannelEnableComplementary()
			{
				BOOST_STATIC_ASSERT(Ch == Ch4);
				if (Ch & Ch3 == Ch3) TIM1->CCER2 |= TIM1_CCER2_CC3NE | (level << 3);
				if (Ch & Ch2 == Ch2) TIM1->CCER1 |= TIM1_CCER1_CC2NE | ((level << 3) << 4);
				if (Ch & Ch1 == Ch1) TIM1->CCER1 |= TIM1_CCER1_CC1NE | (level << 3);
			}
 		
			#pragma inline=forced
			template<Channel Ch>
			static void WriteCompare(uint16_t c)
			{
				*reinterpret_cast<volatile uint8_t*>(&TIM1->CCR1H + Ch * 2) = c >> 8;
				*reinterpret_cast<volatile uint8_t*>(&TIM1->CCR1L + Ch * 2) = c;
			}
			#pragma inline=forced
			template<Channel Ch>
			static void WriteCompareLSB(uint8_t c)
			{
				*reinterpret_cast<volatile uint8_t*>(&TIM1->CCR1L + Ch * 2) = c;
			}
			#pragma inline=forced
			template<Channel Ch>
			static uint16_t ReadCompare()
			{
				return *reinterpret_cast<volatile uint16_t*>(&TIM1->CCR1H + Ch * 2);
			}
			#pragma inline=forced
			template<Channel Ch>
			static uint8_t ReadCompareLSB()
			{
				return *reinterpret_cast<volatile uint8_t*>(&TIM1->CCR1L + Ch * 2);
			}

		};

	}

	namespace T2
	{
		enum Div
			{
				Div1,
				Div2,
				Div4,
				Div8,
				Div16,
				Div32,
				Div64,
				Div128,
				Div256,
				Div512,
				Div1024,
				Div2048,
				Div4096,
				Div8192,
				Div16384,
				Div32768,
			};

		enum Cfg
			{
				Default = 0,
				CEN = TIM2_CR1_CEN,			// Counter enable
				UDIS = TIM2_CR1_UDIS,		// A UEV is not generated, shadow registers keep their value (ARR, PSC).
											// The counter and the prescaler are re-initialized if the UG bit is set.
				URS = TIM2_CR1_URS,		// An update interrupt request is sent only when the counter reaches the
										// overflow/underflow.
				GPbit = TIM2_CR1_OPM,	// General purpose use
				ARPE = TIM2_CR1_ARPE   // TIM2_ARR register is buffered through a preload register
			};

		enum Ints
			{
				UpdInt = TIM2_IER_UIE,
				Ch1Int = TIM2_IER_CC1IE,
				Ch2Int = TIM2_IER_CC2IE,
				Ch3Int = TIM2_IER_CC3IE
			};
		enum Events
		{
			UpdEv = TIM2_EGR_UG,
			Ch1Ev = TIM2_EGR_CC1G,
			Ch2Ev = TIM2_EGR_CC2G,
			Ch3Ev = TIM2_EGR_CC3G
		};

		enum Channel
			{
				Ch1,
				Ch2,
				Ch3,
				All_Ch
			};

		enum ChannelType
		{
			Output,
			Input,
			InputMap0 = Input,  //ICx is mapped on TI1FPx
			InputMap1,  //ICx is mapped on TI2FPx
		};

		enum ActiveLevel
			{
				ActiveHigh,
				ActiveLow
			};
		
		enum ChannelCfgIn
		{
			In_NoFilter,
			In_Filt_n2 = 0x01 << 4u,			// 0001: fSAMPLING= fMASTER, N = 2
			In_Filt_n4 = 0x02 << 4u,			// 0010: fSAMPLING= fMASTER, N = 4
			In_Filt_n8 = 0x03 << 4u,			// 0011: fSAMPLING= fMASTER, N = 8 
			In_Filt_div2_n8 = 0x05 << 4u,		// 0101: fSAMPLING= fMASTER/2, N = 8
			In_Filt_div4_n8 = 0x07 << 4u,		// 0111: fSAMPLING= fMASTER/4, N = 8
			In_Filt_div8_n8 = 0x09 << 4u,		// 1001: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div16_n8 = 0x0C << 4u,		// 1100: fSAMPLING= fMASTER/16, N = 8
			In_Filt_div32_n8 = 0x0F << 4u,		// 1111: fSAMPLING= fMASTER/32, N = 8
//			---================---			
			In_NoPresc = 0,						// 00: no prescaler, capture is done each time an edge is detected on the capture input
			In_Presc_2ev = 0x01 << 2u,			// 01: Capture is done once every 2 events
			In_Presc_4ev = 0x02 << 2u,			// 10: Capture is done once every 4 events
			In_Presc_8ev = 0x03 << 2u,			// 11: Capture is done once every 8 events
		};

		enum ChannelCfgOut
		{
			Out_Frozen = 0,						// 000: Frozen - The comparison between the output compare register TIMx_CCRx and the counter 
			Out_ActiveOnMatch = 0x01 << 4u,		// 001: Set channel x to active level on match. OC1REF signal is forced high when the counter 
												// TIMx_CNT matches the capture/compare register x (TIMx_CCRx).
			Out_InactiveOnMatch = 0x02 << 4u,	// 010: Set channel x to inactive level on match. OC1REF signal is forced low when the counter 
												// TIMx_CNT matches the capture/compare register x (TIMx_CCRx).
			Out_ToggleOnMatch = 0x03 << 4u,		// 011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCRx
			Out_ForceInactive = 0x04 << 4u,		// 100: Force inactive level - OCxREF is forced low
			Out_ForceActive = 0x05 << 4u,		// 101: Force active level - OCxREF is forced high
			Out_PWM_Mode1 = 0x06 << 4u,			// 110: PWM mode 1 - In up-counting, channel x is active as long as TIMx_CNT< TIMx_CCRx. 
												// Otherwise, channel x is inactive. In down-counting,channel x is inactive (OCxREF = 0) as long as 
												// TIMx_CNT> TIMx_CCRx. Otherwise, channel x is active (OCxREF = 1).
			Out_PWM_Mode2 = 0x07 << 4u,			// 111: PWM mode 2 - In up-counting, channel x is inactive as long as TIMx_CNT< TIMx_CCRx. Otherwise, channel x is active
		//		---================---			
			Out_PreloadEnable = 0x01 << 3u,			// Preload register on TIMx_CCRx enabled. Read/write operations access the preload register. 
														// TIMx_CCRx preload value is loaded in the shadow register at each update event.
		};
	
		class Timer2
		{
		private:
		public:
			template <Div divider, Cfg config>
			static void Init()
			{
				TIM2->PSCR = divider;
				TIM2->CR1 = config;
			}
		
			#pragma inline=forced
			static void Enable()
			{
				TIM2->CR1 |= TIM4_CR1_CEN;
			}
			#pragma inline=forced
			static void Disable()
			{
				TIM2->CR1 &= ~TIM4_CR1_CEN;
			}

			#pragma inline=forced
			template <uint16_t mask>
			static void EnableInterrupt()
			{
				TIM2->IER |= mask;
 			}

			#pragma inline=forced
			template <uint16_t mask>
			static void DisableInterrupt()
			{
				TIM2->IER &= ~mask;
			}
			
			#pragma inline=forced
			template <Events ev>
			static void GenerateEvent()
			{
				TIM2->EGR |= ev;
			}

			#pragma inline=forced
			template <Ints flag>
			static bool CheckIntStatus()
			{
				return TIM2->SR1 & flag;
			}
		
			#pragma inline=forced
			template <Ints flag>
			static void ClearIntFlag()
			{
				TIM2->SR1 &= ~flag;
			}
		
			#pragma inline=forced
			static void WriteCounter(uint16_t c)	//Need to stop Timer 
			{
	//			Disable();
				TIM2->CNTRH = c >> 8;
				TIM2->CNTRL = c;
	//			Enable();
			}
		
			#pragma diag_suppress=Pe940
			static uint16_t ReadCounter()
			{
				__asm("LD A, L:0x530a\n"
					"LD XH, A\n"
					"LD A, L:0x530b\n"
					"LD XL, A\n");
			}
			#pragma diag_default=Pe940

			#pragma inline=forced
			static void WriteAutoReload(uint16_t c)
			{
				TIM2->ARRH = c >> 8;
				TIM2->ARRL = c;
			}

			#pragma inline=forced
			template <Channel Ch, ChannelType type, ChannelCfgIn cfg>
			static void SetChannelCfg()
			{
				BOOST_STATIC_ASSERT(type != Output);
				if(Ch == All_Ch)
				{
					TIM2->CCMR1 = TIM2->CCMR2 = TIM2->CCMR3 = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
				}
				else *reinterpret_cast<volatile uint8_t*>(&TIM2->CCMR1 + Ch) = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
			}
				
			#pragma inline=forced
			template <Channel Ch, ChannelType type, ChannelCfgOut cfg>
			static void SetChannelCfg()
			{
				BOOST_STATIC_ASSERT(type == Output);
				if(Ch == All_Ch)
				{
					TIM2->CCMR1 = TIM2->CCMR2 = TIM2->CCMR3 = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
				}
				else *reinterpret_cast<volatile uint8_t*>(&TIM2->CCMR1 + Ch) = static_cast<uint8_t>(type) | static_cast<uint8_t>(cfg);
			}
			
			#pragma inline=forced
			template <Channel Ch, ActiveLevel level = ActiveHigh>
			static void ChannelEnable()
			{	
				if(Ch == All_Ch)
				{
					TIM2->CCER2 |= TIM2_CCER2_CC3E | (level << 1);
					TIM2->CCER1 |= TIM2_CCER1_CC1E | (level << 1) | TIM2_CCER1_CC2E | ((level << 1) << 4);
				}
				if (Ch == Ch3) TIM2->CCER2 |= TIM2_CCER2_CC3E | (level << 1);
				if (Ch == Ch2) TIM2->CCER1 |= TIM2_CCER1_CC2E | ((level << 1) << 4);
				if (Ch == Ch1) TIM2->CCER1 |= TIM2_CCER1_CC1E | (level << 1);
			}

			#pragma inline=forced
			template <Channel Ch>
			static void ChannelDisable()
			{
//				BOOST_STATIC_ASSERT(Ch != All_Ch);
				if(Ch == All_Ch)
				{
					TIM2->CCER2 = TIM2->CCER1 = 0;
				}
				if (Ch == Ch3) TIM2->CCER2 &= ~0x0F;
				if (Ch == Ch2) TIM2->CCER1 &= ~(0x0F << 4);
				if (Ch == Ch1) TIM2->CCER1 &= ~0x0F;
			}

			#pragma inline=forced
			template <Channel Ch>
			static void WriteCompare(uint16_t cmp)
			{
				reinterpret_cast<volatile uint16_t*>(&TIM2->CCR1H)[Ch] = cmp;
			}

			#pragma inline=forced
			template <Channel Ch>
			static void WriteCompare(uint8_t cmp)
			{
				reinterpret_cast<volatile uint8_t*>(&TIM2->CCR1L)[Ch*2] = cmp;
			}

			#pragma inline=forced
			template <Channel Ch>
			static volatile uint8_t& ReadCompare()
			{
				return reinterpret_cast<volatile uint8_t*>(&TIM2->CCR1L)[Ch*2];
			}


		};
	}

	namespace T4
	{
			enum Div
			{
				Div1,
				Div2,
				Div4,
				Div8,
				Div16,
				Div32,
				Div64,
				Div128
			};

			enum Cfg
			{
				CEN = TIM4_CR1_CEN,		// CEN Counter enable
				UDIS = TIM4_CR1_UDIS,	// UDIS  A UEV is not generated, shadow registers keep their value (ARR, PSC).
										//The counter and the	prescaler are re-initialized if the UG bit is set.
				URS = TIM4_CR1_URS,		// URS an update interrupt request is sent only when the counter reaches the
										//overflow/underflow.
				OPM = TIM4_CR1_OPM,
				ARPE = TIM4_CR1_ARPE	//ARPE TIM4_ARR register is buffered through a preload register
			};

		class Timer4
		{
		public:
			template <Div Divider, Cfg Config>
			static void Init()
			{
				TIM4->PSCR = Divider;
				TIM4->CR1 = Config;
			}
			static void Enable()
			{
				TIM4->CR1 |= TIM4_CR1_CEN;
			}
			static void Disable()
			{
				TIM4->CR1 &= ~TIM4_CR1_CEN;
			}
			static void Clear()
			{
				TIM4->CNTR = 0;
			}
			static void EnableInterrupt() // Only one interrupt on that vector (Update)
			{
				TIM4->IER |= TIM4_IER_UIE;
			}
			static void DisableInterrupt()
			{
				TIM4->IER &= ~TIM4_IER_UIE;
			}
			static bool CheckIntStatus()		
			{
				return TIM4->SR1 & TIM4_SR1_UIF;
			}
			static void ClearIntFlag()
			{
				TIM4->SR1 &= ~TIM4_SR1_UIF;
			}
			static void WriteCounter(uint8_t c)
			{
				TIM4->CNTR = c;
			}
			#pragma inline=forced
			static uint8_t ReadCounter()
			{
				return TIM4->CNTR;
			}
			static void WriteAutoReload(uint8_t c)
			{
				TIM4->ARR = c;
			}
		};
	}

	namespace Wdg
	{
		enum Period
		{
			P_16ms,
			P_32ms,
			P_64ms,
			P_128ms,
			P_256ms,
			P_512ms,
			P_1s
		};
		
		class Iwdg
		{
		public:
			#pragma inline=forced
			template<Period period>
			static void Enable()
			{
				IWDG->KR = 0xCC;
				IWDG->KR = 0x55;
				IWDG->RLR = 0xFF;
				IWDG->PR = period;
			}
			#pragma inline=forced
			static void Refresh()
			{
				IWDG->KR = 0xAA;
			}
		};
	}
}
