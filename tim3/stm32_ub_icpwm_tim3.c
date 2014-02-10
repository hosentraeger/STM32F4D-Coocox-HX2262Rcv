//--------------------------------------------------------------
// File     : stm32_ub_icpwm_tim3.c
// Datum    : 20.07.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO, TIM, MISC
// Funktion : Input-Capture (PWM-Messung) per Timer3
//
// Hinweis  : mögliche Pinbelegungen
//            CH1 : [PA6, PB4, PC6]
//            CH2 : [PA7, PB5, PC7]
//
// Settings : Timer3-Frq steht auf 1MHz = 1us (siehe H-File)
//            Ein Messwert von 1     => 1us     = 1MHz
//            Ein Messwert von 10    => 10us    = 100kHz
//            Ein Messwert von 65535 => 65,53ms = 15 Hz
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_icpwm_tim3.h"

uint32_t data[MAXDATAGRAMS];
uint8_t bitnum = 0;
uint8_t datalen = 0;

//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_ICPWM_InitIO(void);
void P_ICPWM_InitTIM(void); 
void P_ICPWM_InitNVIC(void);


//--------------------------------------------------------------
// Globale Variabeln
//--------------------------------------------------------------
ICPWM_TIM3_Var_t ICPWM_TIM3_Var;


//--------------------------------------------------------------
// Definition vom InputCapture Pin (zur PWM-Messung)
//
// Channel : [1...2]
//--------------------------------------------------------------
ICPWM_TIM3_t ICPWM_TIM3 = {
		// Name       ,Channel, PORT , PIN       , CLOCK               , Source
		ICPWM_T3_PC6  ,1      ,GPIOC ,GPIO_Pin_6 ,RCC_AHB1Periph_GPIOC ,GPIO_PinSource6,
};



//--------------------------------------------------------------
// init und start vom InputCapture (PWM-Messung) mit Timer3
//-------------------------------------------------------------- 
void UB_ICPWM_TIM3_Init(void)
{
	// init aller Variabeln
	ICPWM_TIM3_Var.frq=0;
	ICPWM_TIM3_Var.duty=0;

	// init der Funktionen
	P_ICPWM_InitIO();
	P_ICPWM_InitTIM();
	P_ICPWM_InitNVIC();
}


//--------------------------------------------------------------
// auslesen vom Input-Capture Wert (Frq)
//
// Return_Wert : 0         => Messwert nicht gültig
//               1...65535 => Messwert
// Formeln :
//  Mess_Frq = T3_Frq / Messwert
//  Messwert = T3_Frq / Mess_Frq
//--------------------------------------------------------------
uint16_t UB_ICPWM_TIM3_ReadFRQ(void)
{
	uint16_t ret_wert=0;

	ret_wert=ICPWM_TIM3_Var.frq;
	ICPWM_TIM3_Var.frq=0;

	return(ret_wert);
}


//--------------------------------------------------------------
// auslesen vom Input-Capture Wert (DutyCycle)
//
// Return_Wert : 0         => Messwert nicht gültig
//               1...65535 => Messwert
// Formeln :
//  Impuls   = 1/T3_Frq * Messwert
//  Messwert = T3_Frq * Impuls
//--------------------------------------------------------------
uint16_t UB_ICPWM_TIM3_ReadDUTY(void)
{
	uint16_t ret_wert=0;

	ret_wert=ICPWM_TIM3_Var.duty;
	ICPWM_TIM3_Var.duty=0;

	return(ret_wert);
}


//--------------------------------------------------------------
// interne Funktion
// Init aller IO-Pins
//--------------------------------------------------------------
void P_ICPWM_InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Clock Enable
	RCC_AHB1PeriphClockCmd(ICPWM_TIM3.IC_CLK, ENABLE);

	// Config des Pins als AF-Input
	GPIO_InitStructure.GPIO_Pin = ICPWM_TIM3.IC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ICPWM_TIM3.IC_PORT, &GPIO_InitStructure);

	// Alternative-Funktion mit dem IO-Pin verbinden
	GPIO_PinAFConfig(ICPWM_TIM3.IC_PORT, ICPWM_TIM3.IC_SOURCE, GPIO_AF_TIM3);
}



//--------------------------------------------------------------
// interne Funktion
// Init vom Timer
//--------------------------------------------------------------
void P_ICPWM_InitTIM(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	// Clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Vorteiler einstellen
	TIM_PrescalerConfig(TIM3, ICPWM_TIM3_PRESCALE, TIM_PSCReloadMode_Immediate);

	if(ICPWM_TIM3.CHANNEL==1) {
		// Channel 1
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x08;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

		// input Trigger
		TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	}
	if(ICPWM_TIM3.CHANNEL==2) {
		// Channel 2
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

		// input Trigger
		TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
	}

	// Slave-Mode (Reset)
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

	// Timer enable
	TIM_Cmd(TIM3, ENABLE);
}


//--------------------------------------------------------------
// interne Funktion
// Init vom NVIC
//--------------------------------------------------------------
void P_ICPWM_InitNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// NVIC init
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	if(ICPWM_TIM3.CHANNEL==1) {
		// Channel 1
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	}
	if(ICPWM_TIM3.CHANNEL==2) {
		// Channel 2
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	}
}




//--------------------------------------------------------------
// ISR von Timer3
// wird bei einem Capture-Event aufgerufen
//--------------------------------------------------------------
void TIM3_IRQHandler(void)
{
	static uint32_t pulselen = 0;

	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
	{
		// Interrupt Flags loeschen
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		uint32_t period = TIM_GetCapture1 ( TIM3 );
		uint32_t duty   = TIM_GetCapture2 ( TIM3 );

		if ( datalen < MAXDATAGRAMS )
		{
			switch ( bitnum )
			{
			case 25:
				datalen++;
				bitnum = 0;
			case 0:
				pulselen = ( period - duty ) / 31;
				if ( ( pulselen > 300 ) && ( pulselen < 500 ) )
				{
					bitnum = 1;
					data[datalen] = 0;
				}
				else
				{
					pulselen = 0;
				}
				break;
			default:
				if      ( duty < 1 * pulselen * 8 / 10 ) bitnum = 0; 		// pulse too short
				else if ( duty < 1 * pulselen * 10 / 8 ) { data[datalen] <<= 1; bitnum++; } // pulse matches '0'
				else if ( duty < 3 * pulselen * 8 / 10 ) bitnum = 0; 	// pulse somewhere in the middle
				else if ( duty < 3 * pulselen * 10 / 8 ) { data[datalen] <<= 1; data[datalen] |= 1; bitnum++; } // pulse matches '1'
				else bitnum = 0; // pulse too long
				break;
			};
		};
	};

	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) {
		// Interrupt Flags loeschen
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		// aktuellen Frq-Wert auslesen
		ICPWM_TIM3_Var.frq = TIM_GetCapture2(TIM3);

		// aktuellen DutyCycle auslesen
		ICPWM_TIM3_Var.duty = TIM_GetCapture1(TIM3);
	};
};
