//--------------------------------------------------------------
// File     : stm32_ub_icpwm_tim3.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_ICPWM_TIM3_H
#define __STM32F4_UB_ICPWM_TIM3_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"


//--------------------------------------------------------------
// InputCapture Pin (PWM-Messung)
// der Name dient hier nur zur leichteren Lesbarkeit
//--------------------------------------------------------------
#define ICPWM_T3_PC6  0   // PWM-Messung per TIM3 an PC6



//--------------------------------------------------------------
// Input-Capture Einstellungen (Frequenz vom Timer3)
//
// Grundfrequenz = 2*APB1 (APB1=42MHz) => TIM_CLK=84MHz
// prescale  : 0 bis 0xFFFF
//
// T3_Frq = TIM_CLK/(vorteiler+1)
//--------------------------------------------------------------
#define  ICPWM_TIM3_PRESCALE  83     // prescaler => (83 => T3_Frq = 1 MHz)



//--------------------------------------------------------------
// Struktur vom InputCapture Kanals (zur PWM-Messung)
//--------------------------------------------------------------
typedef struct {
  const uint8_t IC_NAME;       // Name
  const uint8_t CHANNEL;       // Channel [1...2]
  GPIO_TypeDef* IC_PORT;       // Port
  const uint16_t IC_PIN;       // Pin
  const uint32_t IC_CLK;       // Clock
  const uint8_t IC_SOURCE;     // Source
}ICPWM_TIM3_t;


//--------------------------------------------------------------
// interne Struktur
//--------------------------------------------------------------
typedef struct {
  uint16_t frq;       // FRQ-Wert
  uint16_t duty;      // DutyCycle
}ICPWM_TIM3_Var_t;


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
void UB_ICPWM_TIM3_Init(void);
uint16_t UB_ICPWM_TIM3_ReadFRQ(void);
uint16_t UB_ICPWM_TIM3_ReadDUTY(void);

#define MAXDATAGRAMS 32
extern uint32_t data[MAXDATAGRAMS];
extern uint8_t bitnum;
extern uint8_t datalen;

//--------------------------------------------------------------
#endif // __STM32F4_UB_ICPWM_TIM3_H
