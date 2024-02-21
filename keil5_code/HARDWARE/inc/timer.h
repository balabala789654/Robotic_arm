#ifndef _TIMER_H
#define _TIMER_H

#include "sys.h"

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM5_PWM_Init(u32 arr,u32 psc);
void TIM1_Int_Init(u16 arr,u16 psc);

void TIM1_UP_TIM10_IRQHandler(void);
void TIM3_IRQHandler(void);

#endif
