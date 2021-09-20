/************************************************************************************//**
* \file         servo.c
* \brief        Servo driver source file.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2019 (c)  by HAN Automotive   http://www.han.nl        All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* \endinternal
****************************************************************************************/

/*
 * f_TIM = 1 MHz
 *
 *       CNT
 *        |
 *        |
 *    ARR |
 *   19999+-----------------------------------------------------------.--------
 *        |                                                        .  .
 *        |                                                     .     .
 *        |                                                  .        .
 *        |                                               .           .
 *        |                                            .              .
 *        |                                         .                 .
 *        |                                      .                    .
 *        |                                   .                       .
 *        |                                .                          .
 *        |                             .                             .
 *        |                          .                                .
 *        |                       .                                   .
 *        |                    .                                      .
 *        |                 .                                         .
 *        |              .                                            .
 *    2000+ - - - - - .                                               .
 *        |        .  |                                               .
 *    1000+- - -.- - - - - - - - - - - - - - - - - - - - - - - - - - -.- - -.-
 *        |  .  |     |                                               .  .  |
 *       0.-----+-----------------------------------------------------.--------
 *
 *        +-----+-----+                                               +-----+
 *        |     |     |                                               |     |
 *        |     |     |                                               |     |
 * OC pin +     +-----+-----------------------------------------------+     +--
 *
 *        |<--->| min_pulse_width_in_us, typically 1000 us
 *
 *        |<--------->| max_pulse_width_in_us, typically 2000 us
 *
 *        |<------------------------- 20 ms ------------------------->|
 */

/****************************************************************************************
* Include files
****************************************************************************************/
#include "servo.h"
#include "stdio.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */

/****************************************************************************************
* Macro definitions
****************************************************************************************/

/****************************************************************************************
* Type definitions
****************************************************************************************/
//#define ERROR_CHECK_ENABLED

typedef struct
{
	TIM_TypeDef *tim;
	uint8_t channel;
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t peripheral_clk;
	uint8_t gpio_af;
    uint8_t pin_source;
	uint32_t rcc_tim_clk;
} tpinMapping;

/****************************************************************************************
* External function prototypes
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/

/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a servo module. */
const static tpinMapping pinMapping[] =
{
	
	{TIM1, 0, GPIOA, GPIO_Pin_8,  RCC_AHBPeriph_GPIOA, GPIO_AF_6, GPIO_PinSource8, RCC_APB2Periph_TIM1},
	{TIM1, 1, GPIOA, GPIO_Pin_9,  RCC_AHBPeriph_GPIOA, GPIO_AF_6, GPIO_PinSource9, RCC_APB2Periph_TIM1},
	{TIM1, 2, GPIOA, GPIO_Pin_10, RCC_AHBPeriph_GPIOA, GPIO_AF_6,  GPIO_PinSource10,RCC_APB2Periph_TIM1},  // Changed, alternate function = 6: Orignal code: {TIM1, 2, GPIOA, GPIO_Pin_10, RCC_AHBPeriph_GPIOA, GPIO_AF_12, GPIO_PinSource10,RCC_APB2Periph_TIM1},
	{TIM2, 0, GPIOA, GPIO_Pin_0,  RCC_AHBPeriph_GPIOA, GPIO_AF_1, GPIO_PinSource0, RCC_APB1Periph_TIM2},
	{TIM2, 1, GPIOA, GPIO_Pin_1,  RCC_AHBPeriph_GPIOA, GPIO_AF_1, GPIO_PinSource1, RCC_APB1Periph_TIM2},
	{TIM2, 3, GPIOA, GPIO_Pin_3,  RCC_AHBPeriph_GPIOA, GPIO_AF_1, GPIO_PinSource3, RCC_APB1Periph_TIM2},
	{TIM3, 0, GPIOC, GPIO_Pin_6,  RCC_AHBPeriph_GPIOC, GPIO_AF_2, GPIO_PinSource6, RCC_APB1Periph_TIM3},
	{TIM3, 1, GPIOC, GPIO_Pin_7,  RCC_AHBPeriph_GPIOC, GPIO_AF_2, GPIO_PinSource7, RCC_APB1Periph_TIM3},
	{TIM3, 2, GPIOC, GPIO_Pin_8,  RCC_AHBPeriph_GPIOC, GPIO_AF_2, GPIO_PinSource8, RCC_APB1Periph_TIM3},
	{TIM4, 0, GPIOB, GPIO_Pin_6,  RCC_AHBPeriph_GPIOB, GPIO_AF_2, GPIO_PinSource6, RCC_APB1Periph_TIM4},
	{TIM4, 1, GPIOB, GPIO_Pin_7,  RCC_AHBPeriph_GPIOB, GPIO_AF_2, GPIO_PinSource7, RCC_APB1Periph_TIM4},
	{TIM4, 2, GPIOB, GPIO_Pin_8,  RCC_AHBPeriph_GPIOB, GPIO_AF_2, GPIO_PinSource8, RCC_APB1Periph_TIM4},
	{TIM4, 3, GPIOB, GPIO_Pin_9,  RCC_AHBPeriph_GPIOB, GPIO_AF_2, GPIO_PinSource9, RCC_APB1Periph_TIM4}
};

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array with all minimum pulse widths for each servo module. */
static uint16_t m_min_pulse_width_in_us[13] =			// Changed (arraysize was 12)
{
	1000,1000,1000,1000,
	1000,1000,1000,1000,
	1000,1000,1000,1000, 1000							// Changed (added one element to array)
};

/** \brief Array with all maximum pulse widths for each servo module. */
static uint16_t m_max_pulse_width_in_us[13] =			// Changed (arraysize was 12)
{
	2000,2000,2000,2000,
	2000,2000,2000,2000,
	2000,2000,2000,2000, 2000							// Changed (added one element to array)
};

/************************************************************************************//**
** \brief     Initializes a servo module.
** \param     pin_id Pin identifier.
** \param     min_pulse_width_in_us Minimum pulse width in microseconds.
** \param     max_pulse_width_in_us Maximum pulse width in microseconds.
** \return    none.
**
****************************************************************************************/
void ServoInit(const uint8_t pin_id, const uint16_t min_pulse_width_in_us, const uint16_t max_pulse_width_in_us)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

    // make sure the id is valid before using it as an array indexer
    if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
      ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
   
	// Save the parameters to global variables
	m_min_pulse_width_in_us[pin_id] = min_pulse_width_in_us;
	m_max_pulse_width_in_us[pin_id] = max_pulse_width_in_us;

	/* Turn on the timer clocks */
	if (pinMapping[pin_id].rcc_tim_clk == RCC_APB2Periph_TIM1)
	{
	RCC_APB2PeriphClockCmd(pinMapping[pin_id].rcc_tim_clk, ENABLE);
	}
	else
	{
	RCC_APB1PeriphClockCmd(pinMapping[pin_id].rcc_tim_clk, ENABLE);
	}

   /* Enable the port's peripheral clock */
    RCC_AHBPeriphClockCmd(pinMapping[pin_id].peripheral_clk, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = pinMapping[pin_id].pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(pinMapping[pin_id].port, &GPIO_InitStructure);

    GPIO_PinAFConfig(pinMapping[pin_id].port, pinMapping[pin_id].pin_source, pinMapping[pin_id].gpio_af);
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(pinMapping[pin_id].tim, &TIM_TimeBaseStructure);


	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = m_min_pulse_width_in_us[pin_id];
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if(pinMapping[pin_id].channel == 0)
	{
		TIM_OC1Init(pinMapping[pin_id].tim, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(pinMapping[pin_id].tim, TIM_OCPreload_Enable);
	}
	else if(pinMapping[pin_id].channel == 1)
	{
		TIM_OC2Init(pinMapping[pin_id].tim, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(pinMapping[pin_id].tim, TIM_OCPreload_Enable);
	}
	else if(pinMapping[pin_id].channel == 2)
	{
		TIM_OC3Init(pinMapping[pin_id].tim, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(pinMapping[pin_id].tim, TIM_OCPreload_Enable);
	}
	else if(pinMapping[pin_id].channel == 3)
	{
		TIM_OC4Init(pinMapping[pin_id].tim, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(pinMapping[pin_id].tim, TIM_OCPreload_Enable);
	}

  TIM_Cmd(pinMapping[pin_id].tim, ENABLE);
  TIM_CtrlPWMOutputs(pinMapping[pin_id].tim, ENABLE);
}

/************************************************************************************//**
** \brief     Updates the pulse width.
** \param     pin_id Pin identifier.
** \param     pulse_width_in_us New pulse width in microsecond.
** \return    none.
**
****************************************************************************************/
void ServoUpdate(const uint8_t pin_id, uint16_t pulse_width_in_us)
{
    // make sure the id is valid before using it as an array indexer
    if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
      ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
	// Verify input parameter
	if(pulse_width_in_us < m_min_pulse_width_in_us[pin_id])
	{
		pulse_width_in_us = m_min_pulse_width_in_us[pin_id];
	}

	if(pulse_width_in_us > m_max_pulse_width_in_us[pin_id])
	{
		pulse_width_in_us = m_max_pulse_width_in_us[pin_id];
	}
	
	if(pinMapping[pin_id].channel == 0)
	{
	pinMapping[pin_id].tim->CCR1 = pulse_width_in_us;
	}
	else if(pinMapping[pin_id].channel == 1)
	{
	pinMapping[pin_id].tim->CCR2 = pulse_width_in_us;
	}
	else if(pinMapping[pin_id].channel == 2)
	{
	pinMapping[pin_id].tim->CCR3 = pulse_width_in_us;
	}
	else if(pinMapping[pin_id].channel == 3)
	{
	pinMapping[pin_id].tim->CCR4 = pulse_width_in_us;		// Changed (CCR3 by CCR4)
	}

}

/********************************* end of servo.c *************************************/