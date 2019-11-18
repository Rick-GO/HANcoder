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
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */

/****************************************************************************************
* Macro definitions
****************************************************************************************/

/****************************************************************************************
* Type definitions
****************************************************************************************/
/**< \brief Structure type with module mapping information. */
typedef struct
{
  TIM_TypeDef * timer_peripheral;
  uint32_t timer_peripheral_clk;
  void (*rcc_apb_periph_clock_cmd_tim)(uint32_t, FunctionalState);
  uint8_t timer_prescale_divider;
} tServoModuleMapping;

/**< \brief Structure type with pin mapping information. */
typedef struct
{
  uint8_t peripheral_clk;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t module_idx;
  void (*tim_oc_init)(TIM_TypeDef*, TIM_OCInitTypeDef*);
  void (*tim_oc_preload_config)(TIM_TypeDef*, uint16_t);
  uint8_t gpio_af;
  uint16_t pin_source;
  void (*tim_set_compare)(TIM_TypeDef*, uint32_t);
} tServoPinMapping;

/****************************************************************************************
* External function prototypes
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/

/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a servo timer module. note that
 *         TIM2-7 and TIM12-14 have a max input clock frequency of 84 Mhz as opposed
 *         to 168 MHz so these modules need an extra prescale divider.
 */
const static tServoModuleMapping moduleMapping[] =
{
  { TIM1,  RCC_APB2Periph_TIM1,  RCC_APB2PeriphClockCmd, 1 }, /* idx 0: TIM1  */
  { TIM2,  RCC_APB1Periph_TIM2,  RCC_APB1PeriphClockCmd, 2 }, /* idx 1: TIM2  */
  { TIM3,  RCC_APB1Periph_TIM3,  RCC_APB1PeriphClockCmd, 2 }, /* idx 2: TIM3  */
  { TIM4,  RCC_APB1Periph_TIM4,  RCC_APB1PeriphClockCmd, 2 }, /* idx 3: TIM4  */
  { TIM8,  RCC_APB2Periph_TIM8,  RCC_APB2PeriphClockCmd, 1 }, /* idx 4: TIM8  */
  { TIM9,  RCC_APB2Periph_TIM9,  RCC_APB2PeriphClockCmd, 1 }, /* idx 5: TIM9  */
  { TIM10, RCC_APB2Periph_TIM10, RCC_APB2PeriphClockCmd, 1 }, /* idx 6: TIM10 */
  { TIM11, RCC_APB2Periph_TIM11, RCC_APB2PeriphClockCmd, 1 }, /* idx 7: TIM11 */
  { TIM13, RCC_APB1Periph_TIM13, RCC_APB1PeriphClockCmd, 2 }, /* idx 8: TIM13 */
  { TIM14, RCC_APB1Periph_TIM14, RCC_APB1PeriphClockCmd, 2 }  /* idx 9: TIM14 */
};

/** \brief Array with all configuration parameters of a servo output pin. */
const static tServoPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9,  0, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM1,  GPIO_PinSource9 , TIM_SetCompare1}, /* idx 0:  SERVO_TIM1_PIN_PE9   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11, 0, TIM_OC2Init, TIM_OC2PreloadConfig, GPIO_AF_TIM1,  GPIO_PinSource11, TIM_SetCompare2}, /* idx 1:  SERVO_TIM1_PIN_PE11  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_13, 0, TIM_OC3Init, TIM_OC3PreloadConfig, GPIO_AF_TIM1,  GPIO_PinSource13, TIM_SetCompare3}, /* idx 2:  SERVO_TIM1_PIN_PE13  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_14, 0, TIM_OC4Init, TIM_OC4PreloadConfig, GPIO_AF_TIM1,  GPIO_PinSource14, TIM_SetCompare4}, /* idx 3:  SERVO_TIM1_PIN_PE14  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5,  1, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM2,  GPIO_PinSource5 , TIM_SetCompare1}, /* idx 4:  SERVO_TIM2_PIN_PA5   */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6,  2, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM3,  GPIO_PinSource6 , TIM_SetCompare1}, /* idx 5:  SERVO_TIM3_PIN_PA6   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5,  2, TIM_OC2Init, TIM_OC2PreloadConfig, GPIO_AF_TIM3,  GPIO_PinSource5 , TIM_SetCompare2}, /* idx 6:  SERVO_TIM3_PIN_PB5   */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12, 3, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM4,  GPIO_PinSource12, TIM_SetCompare1}, /* idx 7:  SERVO_TIM4_PIN_PD12  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13, 3, TIM_OC2Init, TIM_OC2PreloadConfig, GPIO_AF_TIM4,  GPIO_PinSource13, TIM_SetCompare2}, /* idx 8:  SERVO_TIM4_PIN_PD13  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  4, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM8,  GPIO_PinSource6 , TIM_SetCompare1}, /* idx 9:  SERVO_TIM8_PIN_PC6   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  4, TIM_OC2Init, TIM_OC2PreloadConfig, GPIO_AF_TIM8,  GPIO_PinSource7 , TIM_SetCompare2}, /* idx 10: SERVO_TIM8_PIN_PC7   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5,  5, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM9,  GPIO_PinSource5 , TIM_SetCompare1}, /* idx 11: SERVO_TIM9_PIN_PE5   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6,  5, TIM_OC2Init, TIM_OC2PreloadConfig, GPIO_AF_TIM9,  GPIO_PinSource6 , TIM_SetCompare2}, /* idx 12: SERVO_TIM9_PIN_PE6   */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_6,  6, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM10, GPIO_PinSource6 , TIM_SetCompare1}, /* idx 13: SERVO_TIM10_PIN_PF6  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_7,  7, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM11, GPIO_PinSource7 , TIM_SetCompare1}, /* idx 14: SERVO_TIM11_PIN_PF7  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_8,  8, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM13, GPIO_PinSource8 , TIM_SetCompare1}, /* idx 15: SERVO_TIM13_PIN_PF8  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_9,  9, TIM_OC1Init, TIM_OC1PreloadConfig, GPIO_AF_TIM14, GPIO_PinSource9 , TIM_SetCompare1}  /* idx 16: SERVO_TIM14_PIN_PF9  */
};

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array with all minimum pulse widths for each servo module. */
static uint16_t m_min_pulse_width_in_us[17] =
{
	1000,1000,1000,1000,
	1000,1000,1000,1000,
	1000,1000,1000,1000,
	1000,1000,1000,1000,
    1000
};

/** \brief Array with all maximum pulse widths for each servo module. */
static uint16_t m_max_pulse_width_in_us[17] =
{
	2000,2000,2000,2000,
	2000,2000,2000,2000,
	2000,2000,2000,2000,
	2000,2000,2000,2000,
    2000
};

/************************************************************************************//**
** \brief     Initializes a servo module.
** \param     pinID Pin identifier.
** \param     min_pulse_width_in_us Minimum pulse width in microseconds.
** \param     max_pulse_width_in_us Maximum pulse width in microseconds.
** \return    none.
**
****************************************************************************************/
void ServoInit(const uint8_t pinID, const uint16_t min_pulse_width_in_us, const uint16_t max_pulse_width_in_us)
{
    GPIO_InitTypeDef  gpio_init;
    TIM_OCInitTypeDef tim_ocinit;
    uint16_t prescaler;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_BDTRInitTypeDef bdtr;

	// Save the parameters to global variables
	m_min_pulse_width_in_us[pinID] = min_pulse_width_in_us;
	m_max_pulse_width_in_us[pinID] = max_pulse_width_in_us;
    
	// Temporary local variables
	register uint32_t m_idx = pinMapping[pinID].module_idx;

    /* make sure the id is valid before using it as an array indexer */
    if(!(pinID < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
        ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }

    /* enable the port's peripheral clock */
    RCC_AHB1PeriphClockCmd(pinMapping[pinID].peripheral_clk, ENABLE);
    
    /* prepare pin configuration */
    gpio_init.GPIO_Pin = pinMapping[pinID].pin;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* initialize the pin */
    GPIO_Init(pinMapping[pinID].port, &gpio_init);    
    
    /* connect the pin to it's timer module alternate function */
    GPIO_PinAFConfig(pinMapping[pinID].port, pinMapping[pinID].pin_source, pinMapping[pinID].gpio_af);
    
    /* turn on the timer clock */
    moduleMapping[m_idx].rcc_apb_periph_clock_cmd_tim(moduleMapping[m_idx].timer_peripheral_clk, ENABLE);
        
    /* configure the timer output channel as PWM */
    TIM_OCStructInit(&tim_ocinit);
    tim_ocinit.TIM_OCMode = TIM_OCMode_PWM1;
    tim_ocinit.TIM_OutputState = TIM_OutputState_Enable;
    tim_ocinit.TIM_OutputNState = TIM_OutputNState_Disable;
    tim_ocinit.TIM_Pulse = 0;
    tim_ocinit.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_ocinit.TIM_OCIdleState = TIM_OCIdleState_Reset;
    tim_ocinit.TIM_OCNPolarity = TIM_OCNPolarity_High;
    tim_ocinit.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    pinMapping[pinID].tim_oc_init(moduleMapping[m_idx].timer_peripheral, &tim_ocinit);
    pinMapping[pinID].tim_oc_preload_config(moduleMapping[m_idx].timer_peripheral, TIM_OCPreload_Enable);
      
    /* compute the prescaler value */
    prescaler = ( SystemCoreClock / moduleMapping[m_idx].timer_prescale_divider / 1000000.0f) - 1;

    /* time base configuration */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = 20000 - 1;;
    tim_base.TIM_Prescaler = prescaler;
    tim_base.TIM_ClockDivision = 0;
    tim_base.TIM_RepetitionCounter = 0;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(moduleMapping[m_idx].timer_peripheral, &tim_base);

    if( (moduleMapping[m_idx].timer_peripheral == TIM1) || (moduleMapping[m_idx].timer_peripheral == TIM8) )
    {
        TIM_BDTRStructInit(&bdtr);
        bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
        TIM_BDTRConfig(moduleMapping[m_idx].timer_peripheral, &bdtr);
    }

    /* enable the timer */
    TIM_ARRPreloadConfig(moduleMapping[m_idx].timer_peripheral, ENABLE);
    TIM_Cmd(moduleMapping[m_idx].timer_peripheral, ENABLE);
    TIM_CtrlPWMOutputs(moduleMapping[m_idx].timer_peripheral, ENABLE);
}

/************************************************************************************//**
** \brief     Updates the pulse width.
** \param     pinID Pin identifier.
** \param     pulse_width_in_us New pulse width in microsecond.
** \return    none.
**
****************************************************************************************/
void ServoUpdate(const uint8_t pinID, uint16_t pulse_width_in_us)
{
    // make sure the id is valid before using it as an array indexer
    if(!(pinID < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
        ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
	// Verify input parameter
	if(pulse_width_in_us < m_min_pulse_width_in_us[pinID])
	{
		pulse_width_in_us = m_min_pulse_width_in_us[pinID];
	}

	if(pulse_width_in_us > m_max_pulse_width_in_us[pinID])
	{
		pulse_width_in_us = m_max_pulse_width_in_us[pinID];
	}
	
	// Set the new compare value
	pinMapping[pinID].tim_set_compare(moduleMapping[pinMapping[pinID].module_idx].timer_peripheral, pulse_width_in_us);
}

/********************************* end of servo.c *************************************/