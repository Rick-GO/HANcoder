/************************************************************************************//**
* \file         servo.h
* \brief        Servo driver header file.
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

#ifndef SERVO_H_
#define SERVO_H_

/****************************************************************************************
Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define SERVO_TIM1_PIN_PE9    (0)                          /* Connector PE-12 (Ch. 1)  */
#define SERVO_TIM1_PIN_PE11   (1)                          /* Connector PE-14 (Ch. 2)  */
#define SERVO_TIM1_PIN_PE13   (2)                          /* Connector PE-16 (Ch. 3)  */
#define SERVO_TIM1_PIN_PE14   (3)                          /* Connector PE-17 (Ch. 4)  */
#define SERVO_TIM2_PIN_PA5    (4)                          /* Connector 4-D13 (Ch. 1)  */
#define SERVO_TIM3_PIN_PA6    (5)                          /* Connector 4-D12 (Ch. 1)  */
#define SERVO_TIM3_PIN_PB5    (6)                          /* Connector 4-D11 (Ch. 2)  */
#define SERVO_TIM4_PIN_PD12   (7)                          /* Connector PD-15 (Ch. 1)  */
#define SERVO_TIM4_PIN_PD13   (8)                          /* Connector PD-16 (Ch. 2)  */
#define SERVO_TIM8_PIN_PC6    (9)                          /* Connector UEXT-3 (Ch. 1) */
#define SERVO_TIM8_PIN_PC7    (10)                         /* Connector UEXT-4 (Ch. 2) */
#define SERVO_TIM9_PIN_PE5    (11)                         /* Connector 3-D4  (Ch. 1)  */
#define SERVO_TIM9_PIN_PE6    (12)                         /* Connector 3-D5  (Ch. 2)  */
#define SERVO_TIM10_PIN_PF6   (13)                         /* Connector PF-9  (Ch. 1)  */
#define SERVO_TIM11_PIN_PF7   (14)                         /* Connector PF-10 (Ch. 1)  */
#define SERVO_TIM13_PIN_PF8   (15)                         /* Connector PF-11 (Ch. 1)  */
#define SERVO_TIM14_PIN_PF9   (16)                         /* Connector PF-12 (Ch. 1)  */

/****************************************************************************************
* Type definitions
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void ServoInit(const uint8_t pinID, const uint16_t min_pulse_width_in_us, const uint16_t max_pulse_width_in_us);
void ServoUpdate(const uint8_t pinID, uint16_t pulse_width_in_us);

#endif /* SERVO_H_ */
/********************************* end of servo.h *************************************/
