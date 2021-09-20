/************************************************************************************//**
* \file         anin.c
* \brief        Analog inputs driver source file.
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include "anin.h"                                     /* Analog input driver           */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f30x.h"                                /* STM32 registers               */
#include "stm32f30x_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Total number of analog input channels */
#define ANIN_MAX_CHANNELS         (16)

/** \brief Invalid index value for array aninChannelCfgInfo */
#define ANIN_INVALID_CHANNEL_IDX  (ANIN_MAX_CHANNELS)

#define ADC1CHANNELS	(8)
#define ADC2CHANNELS	(6)
#define ADC3CHANNELS	(2)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type with pin mapping information. */
typedef struct
{
  uint32_t peripheral;
  GPIO_TypeDef* port;
  uint16_t pin;
  ADC_TypeDef* adc;
  uint8_t adcChannel;
  uint8_t adcId;
} tAninPinMapping;

/** \brief Structure type to keep track of a channel configuration. */
typedef struct
{
  uint8_t pinIdx;
  uint8_t configuredFlg;
} tAninChannelCfgInfo;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void    AninInit(void);
static void    AninReinit(void);
static uint8_t AninGetNumConfiguredChannels(uint8_t adcId);
static uint8_t AninGetChannelIndex(uint8_t id);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tAninPinMapping pinMapping[] =
{
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_0, ADC1, ADC_Channel_6, 0 }, /* 0:  ANIN_PORTC_PIN0 ADC1 CH6 */
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_1, ADC1, ADC_Channel_7, 0 }, /* 1:  ANIN_PORTC_PIN1 ADC1 CH7 */
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_2, ADC1, ADC_Channel_8, 0 }, /* 2:  ANIN_PORTC_PIN2 ADC1 CH8 */
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_3, ADC1, ADC_Channel_9, 0 }, /* 3:  ANIN_PORTC_PIN3 ADC1 CH9 */
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_4, ADC2, ADC_Channel_5, 1 }, /* 4:  ANIN_PORTC_PIN4 ADC2 CH5 */
  { RCC_AHBPeriph_GPIOC, GPIOC, GPIO_Pin_5, ADC2, ADC_Channel_11,1 }, /* 5:  ANIN_PORTC_PIN5 ADC2 CH11 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_3, ADC1, ADC_Channel_4, 0 }, /* 6:  ANIN_PORTA_PIN3 ADC1 CH4 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_2, ADC1, ADC_Channel_3, 0 }, /* 7:  ANIN_PORTA_PIN2 ADC1 CH3 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_0, ADC1, ADC_Channel_1, 0 }, /* 8:  ANIN_PORTA_PIN0 ADC1 CH1 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_1, ADC1, ADC_Channel_2, 0 }, /* 9:  ANIN_PORTA_PIN1 ADC1 CH2 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_4, ADC2, ADC_Channel_1, 1 }, /* 10: ANIN_PORTA_PIN4 ADC2 CH1 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_7, ADC2, ADC_Channel_4, 1 }, /* 11: ANIN_PORTA_PIN7 ADC2 CH8 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_6, ADC2, ADC_Channel_3, 1 }, /* 12: ANIN_PORTA_PIN6 ADC2 CH7 */
  { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_5, ADC2, ADC_Channel_2, 1 }, /* 13: ANIN_PORTA_PIN5 ADC2 CH2 */
  { RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_0, ADC3, ADC_Channel_12,2 }, /* 14: ANIN_PORTB_PIN0 ADC3 CH12 */
  { RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_1, ADC3, ADC_Channel_1, 2 }  /* 15: ANIN_PORTB_PIN1 ADC3 CH1 */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array for storing the analog to digital conversion results. */
static uint16_t aninConversionResults1[ADC1CHANNELS]; // Number of ADC1 Channels
static uint16_t aninConversionResults2[ADC2CHANNELS]; // Number of ADC2 Channels
static uint16_t aninConversionResults3[ADC3CHANNELS]; // Number of ADC3 Channels

/** \brief Flag to determine is the driver was already initialized. */
static uint8_t aninInitialized = FALSE;

/** \brief Array to keep track of a channel's configuration. */
/*TODO better memory performance  */
static tAninChannelCfgInfo aninChannelCfgInfo[3][ANIN_MAX_CHANNELS];

/** \brief Flag to determine if the configuration was changed and requires updating. */
static uint8_t aninConfigurationChangedFlg = FALSE;

/** \brief Variable that holds the period time of ADCconversionTask in OS ticks. This allows
 **         a dynamic override through function ADCconversionTaskSetPeriod. */
static portTickType ADCconversionTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);


/************************************************************************************//**
** \brief     Initializes the analog inputs driver.
** \return    none.
**
****************************************************************************************/
static void AninInit(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  uint8_t idx;

  /* only do initialization once */
  if (aninInitialized == FALSE)
  {
    /* set flag */
    aninInitialized = TRUE;
    
	for (idx=0; idx<ADC1CHANNELS; idx++)
    {
		aninConversionResults1[idx] = 0;
		aninChannelCfgInfo[0][idx].configuredFlg = FALSE; 
	}
	
	for (idx=0; idx<ADC2CHANNELS; idx++)
    {
		aninConversionResults2[idx] = 0;
		aninChannelCfgInfo[1][idx].configuredFlg = FALSE; 
	}
	
	for (idx=0; idx<ADC3CHANNELS; idx++)
    {
		aninConversionResults3[idx] = 0;
		aninChannelCfgInfo[2][idx].configuredFlg = FALSE; 
	}
	
    /* enable the DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* enable the DMA2 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	/* enable the ADC12 clocks */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div8);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);

	/* TODO implement another, defined delay mechanism */
	uint32_t counter = 0;
	while(counter++ <= 1000000);

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);
	
	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC3);
	
	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	while(ADC_GetCalibrationStatus(ADC3) != RESET );

	
    /* ADC common init */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 5;
	
    ADC_CommonInit(ADC1,&ADC_CommonInitStructure);
	ADC_CommonInit(ADC2,&ADC_CommonInitStructure);
	ADC_CommonInit(ADC3,&ADC_CommonInitStructure);

  }
} /*** end of AninInit ***/


/************************************************************************************//**
** \brief     Reinitializes the analog input driver for a different number of channels.
** \return    none.
**
****************************************************************************************/

static void AninReinit(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  uint8_t channelId;
  uint8_t adcId;

  /* De initialize current configurations */
  /* NOTE: De-initializations needs to be done before the new initializations! */
  DMA_DeInit(DMA1_Channel1);
  DMA_DeInit(DMA2_Channel1);
  DMA_DeInit(DMA2_Channel5);
  
  ADC_DeInit(ADC1);
  ADC_DeInit(ADC2);
  ADC_DeInit(ADC3);
  
  /* Initialize ADC1 with DMA 1 Channel 1 */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aninConversionResults1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = AninGetNumConfiguredChannels(0);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = AninGetNumConfiguredChannels(0);
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Initialize ADC2 with DMA 2 Channel 1 */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aninConversionResults2;
  DMA_InitStructure.DMA_BufferSize = AninGetNumConfiguredChannels(1);
  DMA_Init(DMA2_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Channel1, ENABLE);

  ADC_InitStructure.ADC_NbrOfRegChannel = AninGetNumConfiguredChannels(1);
  ADC_Init(ADC2, &ADC_InitStructure);

  /* Initialize ADC3 with DMA 2 Channel 5 */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aninConversionResults3;
  DMA_InitStructure.DMA_BufferSize = AninGetNumConfiguredChannels(2);
  DMA_Init(DMA2_Channel5, &DMA_InitStructure);
  DMA_Cmd(DMA2_Channel5, ENABLE);

  ADC_InitStructure.ADC_NbrOfRegChannel = AninGetNumConfiguredChannels(1);
  ADC_Init(ADC3, &ADC_InitStructure);

  /* Configure all the ADC channels */
  for (adcId=0; adcId<3; adcId++)
  {
	  for (channelId=0; channelId<AninGetNumConfiguredChannels(adcId); channelId++)
	  {
		ADC_RegularChannelConfig(pinMapping[aninChannelCfgInfo[adcId][channelId].pinIdx].adc, 
								 pinMapping[aninChannelCfgInfo[adcId][channelId].pinIdx].adcChannel,
									channelId+1, ADC_SampleTime_61Cycles5);
	  }
  }

  /* Activate DMA and ADC peripherlas */
  ADC_DMAConfig(ADC1,ADC_DMAMode_OneShot);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  ADC_StartConversion(ADC1);


  ADC_DMAConfig(ADC2,ADC_DMAMode_OneShot);
  ADC_DMACmd(ADC2, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
  ADC_StartConversion(ADC2);


  ADC_DMAConfig(ADC3,ADC_DMAMode_OneShot);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));
  ADC_StartConversion(ADC3);
} /*** end of AninReinit ***/


/************************************************************************************//**
** \brief     Finds the number of channels that are configured.
** \return    Number of configured channels.
**
****************************************************************************************/
static uint8_t AninGetNumConfiguredChannels(uint8_t adcId)
{
  uint8_t count = 0;

  for (count=0; count<ANIN_MAX_CHANNELS; count++)
  {
    if (aninChannelCfgInfo[adcId][count].configuredFlg == FALSE)
    {
      break;
    }
  }

  return count;
} /*** end of AninGetNumConfiguredChannels ***/


/************************************************************************************//**
** \brief     Obtains the index into the aninChannelCfgInfo[] array of a specific pin.
** \param     id Pin identifier.
** \return    Index into the aninChannelCfgInfo[] array or ANIN_INVALID_CHANNEL_IDX if
**            not found.
**
****************************************************************************************/
static uint8_t AninGetChannelIndex(uint8_t id)
{
  uint8_t count;
  uint8_t channelIdx = ANIN_INVALID_CHANNEL_IDX;
  uint8_t adcId = pinMapping[id].adcId;

  /* iterate through the array */
  for (count=0; count<ANIN_MAX_CHANNELS; count++)
  {
    /* is this the one we are looking for? */
    if ( (aninChannelCfgInfo[adcId][count].configuredFlg == TRUE) &&
         (aninChannelCfgInfo[adcId][count].pinIdx == id) )
    {
      /* store it and stop searching */
      channelIdx = count;
      break;
    }
  }
  /* return the results */
  return channelIdx;
} /*** end of AninGetChannelIndex ***/


/************************************************************************************//**
** \brief     Configures a analog input pin.
** \param     id Pin identifier.
** \return    none.
**
****************************************************************************************/
void AninConfigure(uint8_t id, uint8_t filtered)
{
  GPIO_InitTypeDef gpio_init;
  uint8_t nextFreeChannelIdx[3];
  uint8_t adcId = pinMapping[id].adcId;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_ANIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* make sure the driver is initialized */
  AninInit();
  /* only continue if a configuration for this pin identifier wasn't already made */
  if (AninGetChannelIndex(id) == ANIN_INVALID_CHANNEL_IDX)
  {
    /* enable the port's peripheral clock */
    RCC_AHBPeriphClockCmd(pinMapping[id].peripheral, ENABLE);
    /* prepare pin configuration */
    gpio_init.GPIO_Pin = pinMapping[id].pin;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    /* initialize the pin */
    GPIO_Init(pinMapping[id].port, &gpio_init);
    /* determine next free channel index */
    nextFreeChannelIdx[adcId] = AninGetNumConfiguredChannels(adcId);
    /* make sure there is still a free spot in the results array */
	/*TODO ADC channel specific check (*/
    if (!(nextFreeChannelIdx[adcId] < ANIN_MAX_CHANNELS))
    {
      ErrCodesSetError(ER_CODE_ANIN_NO_FREE_CHANNELS, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* store what pin identifier belongs to the newly configured channel */
    aninChannelCfgInfo[adcId][nextFreeChannelIdx[adcId]].pinIdx = id;
    aninChannelCfgInfo[adcId][nextFreeChannelIdx[adcId]].configuredFlg = TRUE;
    /* request processing of the configuration change */
    aninConfigurationChangedFlg = TRUE;
  }
} /*** end of AninConfigure ***/


/************************************************************************************//**
** \brief     Obtains the conversion result of the analog input pin.
** \param     id Pin identifier.
** \return    12-bit analog to digital conversion result.
**
****************************************************************************************/
uint16_t AninGet(uint8_t id, uint8_t filtered)
{
  uint8_t channelIdx;
  uint16_t result = 0;
  uint8_t adcId = pinMapping[id].adcId;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_ANIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* if no analog inputs are configured, the driver is not initialized and nothing
   * needs to be done
   */
  if (aninInitialized == TRUE)
  {
    /* find the channel index that belongs to this pin identifier */
    channelIdx = AninGetChannelIndex(id);

    if (channelIdx != ANIN_INVALID_CHANNEL_IDX)
    {
      /* read from the results array where the conversion result was stored by the DMA */
	  if(adcId == 0)
	  {
      result = aninConversionResults1[channelIdx];
	  }
	  else if(adcId == 1)
	  {
	  result = aninConversionResults2[channelIdx];
	  }
	  else if(adcId == 2)
	  {
	  result = aninConversionResults3[channelIdx];
	  }
    }
  }
  /* return the result */
  return result;
} /*** end of AninGet ***/


/************************************************************************************//**
** \brief     Starts an analog to digital conversion sequence for all configured
**            channels.
** \return    none.
**
****************************************************************************************/
void AninConvert(void)
{
  /* if no analog inputs are configured, the driver is not initialized and nothing
   * needs to be done
   */
  if (aninInitialized == TRUE)
  {
    /* is a configuration change request pending? */
    if (aninConfigurationChangedFlg == TRUE)
    {
      /* reset flag */
      aninConfigurationChangedFlg = FALSE;
      /* re-initialize the driver for the new configuration */
      AninReinit();
    }
    /* start the conversion */
    ADC_StartConversion(ADC1);
	ADC_StartConversion(ADC2);
	ADC_StartConversion(ADC3);
  }
} /*** end of AninConvert ***/

/************************************************************************************//**
** \brief     Configures the period time of ADCfilterTask.
** \param     period Task period time in microseconds.
** \return    none.
**
****************************************************************************************/
void ADCconversionTaskSetPeriod(uint32_t period)
{
  ADCconversionTaskPeriodTicks = ((portTickType)period/portTICK_PERIOD_US);
} /*** end of ADCconversionTaskSetPeriod ***/

/************************************************************************************//**
** \brief     Task to automatically start conversion and filter the ADC results
** \param     *pvParameters (required by FREERTOS)
** \return    none.
** Ticket ref #71
****************************************************************************************/
void ADCconversionTask(void *pvParameters)
{
  /* Declare local variables */
  portTickType lastWakeTime;

  /* initialize to current time */
  lastWakeTime = xTaskGetTickCount();

  /* enter task body */
  for( ;; )
  {
	/*Start conversion of ADC's (and filtering) */
	ADCconversionTaskFunction();
	/* activate this task periodically, run ten times faster than application */
    vTaskDelayUntil(&lastWakeTime, ADCconversionTaskPeriodTicks);
  } /* End of task body */
  
} /*** End of ADCconversionTask ***/

/************************************************************************************//**
** \brief     Task to filter the ADC results
** \param     none
** \return    none.
** Ticket ref #71
****************************************************************************************/
void ADCconversionTaskFunction(void)
{
	
	/* Start the analog conversions */
	AninConvert();	
} /*** end of ADCfilterTaskFunction ***/


/************************************ end of anin.c ************************************/


