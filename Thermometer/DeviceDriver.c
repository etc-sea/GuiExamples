/*******************************************************************************
* FILE:
*   DeviceDriver.c
********************************************************************************
*
* DESCRIPTION:
*   This file implements an interface between an Embedded Wizard generated UI
*   application and the Hardware Abstraction Layer proviced for the STM32
*   microcontroller
*   This device driver is the counterpart to a device class implemented within
*   your Embedded Wizard project.
*
********************************************************************************
*
*   Within this example, the access to the GPIO pins is demonstrated by
*   connecting 5 LEDs to the D0-D4 pins of the Arduino header
*
*******************************************************************************/

/* HAL header files to be used */
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* also the Embedded Wizard runtime environment is needed */
#include "ewrte.h"

/* the BSP interface for EW */
#include "ew_bsp_event.h"

/* and the header filed generated for the unit containing the DeviceInterface
   based class in the UI
*/
#include "Application.h"

/*
   Create a static variable to keep the global instance (autoobject) of the
   device class. The type of the variable has to correspond to the device class
   and the unit name, for example the static variable of the class 'DeviceClass'
   from the unit 'Application' has the type 'ApplicationDeviceClass'.
*/
static ApplicationDeviceClass DeviceObject = 0;

/* peripheral handles */
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* DMA buffer */
uint16_t DHT_reply[42];

/* process loop flag */
XBool DHT_reply_ready = 0;

/* connect TIM1 Channel1 peripheral to the DMA2 interrupt handler */
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}


/* Callback function of the Timer InputCapture */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* TIM1 DMA DeInit */
  HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_DeInit(&htim1);

  /* notify the GUI thread about the reading completed */
  DHT_reply_ready = 1;

  /* also make sure that a new UI update cycle is issued as soon as possible */
  EwBspEventTrigger();
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_Initialize
*
* DESCRIPTION:
*   This function initializes the HAL components needed by the device driver
*   The function is called from the main module, after the initialization
*   of your GUI application.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void DeviceDriver_Initialize( void )
{
  /* configure LED */
  BSP_LED_Init(LED1);

  // make sure GPIOA is clocked (normally it is, but to be safe)
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // configure global peripheral handle structures
  htim1.Instance = TIM1;

  /*
     Get access to the counterpart of this device driver: get access to the
     device class that is created as autoobject within the Embedded Wizard
     project.
  */
  DeviceObject = EwGetAutoObject( &ApplicationDevice, ApplicationDeviceClass );

  /*
     After accessing the autoobject, lock it as long as you need
     the access (typically, until your device driver is closed).
     Locking it ensures that the object of the device class will
     be kept within the memory and not freed by the Garbage Collector.
  */
  EwLockObject( DeviceObject );
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_Deinitialize
*
* DESCRIPTION:
*   This function deinitializes the HAL components initialized at the beginning
*   The function is called from the main module, before the GUI
*   application will be deinitialized.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void DeviceDriver_Deinitialize( void )
{
  BSP_LED_DeInit(LED1);

  /*
     Don't forget to unlock the autoobject
  */

  if ( DeviceObject )
    EwUnlockObject( DeviceObject );

  DeviceObject = 0;
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_ProcessData
*
* DESCRIPTION:
*   This function is called from the main UI loop, in
*   order to process data and events from your particular device.
*   This function is responisble to update properties within the device class
*   if the corresponding state or value of the real device has changed.
*   This function is also responsible to trigger system events if necessary.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   The function returns a non-zero value if a property has changed or if a
*   system event was triggered. If nothing happens, the function returns 0.
*
*******************************************************************************/
int DeviceDriver_ProcessData( void )
{
  int needUpdate = 0;

  /* check for a valid access to the autoobject of the device class */
  if ( DeviceObject == 0 )
    return 0;

  /*
     This is the place to call the UpdateProperty functions of the GUI
     DeviceInterface properties, if any
  */

  /* DHT11 DMA transmission finished */
  if (DHT_reply_ready)
  {
    uint32_t bits = 0;
    uint8_t crc = 0;
    XBool success;
    uint32_t i;

    DHT_reply_ready = 0;

    for (i = 0; i < 32; i++)
    {
      if (DHT_reply[i+2] - DHT_reply[i+1] > 90)
        bits |= 1 << (31 -i);
    }
    for (i = 0; i < 8; i++)
    {
      if (DHT_reply[i+34] - DHT_reply[i+33] > 90)
        crc |= 1 << (7 -i);
    }
    success = crc == (uint8_t)(bits >> 24) + (uint8_t)(bits >> 16) + (uint8_t)(bits >> 8) + (uint8_t)bits;

    BSP_LED_Toggle(LED1);

    /* if transmission was successful, send back to the GUI the decoded
       temperature and humidity values */
    ApplicationDeviceClass__UpdateDHT11State( DeviceObject,
      (XInt32)((bits >> 8) & 0xFF ),
      (XInt32)((bits >> 24) & 0xFF ),
      success);
    needUpdate = 1;
  }

  /*
     Trigger system events if necessary, e.g. if a certain situation happens,
     if an error occurs or just if a certain value has changed...
  */

  return needUpdate;
}

/*******************************************************************************
* FUNCTION:
*   DeviceDriver_StartCondition
*
* DESCRIPTION:
*   Issue a start condition to the DHT11 sensor
*   Start condition is pulling the data line for at least 18ms
*
* PARAMETERS:
*   - None
*
*******************************************************************************/
void DeviceDriver_StartCondition()
{
  // configure the pin as opendrain output
  GPIO_InitTypeDef init;
  init.Mode = GPIO_MODE_OUTPUT_OD;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_HIGH;
  init.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &init);
  // and pull down
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_StartReceive
*
* DESCRIPTION:
*   Start a receive operation from the DHT11
*   this will issue a DMA transfer from Input Capture to measure the bit
*   lengths sent by the sensor
*   42 capture events are transferred, to include also the sync reply
*
* PARAMETERS:
*   - None
*
*******************************************************************************/
void DeviceDriver_StartReceive()
{
  // switch the sensor pin to input connected to the timer
  GPIO_InitTypeDef init;
  init.Mode = GPIO_MODE_AF_OD;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_HIGH;
  init.Alternate = GPIO_AF1_TIM1;
  init.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &init);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  // make sure timer is reset
  HAL_TIM_Base_DeInit(&htim1);

  /* first the DMA channel must be initialized and after that the Timer itself
     otherwise the DMA will just ignore the event flags from the Timer and
     no transmission will take place */

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  /* TIM1 DMA Init */
  /* TIM1_CH1 Init */
  hdma_tim1_ch1.Instance = DMA2_Stream1;
  hdma_tim1_ch1.Init.Channel = DMA_CHANNEL_6;
  hdma_tim1_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
  hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_tim1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
  {
     return;
  }

  __HAL_LINKDMA(&htim1,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

  /* after DMA is initialized, initialize the timer as well */
  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim1.Init.Prescaler = (SystemCoreClock / 1000000) - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
      return;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
      HAL_TIM_Base_DeInit(&htim1);
      return;
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
        HAL_TIM_Base_DeInit(&htim1);
      return;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
        HAL_TIM_Base_DeInit(&htim1);
      return;
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
        HAL_TIM_Base_DeInit(&htim1);
      return;
  }

  /* finally, after the peripherals are initialized, strt the the DMA transfer */
  HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)DHT_reply, 42);
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_CancelReceive
*
* DESCRIPTION:
*   Cancel a running DMA operation (most likely due to a timeout)
*
* PARAMETERS:
*   - None
*
*******************************************************************************/
void DeviceDriver_CancelReceive()
{
    // stop DMA
    HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_DeInit(&htim1);
}
