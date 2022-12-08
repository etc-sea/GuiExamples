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

/* also the Embedded Wizard runtime environment is needed */
#include "ewrte.h"

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
  /* Configure SideLedbar pins as output */
  GPIO_InitTypeDef init;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Speed = GPIO_SPEED_HIGH;
  /* D0 => PC7 */
  init.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOC, &init);
  /* D1 => PC6 */
  init.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOC, &init);
  /* D2 => PG6 */
  init.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOG, &init);
  /* D3 => PB4 */
  init.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &init);
  /* D4 => PG7 */
  init.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOG, &init);
  /* don't forget the pin clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();


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
     The LEDs do not need to signal back the state to the GUI
  */

  /*
     Trigger system events if necessary, e.g. if a certain situation happens,
     if an error occurs or just if a certain value has changed...
  */

  return needUpdate;
}


/*******************************************************************************
* FUNCTION:
*   DeviceDriver_SetLedBar
*
* DESCRIPTION:
*   This function turns on or off a led on the sidebar
*
* PARAMETERS:
*   - aLed: LED number (0 to 4)
*   - aState: LED state (0: off, 1: on)
*
*******************************************************************************/
void DeviceDriver_SetLedBar( XInt32 aLed, XBool aState )
{
  switch ( aLed )
  {
    case 0:
      /* LED0 => PG7 */
      HAL_GPIO_WritePin( GPIOG, GPIO_PIN_7, aState ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 1:
      /* LED1 => PB4 */
      HAL_GPIO_WritePin( GPIOB, GPIO_PIN_4, aState ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 2:
      /* LED2 => PG6 */
      HAL_GPIO_WritePin( GPIOG, GPIO_PIN_6, aState ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 3:
      /* LED3 => PC6 */
      HAL_GPIO_WritePin( GPIOC, GPIO_PIN_6, aState ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 4:
      /* LED4 => PC7 */
      HAL_GPIO_WritePin( GPIOC, GPIO_PIN_7, aState ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
  }
}
