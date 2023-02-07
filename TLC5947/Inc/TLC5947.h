/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc5947.h
  * @brief          : Header for tlc5947.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TLC5947_H
#define __TLC5947_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
//#include "spi1.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TLC5947_CHANNELS 24 //Cantidad de canales a escribir por cada tlc5947
#define TLC5947_DRIVER_AMOUNT 1 //Cantidad de tlc5947 que uso
#define TOTAL_CHANNELS (TLC5947_CHANNELS*TLC5947_DRIVER_AMOUNT)
#define SPI_BYTE_AMOUNT (TLC5947_CHANNELS*12*TLC5947_DRIVER_AMOUNT/8) //Para solo una placa tengo que enviar 12 bits x 24 canales/8 bits = 36 Bytes

//possible colors
#define RED 48
#define GREEN 49
#define BLUE 50
#define RGB 51 // currently not supported
//led intensity
#define HIGH 52
#define MID 53
#define LOW 54
//IMG or STR to display
#define FABITEST 0
#define LINE 1
#define LETTERa 2

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void TLC_Write(uint8_t data[]);// prototipo para mandar todo un vector de bytes
void TLC_Update(void);
void FillArray(uint8_t colorIntensity,uint8_t ledControl);// fills leds array depending on what is input in control

/* USER CODE END PFP */


#endif /* __TLC5947_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
