/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @Author			: Dmitrij Bauer
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**********Deklaration der öffentlichen Datentypen**********/

/**
 * Datenstruktur zur Verwaltung von einem Gpio-Pin.
 * Für das Übetragen von Bits soll die Methode
 * HAL_setPin(GpioPin, uint8_t) verwendet werden.
 */
typedef struct gpiopin_{
	uint16_t pin;				//Gpio-Pin.
	GPIO_TypeDef* gpioPort;		//der dazugehörige Gpio-Port.
}GpioPin;

/**
 * Datenstruktur zur Verwaltung der Gpio-Pins.
 * Schnittstelle zur Verwendung der Pins von außerhalb
 * der main.c Datei.
 * Vor der Verwendung muss diese Datenstruktur initialisiert
 * werden, indem die vom Mikrocontroller definierten Pins
 * den Gpio-Pins in der Datenstruktur zugeordnet werden.
 *
 */
typedef struct gpiopininterface_{
	struct gpiopin_ dataPins[4];	//Gpio-Pins, die für das Übertragen der Daten zuständig sind.
	struct gpiopin_ rsPin;			//RS-Pin : 1=Daten werden übertragen/0=Anweisung wird übertragen.
	struct gpiopin_ ePin;			//E-Pin. Soll gesetzt werden, falls ein Byte übertragen werden soll.
	struct gpiopin_ tempPin;		//Pin zur Kommunikation mit dem Temperatur-Sensor (One Wire).
}GpioPinInterface;


/**********Deklaration von öffentlichen Funktionen**********/

/**
 * Blockiert das Hauptprogramm für eine bestimmte Anzahl
 * an Mikrosekunden.
 * Für die Zeitmessung wird der Timer TIM14 verwendet. Beim Aufruf der
 * Funktion wird im ersten Schritt der Zähler des Timers auf den Wert
 * 0 zurückgesetzt. Im zweiten Schritt wird der Wert des Zählers aktiv geprüft, 
 * bis dieser den gewünschten Wert erreicht hat.
 * Diese Funktion setzt voraus, dass TIM14 bereits initialisiert und gestartet
 * wurde.
 * ------
 * Quelle: https://controllerstech.com/ds18b20-and-stm32/
 * Die Funktion "void delay(us)" bietet eine sehr kurze und einfache
 * Implementierung einer präzisen Delay-Funktion. Da diese nur aus
 * zwei Codezeilen besteht, wurde diese so übernommen.
 * ------
 * @param us	Anzahl der Mikrosekunden, die gewartet werden soll.
 */
void waitUs(uint16_t us);

/**
 * Liefert den ausgelesenen ADC-Wert als Prozentsatz (0-100).
 * Der in getAbsADCValue() ausgelesene Wert wird anhand
 * des Wertes in adcResolution skaliert.
 */
uint8_t getRelADCValue();

/**
 * Sendet ein String über die UART-Schnittstelle.
 * @param buffer	String, welches übertragen werden soll.
 */
void UART_log(char buffer[]);

/**
 * Initialisiert den Timer TIM6.
 */
void initTimer();

/**
 * Schreibt das gegebene Bit am entsprechenden Pin.
 * @param gpioPin	Gpio-Pin, an dem das Bit ausgegeben werden soll.
 * @param bit		Bit, welches am Pin ausgegeben werden soll (0<=bit<=1);
 */
void HAL_setPin(GpioPin gpioPin, uint8_t bit);

/**
 * Liest das Bit vom entsprechenden Pin.
 * @param gpioPin	Gpio-Pin, welches gelesen werden soll.
 * @return 			das ausgelesene Bit.
 */
uint8_t HAL_readPin(GpioPin gpioPin);

/**
 * Setzt den gegebenen Pin in den Output-Modus.
 * @param gpioPin	Gpio-Pin, welcher in den Output-Modus gesetzt  werden soll.
 */
void HAL_pinToOutput(GpioPin gpioPin);

/**
 * Setzt den gegebenen Pin in den Input-Modus.
 * @param gpioPin	Gpio-Pin, welcher in den Input-Modus gesetzt  werden soll.
 */
void HAL_pinToInput(GpioPin gpioPin);

/**********Globale Variablen**********/

GpioPinInterface pinInterface;		//Pin-Schnittstelle(globale Variable).


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define HUMID_Pin GPIO_PIN_0
#define HUMID_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_5
#define TEMP_GPIO_Port GPIOC
#define LCD_DB4_Pin GPIO_PIN_1
#define LCD_DB4_GPIO_Port GPIOB
#define BT_DOWN_Pin GPIO_PIN_10
#define BT_DOWN_GPIO_Port GPIOB
#define BT_DOWN_EXTI_IRQn EXTI4_15_IRQn
#define LCD_DB7_Pin GPIO_PIN_13
#define LCD_DB7_GPIO_Port GPIOB
#define LCD_DB6_Pin GPIO_PIN_14
#define LCD_DB6_GPIO_Port GPIOB
#define LCD_DB5_Pin GPIO_PIN_15
#define LCD_DB5_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_6
#define LCD_E_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOC
#define BT_UP_Pin GPIO_PIN_8
#define BT_UP_GPIO_Port GPIOA
#define BT_UP_EXTI_IRQn EXTI4_15_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BT_LEFT_Pin GPIO_PIN_4
#define BT_LEFT_GPIO_Port GPIOB
#define BT_LEFT_EXTI_IRQn EXTI4_15_IRQn
#define BT_RIGHT_Pin GPIO_PIN_5
#define BT_RIGHT_GPIO_Port GPIOB
#define BT_RIGHT_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
