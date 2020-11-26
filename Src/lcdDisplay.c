
/**
 * HEADER
 *
 * Modul zur Kommunikation mit einem 16x2 LCD-Display.
 * @Author: Dmitrij Bauer, Tim Wegner
 */

#include "lcdDisplay.h"

/**********private Defines**********/

#define NIBBLE_SIZE 4
#define BYTE_SIZE NIBBLE_SIZE * 2
#define ENABLE_DELAY 3
#define DISPLAY_CLEAR_DELAY 2

//Function Set
#define FUNCTION_SET 0x20
#define FUNCTION_SET_8MODE 0x10
#define FUNCTION_SET_4MODE 0x00
#define FUNCTION_SET_1ROW 0x00
#define FUNCTION_SET_2ROWS 0x08
#define FUNCTION_SET_FSIZE_5_8 0x00
#define FUNCTION_SET_FSIZE_5_11 0x04

//Clear Display
#define CLEAR_DISPLAY 0x01

//Display ON/OFF
#define DISPLAY_ON 0x0c
#define DISLPAY_OFF 0x08
#define DISPLAY_CURSOR_ON 0x02
#define DISPLAY_CURSOR_OFF 0x00
#define DISPLAY_CURSOR_POS_ON 0x01
#define DISPLAY_CURSOR_POS_OFF 0x00

//Entry Mode
#define ENTRY_MODE 0x04
#define ENTRY_MODE_CURSER_INCR 0x02

//Set DDRAM Adress
#define SET_DDRAM_ADR 0x80
#define SET_DDRAM_ADR_1ROW 0x00
#define SET_DDRAM_ADR_2ROW 0x40


/**********Deklaration von privaten Funktionen**********/

/**
 * Überträgt das obere Nibble (Bit: 7, 6, 5, 4) eines Byte an das LCD-Display.
 * @param byte	Byte, welches übertragen werden soll. Es werden nur
 * 				die oberen 4 Bit übertragen.
 */
void sendNibble(uint8_t nibble);

/**
 * Setzt alle Gpio-Pins der Schnittstelle zurück.
 */
void clearOutputPins();

/**********Definition von Funktionen**********/

void sendString(enum LCDRow row, char* string){
	switch (row) {
		case FIRST_ROW:
			sendByte(False, False, SET_DDRAM_ADR | SET_DDRAM_ADR_1ROW);
			break;
		case SECOND_ROW:
			sendByte(False, False, SET_DDRAM_ADR | SET_DDRAM_ADR_2ROW);
			break;
	}
	char* tempPointer = string;
	char c = *tempPointer;
	int charCounter = 0;
	while(c != '\0' && charCounter < DISPLAY_ROW_LENGTH){
		charCounter++;
		sendByte(True, False, c);
		tempPointer++;
		c = *tempPointer;
	}
}

void initLcdDisplay(){
	HAL_Delay(120);
	clearOutputPins();
	sendByte(False, False, FUNCTION_SET |
						  FUNCTION_SET_8MODE);
	sendByte(False, False, FUNCTION_SET |
						   FUNCTION_SET_8MODE);
	sendByte(False, True, FUNCTION_SET |
						  FUNCTION_SET_4MODE);
	sendByte(False, False, FUNCTION_SET |
						   FUNCTION_SET_4MODE |
						   FUNCTION_SET_2ROWS |
						   FUNCTION_SET_FSIZE_5_8);
	sendByte(False, False, DISLPAY_OFF);
	clearDisplay();
	sendByte(False, False, ENTRY_MODE |
		   	   	   ENTRY_MODE_CURSER_INCR);
	sendByte(False, False, DISPLAY_ON |
						   DISPLAY_CURSOR_POS_OFF);
}

void clearDisplay(){
	sendByte(False, False, CLEAR_DISPLAY);
	HAL_Delay(DISPLAY_CLEAR_DELAY);
}

void enableCursor(){
	sendByte(False, False, SET_DDRAM_ADR |
						   SET_DDRAM_ADR_1ROW);
	sendByte(False, False, DISPLAY_ON |
						   DISPLAY_CURSOR_POS_ON);
}

void setCursorPos(enum LCDRow row, uint8_t pos){
	uint8_t cursorPos = pos;
	switch (row) {
		case FIRST_ROW:
			cursorPos = pos | SET_DDRAM_ADR_1ROW;
			break;
		case SECOND_ROW:
			cursorPos = pos | SET_DDRAM_ADR_2ROW;
			break;
	}
	if(row == 1){
		cursorPos = pos | SET_DDRAM_ADR_2ROW;
	}
	sendByte(False, False, SET_DDRAM_ADR | cursorPos);
}

void disableCursor(){
	sendByte(False, False, DISPLAY_ON |
						   DISPLAY_CURSOR_POS_OFF);
	sendByte(False, False, SET_DDRAM_ADR |
							   SET_DDRAM_ADR_1ROW);
}

void sendByte (enum Bool isData, enum Bool upperBitsOnly, uint8_t byte){
	HAL_setPin(pinInterface.ePin, 0);
	if(isData == True) HAL_setPin(pinInterface.rsPin, 1);
	else HAL_setPin(pinInterface.rsPin, 0);
	sendNibble(byte);
	if(upperBitsOnly == False) sendNibble(byte<<4);
	clearOutputPins();
}

void sendNibble(uint8_t nibble){
	int i;
	for(i = 0; i < NIBBLE_SIZE; i++){
		HAL_setPin(pinInterface.dataPins[i], BIT_ON_POS(nibble, (i + NIBBLE_SIZE)));
	}
	HAL_setPin(pinInterface.ePin, 1);
	HAL_Delay(ENABLE_DELAY);
	HAL_setPin(pinInterface.ePin, 0);
	HAL_Delay(ENABLE_DELAY);
}

void clearOutputPins(){
	int i;
	for(i = 0; i < NIBBLE_SIZE; i++){
		HAL_setPin(pinInterface.dataPins[i], 0);
	}
	HAL_setPin(pinInterface.rsPin, 0);
}
