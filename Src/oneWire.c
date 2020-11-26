
/**
 * HEADER
 *
 * Modul zur Kommunikation mit einem Temperatursensor
 * des Typs D18S20 mittels OneWire-Bus.
 * @Author: Tim Wegner
 */

#include <oneWire.h>

/**********private Defines**********/
//General Delays
#define SLOT_LENGTH 100
#define RECOVERY 3

//WriteBit
#define WRITE_1_DELAY 5

//RESET
#define RESET_PULLDOWN 500
#define DELAY_BEFORE_SAMPLE 60
#define DELAY_END_RESET 440

//READ
#define DELAY_READ_BUS_LOW 1
#define DELAY_BEFORE_READ 15
#define DELAY_END_READ 100

//Commands
#define CONVERT_T 0x44
#define READ_SRCATCHPAD 0xBE
#define SKIP_ROM 0xCC

//ERROR
#define NO_DEVICE 420

//CONVERT
#define CONVERT_DELAY 50
//Sratchpad-Positionen
#define LSB 0
#define MSB 1
#define COUNT_REMAIN 6
#define COUNT_PER_C 7

/**********Deklaration von privaten Funktionen**********/

/**
 * Sendet den Befehl "convertT" an den am OneWire-Bus angeschlossenen
 * Temperatursensor und wartet anschließend, bis dieser Befehl bearbeitet wurde.
 * ------
 * Sollte kein Temperatursensor an dem OneWire-Bus angeschlossen sein,
 * dann wird eine Debug-Ausgabe generiert und kein Befehl gesendet.
 * -----
 * Diese Methode funktioniert nur, wenn maximal ein einziges Gerät an den
 * OneWire-Bus angeschlossen ist.
 */
static void convertT();

/**
 * Liefert die aktuelle Temperatur des Temperatursensors.
 * ------
 * Sendet den Befehl "readScratchpad" an dem am OneWire-Bus angeschlossenen
 * Temperatursensor und ließt anschließend sein Scratchpad aus.
 * ------
 * Sollte kein Temperatursensor an dem OneWire-Bus angeschlossen sein,
 * wird eine Debug-Ausgabe generiert und die Temperatur 420*C zurückgegeben.
 * -----
 * Diese Methode funktioniert nur, wenn maximal ein einziges Gerät an den
 * OneWire-Bus angeschlossen ist.
 * -----
 * Quelle:
 * https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/162.html
 * Bei der Erstellung dieser Funktion diente die Funktion "Read_Temperature()" unter
 * "Reading Device Temperature" als Hilfestellung für das Umwandeln der Temperatur.
 */
static float readScratchpad();

/**
 * Liefert den Wert des Reset-Puls von dem am OneWire-Bus angeschlossenen
 * Temperatursensor, nachdem ein "reset"-Befehl gesendet wurde.
 */
static uint8_t reset();

/**
 * Sendet den Befehl "skipROM" an den am OneWire-Bus angeschlossenen
 * Temperatursensor.
 */
static void skipROM();

/**
 * Sendet einen beliebgen Befehl, welcher in 8 Bit codiert ist,
 * an den am OneWire-Bus angeschlossenen Temperatursensor.
 * @param command Der Befehl, welcher gesendet werden soll.
 */
static void sendCommand(uint8_t command);

/**
 * Sendet einen Bit an den am OneWire-Bus angeschlossenen
 * Temperatursensor.
 * @param writeOne Der Wert, welcher gesendet werden soll.
 * 				0, falls eine 0 gesendet werden soll und
 * 				1, falls eine 1 gesendet werden soll.
 */
static void writeBit(uint8_t writeOne);

/**
 * Liefert den Wert eines vom Temperatursensor, welcher
 * an dem OneWire-Bus angeschlossen ist, gesendeten Bits.
 */
static uint8_t readBit();

float getTemp(){
	convertT();
	return readScratchpad();
}

static void convertT(){
	uint8_t resetPuls = reset();
	if(resetPuls == 1){
		UART_log("No Device\n");
	}else{
		HAL_pinToOutput(pinInterface.tempPin);
		skipROM();
		sendCommand(CONVERT_T);
		HAL_Delay(CONVERT_DELAY);
	}

}

static float readScratchpad(){
	uint8_t resetPuls = reset();
	float temp = NO_DEVICE;
	if(resetPuls == 1){
		UART_log("No Device\n");
	}else{
		HAL_pinToOutput(pinInterface.tempPin);
		skipROM();
		sendCommand(READ_SRCATCHPAD);

		uint8_t sratchpad[9] = {0};
		for(int i = 0; i < 9; i++){
			uint8_t result = 0;
			for(int j = 0; j<8; j++){
				if (readBit() == 1) result |= 0x01 << j;
				waitUs(RECOVERY);
			}
			sratchpad[i] = result;
		}
		if(sratchpad[MSB] == 0xff){
			uint8_t byte = sratchpad[LSB];
			byte = (~byte) + 1;
			temp = byte >> 1;
			temp *= -1.0;
		}else{
			temp = sratchpad[LSB] >> 1;
		}
		float tempFraction = (sratchpad[COUNT_PER_C] - sratchpad[COUNT_REMAIN]) / (float)sratchpad[COUNT_PER_C];
		temp = temp - 0.25 + tempFraction;
	}
	return temp;
}

static uint8_t reset(){
	HAL_pinToOutput(pinInterface.tempPin);

	HAL_setPin(pinInterface.tempPin,0);
	waitUs(RESET_PULLDOWN);
	HAL_setPin(pinInterface.tempPin,1);
	waitUs(DELAY_BEFORE_SAMPLE);

	HAL_pinToInput(pinInterface.tempPin);
	uint8_t deviceIsPresent = HAL_readPin(pinInterface.tempPin);

	waitUs(DELAY_END_RESET);
	return deviceIsPresent;
}

static void skipROM(){
	sendCommand(SKIP_ROM);
}

static void sendCommand(uint8_t command){
	uint8_t i;
	uint8_t temp;
	for(i = 0; i < 8; i++){
		temp = command >>i;
		temp &= 0x01;
		if(temp == 1){
			writeBit(1);
		}else{
			writeBit(0);
		}
	}
	waitUs(RECOVERY);
}

static void writeBit(uint8_t writeOne){
	uint8_t delayTime = writeOne >= 1? WRITE_1_DELAY : SLOT_LENGTH;
	uint8_t delayEnd = SLOT_LENGTH - delayTime;

	HAL_setPin(pinInterface.tempPin,0);
	waitUs(delayTime);
	HAL_setPin(pinInterface.tempPin,1);
	waitUs(delayEnd);
}

static uint8_t readBit(){
	HAL_pinToOutput(pinInterface.tempPin);
	HAL_setPin(pinInterface.tempPin,0);
	waitUs(DELAY_READ_BUS_LOW);
	HAL_pinToInput(pinInterface.tempPin);
	waitUs(DELAY_BEFORE_READ);
	uint8_t readBit = HAL_readPin(pinInterface.tempPin);
	waitUs(DELAY_END_READ);
	return readBit;
}
