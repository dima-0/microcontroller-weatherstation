#ifndef __LCD_DISPLAY_H
#define __LCD_DISPLAY_H

/**
 * HEADER
 *
 * Modul zur Kommunikation mit einem 16x2 LCD-Display.
 * @Author: Dmitrij Bauer, Tim Wegner
 */

#include "utils.h"
#include "main.h"


/**********öffentliche Defines**********/

/**
 * Zeilen auf dem Display.
 */
enum LCDRow{FIRST_ROW, SECOND_ROW};

#define DISPLAY_ROW_LENGTH 16		//Anzahl der Zeichen, die auf dem Display in eine Zeile passen.


/**********Deklaration von öffentlichen Funktionen**********/

/**
 * Überträgt einen String an das LCD-Display, welches
 * dargestellt werden soll.
 * Der String sollte am besten die maximale Länge von 17 haben, weil
 * das Display maximal nur 16 Zeichen darstellen kann. Falls der
 * String länger sein sollte, werden alle restlichen Zeichen
 * ignoriert.
 * @param row		Zeile, auf der String angezeigt werden soll:
 * 					FIRST_ROW: 		1. Zeile
 * 					SECOND_ROW:  2. Zeile
 * @param string	String, welches angezeigt werden soll (Länge: <= 17).
 */
void sendString(enum LCDRow row, char* string);

/**
 * Initialisiert das LCD-Display.
 * @param pinInterface	Pin-Schnittstelle, welche für die Kommunikation mit dem
 * 						LCD-Display verwendet werden soll (!=NULL).
 */
void initLcdDisplay();

/**
 * Entfernt den Inhalt der beiden Zeilen auf dem Display.
 */
void clearDisplay();

/**
 * Aktiviert den Cursor (auf dem Display) auf der ersten Position der 1. Zeile.
 */
void enableCursor();

/*
 * Deaktiviert den Cursor (auf dem Display).
 */
void disableCursor();

/**
 * Setzt den Cursor (auf dem Display) auf die gegebene Position.
 * @param row	Zeile, auf der Cursor platziert werden soll:
 * 				FIRST_ROW: 1. Zeile
 * 				SECOND_ROW: 2. Zeile
 */
void setCursorPos(enum LCDRow row, uint8_t pos);

/**
 * Überträgt ein Byte an das LCD-Display.
 * @param isData			True, falls das Byte Daten beinhaltet. False, falls das Byte eine Anweisung ist.
 * @param upperBitsOnly		True, falls nur die oberen 4 Bit übertragen werden sollen. False andernfalls.
 * @param byte				Byte, welches übertragen werden soll.
 */
void sendByte(enum Bool isData, enum Bool upperBitsOnly, uint8_t byte);


#endif

