#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/**
 * HEADER
 *
 * Modul zur Verwaltung des internen Zustandes der Messstation.
 * @Author: Dmitrij Bauer
 */

#include "lcdDisplay.h"
#include "time.h"
#include "utils.h"


/**********�ffentliche Defines**********/

/**
 * Tasten zur Steuerung der Messstation.
 */
enum Button{UP, DOWN, LEFT, RIGHT};

/**
 *	M�gliche Zust�nde:
 *	- SET_TIME: 	Einstellung der Zeit
 *	- TIME: 		nur Uhrzeit
 *	- TIME_TEMP:	Uhrzeit/Temperatur
 *	- HUMID_TEMP:	Luftfeuchtigkeit/Temperatur
 *	- MULTI_STATE:  abwechselnd Uhrzeit(1) und Luftfeuchtigkeit/Temperatur(2)
 *					(Reihenfolge: 1->2->1)
 */
enum State{SET_TIME=0, TIME=1, TIME_TEMP=2, HUMIDITY_TEMP=3, MULTI_STATE=4};

/**********Deklaration der �ffentlichen Datentypen**********/

/**
 * Datenstruktur zur Verwaltung von einem Timer-Event.
 * Setzt voraus, dass der Timer genau einmal in der Sekunde
 * die Funktion onTimerUpdate() aufruft.
 */
typedef struct updateevent_{
	enum Bool ignore;					//True, falls das Event ignoriert werden soll. False andernfalls.
	enum Bool ready;					//True, falls das Update ausgel�st werden soll. False andernfalls.
	int timerCount;						//Anzahl der Sekunden seit dem letzten Event (>=0).
	int maxTimerCount;					//Update-Intervall in Sekunden (>=1).
}UpdateEvent;

/**
 * Datenstruktur zur Verwaltung der UpdateEvents.
 * Die Funktion initEventHandler() muss vor der Verwendung
 * einmal aufgerufen werden.
 */
typedef struct updateeventhandler_{
	enum Bool updateFlag;				//Ist True, falls das 'ready'-Flag bei mind. einem UpdateEvent auf True gesetzt ist.
	struct updateevent_ setTime;		//Best�tigung bei der Einstellung der Zeit.
	struct updateevent_ time;			//Aktualisierung der Uhrzeit und Ausgabe auf dem Display.
	struct updateevent_ temp;			//Aktualisierung der Temperatur und Ausgabe auf dem Display.
	struct updateevent_ humidity;		//Aktualisierung der Luftfeuchtigkeit und Ausgabe auf dem Display.
	struct updateevent_ multiState;		//Wechsel der Ansicht.
}UpdateEventHandler;

/**
 * Datenstruktur zur Verwaltung der Tastendruck-Events.
 * Mit der Funktion onButtonPressed(Button) kann ein
 * Flag bei entsprechender Taste gesetzt werden.
 */
typedef struct buttonhandler_{
	enum Bool btnPressedFlag;			//Ist True, falls mind. eines der anderen Flags auf True gesetzt ist.
	enum Bool btnUP;					//True, falls ein Tastendruck bei 'UP' registriert wurde. False andernfalls.
	enum Bool btnDOWN;					//True, falls ein Tastendruck bei 'DOWN' registriert wurde. False andernfalls.
	enum Bool btnLEFT;					//True, falls ein Tastendruck bei 'LEFT' registriert wurde. False andernfalls.
	enum Bool btnRIGHT;					//True, falls ein Tastendruck bei 'RIGHT' registriert wurde. False andernfalls.
}ButtonHandler;

/**
 * Datenstruktur zur Verwaltung des internen Zustandes der Messstation.
 * Die Funktion initController() muss vor der Verwendung einmal
 * aufgerufen werden.
 * Durch den Aufruf der Funktion update() kann der interne Zustand
 * der Messstation aktualisiert werden.
 */
typedef struct controller_{
	enum State currentState;			//Zustand, der zurzeit aktiv ist.
	enum TimePos cursorPos;				//Position des Cursors (nur relevant, wenn im Zustand SET_TIME).
	enum State multiState;				//Ansicht(Zustand), der zurzeit aktiv ist (nur relevant, wenn im Zustand MULTI_STATE).
	struct time_ currentTime;			//Aktuelle Uhrzeit.
	float temp;							//Aktuelle Temperatur.
	uint8_t humidity;							//Aktuelle Luftfeuchtigkeit.
	char firstRowBuffer[DISPLAY_ROW_LENGTH + 1];	//Ausgabe-Puffer (1. Display-Zeile).
	char secondRowBuffer[DISPLAY_ROW_LENGTH + 1];	//Ausgabe-Puffer (2. Display-Zeile).
	char timeStrBuffer[TIME_STRING_LEN + 1];		//Puffer f�r den String, der die Uhrzeit enth�lt.
	struct updateeventhandler_ eventHandler;		//Datenstruktur zur Verwaltung der UpdateEvents.
	struct buttonhandler_ btnHandler;				//Datestruktur zur Verwaltung der Tastendr�cke.
}Controller;


/**********Deklaration von �ffentlichen Funktionen**********/

/**
 * Initialisiert die Datenstruktur Controller, der den internen Zustand
 * der Messstation beinhalten soll.
 * Liest bei erster Ausf�hrung die Werte f�r Temperatur und Luftfeuchtigkeit aus und
 * initialisiert die Uhrzeit.
 * Dar�ber hinaus wird die Datenstruktur UpdateEventHandler initialisiert und
 * der Timer(TIM6), der jede Sekunde die internen Z�hler erh�hen soll, gestartet.
 *
 */
void initController(Controller* controller);

/**
 * Aktualisiert den Zustand der Messstation, falls ein Tastendruck
 * oder ein UpdateEvent registriert wurde.
 */
void update(Controller* controller);

/**
 * Setzt im ButtonHandler das Flag f�r die registrierte Taste.
 * @param pressedButton	Taste, die gedr�ckt wurde und jetzt im ButtonHandler
 * 						registriert werden soll.
 */
void onButtonPressed(ButtonHandler* btnHandler, enum Button pressedButton);

/**
 * Aktualisiert die internene Sekunden-Z�hler der UpdateEvents und setzt
 * 'ready'-Flags, falls der Z�hler den maximalen Wert erreicht hat.
 * Diese Funktion muss genau einmal in der Sekunde aufgerufen werden.
 */
void onTimerUpdate(Controller* controller);

#endif

