#include "oneWire.h"
#include "controller.h"
#include "main.h"
#include <stdio.h>

/**
 * HEADER
 *
 * Modul zur Verwaltung des internen Zustandes der Messstation.
 * @Author: Dmitrij Bauer
 */

/**********private Defines**********/
//Zeit-Intervalle (in Sekunden)
#define UPDATE_TIME 1			//Aktualisierung der Uhrzeit.
#define UPDATE_TEMP 5			//Auslesen der Temperatur.
#define UPDATE_HUMID 5			//Auslesen der Luftfeuchtigkeit.
#define CONFIRM_SET_TIME 10		//Wartezeit zum bestätigen der Uhrzeit (im Zustand: SET_TIME).
#define SWITCH_DISPLAY 30		//Wartezeit zum wechseln der Ansicht (im Zustand: MULTI_STATE).

/**********Deklaration von privaten Funktionen**********/

/**
 * Inkrementiert den Sekunden-Zähler eines Events
 * und setzt das 'ready'-Flag, falls der Zähler den
 * maximalen Wert errreicht hat.
 * @param event		Zeiger des UpdateEvents, dessen Zähler erhöht werden soll (!=NULL).
 */
static void incrTimerCounter(UpdateEvent* event);

/**
 * Initialisiert die Datenstruktur zur Verwaltung von Events.
 * @param eventHandler		Zeiger des EventHandlers, der initialisiert werden (!=NULL).
 */
static void initEventHandler(UpdateEventHandler* eventHandler);

/**
 * Funktion des Zustandes SET_TIME.
 * Reagiert auf die Events vom ButtonHandler und EventHandler.
 * ------
 * Aktionen beim Tastendruck:
 * 	- 'UP': 	inkrementiert die Zahl auf der aktuellen Cursor-Position um 1.
 * 	- 'DOWN':	dekrementiert die Zahl auf der aktuellen Cursor-Position um 1.
 * 	- 'LEFT':	bewegt den Cursor auf die vorherige Position.
 * 	- 'RIGHT':	bewegt den Cursor auf die nächstmögliche Position.
 * Aktionen beim UpdateEvent:
 * 	- Initialisierung des Zustandes: TIME.
 *
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void stateSetTime(Controller* controller);

/**
 * Funktion des Zustandes TIME.
 * Reagiert auf die Events vom ButtonHandler und EventHandler.
 * ------
 * Aktionen beim Tastendruck:
 * 	- 'UP': 	Initialisierung des Zustandes: SET_TIME.
 * 	- 'LEFT':	Initialisierung des Zustandes: MULTI_STATE.
 * 	- 'RIGHT':	Initialisierung des Zustandes: TIME_TEMP.
 * Aktionen beim UpdateEvent:
 * 	- Ausgabe der neuen Uhrzeit auf dem Display.

 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void stateTime(Controller* controller);

/**
 * Funktion des Zustandes TIME_TEMP.
 * Reagiert auf die Events vom ButtonHandler und EventHandler.
 * ------
 * Aktionen beim UpdateEvent:
 * 	- 'UP': 	Initialisierung des Zustandes: SET_TIME.
 * 	- 'LEFT':	Initialisierung des Zustandes: TIME.
 * 	- 'RIGHT':	Initialisierung des Zustandes: HUMIDITY_TEMP.
 * Aktionen beim TimerUpdate:
 * 	- Ausgabe der neuen Uhrzeit auf dem Display.
 * 	- Ausgabe der neuen Temperatur auf dem Display.

 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void stateTimeTemp(Controller* controller);

/**
 * Funktion des Zustandes TIME_TEMP.
 * Reagiert auf die Events vom ButtonHandler und EventHandler.
 * ------
 * Aktionen beim Tastendruck:
 * 	- 'UP': 	Initialisierung des Zustandes: SET_TIME.
 * 	- 'LEFT':	Initialisierung des Zustandes: TEMP_TIME.
 * 	- 'RIGHT':	Initialisierung des Zustandes: MULTI_STATE.
 * Aktionen beim UpdateEvent:
 * 	- Ausgabe der neuen Luftfeuchtigkeit auf dem Display.
 * 	- Ausgabe der neuen Temperatur auf dem Display.

 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void stateHumidTemp(Controller* controller);

/**
 * Funktion des Zustandes MULTI_STATE.
 * Reagiert auf die Events vom ButtonHandler und EventHandler.
 * In diesem Zustand werden die Zustände TIME und HUMIDITY_TIME
 * peridiodisch geschaltet.
 * ------
 * Aktionen beim Tastendruck:
 * 	- 'UP': 	Initialisierung des Zustandes: SET_TIME.
 * 	- 'LEFT':	Initialisierung des Zustandes: HUMIDITY_TEMP.
 * 	- 'RIGHT':	Initialisierung des Zustandes: TIME.
 * Aktionen beim UpdateEvent:
 * 	- Schalten in den nächsten Zustand (TIME->HUMIDITY_TEMP
 * 	oder HUMIDITY_TEMP->TIME).
 * 	- Ausgabe auf der neuen Daten auf dem Display. Falls TIME aktiv, dann
 * 		Uhrzeit. Sonst Luftfeuchtigkeit und Temperatur.

 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void stateMultiState(Controller* controller);

/**
 * Setzt alle Flags des ButtonHandlers zurück.
 * @param btHandler		Zeiger des ButtonHandlers, der zurückgesetzt werden soll (!=NULL).
 */
static void resetButtonHandler(ButtonHandler* btnHandler);

/**
 * Setzt das updateFlag und bei allen UpadteEvents das 'ready'-Flag zurück.
 * @param eventHandler		Zeiger des EventHandlers, der zurückgesetzt werden soll (!=NULL).
 */
static void resetUpdateHandler(UpdateEventHandler* eventHandler);

/**
 * Initialisiert den gegebenen Zustand.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 * @param state	Zustand, der initialisiert werden soll.
 */
static void initState(Controller* controller, enum State state);

/**
 * Aktualisiert jeweils die internen Werte für die
 * Uhrzeit, Temperatur oder die Luftfeuchtigkeit
 * wenn das 'ready'-Flag beim entsprechenden
 * UpdateEvent gesetzt ist.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void updateValues(Controller* controller);

/**
 * Gibt die aktuelle Uhrzeit auf dem Display aus.
 * Die Uhrzeit erscheint auf dem Display in der 1. Zeile
 * im Format 'HH:MM:SS'.
 * Am Ende der Zeile wird der aktuelle Modus angezeigt.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void printTime(Controller* controller);

/**
 * Gibt die aktuelle Temperatur auf dem Display aus.
 * Die Temperatur erscheint auf dem Display in der 2. Zeile
 * im Format 'Temp: <T>°C'.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void printTemp(Controller* controller);

/**
 * Gibt die aktuelle Luftfeuchtigkeit (in Prozent) auf dem Display aus.
 * Die Luftfeuchtigkeit erscheint auf dem Display in der 1. Zeile
 * im Format 'Humidity: <H>%'.
 * Am Ende der Zeile wird der aktuelle Modus angezeigt.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void printHumidity(Controller* controller);

/**
 * Gibt die aktuell eingestellte Uhrzeit auf dem Display aus.
 * Die eingestellte Uhrzeit erscheint auf dem Display in der 2. Zeile
 * im Format 'HH:MM:SS'.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void printSetTime(Controller* controller);

/**
 * Gibt den beschreibenden Text 'set time' auf dem Display in
 * der 1. Zeile aus.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static void printSetTimeDescr(Controller* controller);

/**
 * Liefert die aktuelle Position des Cursors auf dem Display.
 * Beispiel: Uhrzeit-Format 'H1H2:M1M2:S1S2'
 * Der interne Curser 'cursorPos' wird auf Position M1(2) verschoben,
 * der Cursor auf dem Display würde somit somit auf ':' landen.
 * Um das zu verhindern wird die Position um 1 inkrementiert.
 * @param controller	Zeiger auf den Controller, der den aktuellen Programm-Zustand enthält (!=NULL).
 */
static uint8_t getCurserPos(Controller* controller);


/**********Definition von Funktionen**********/

void initController(Controller* controller){
	Time currentTime = {0, 0, 0, 0, 0, 0};
	controller->currentTime = currentTime;
	controller->cursorPos = H1;
	controller->temp = getTemp();
	controller->humidity = getRelADCValue();
	UpdateEventHandler eventHandler = {0};
	initEventHandler(&eventHandler);
	controller->eventHandler = eventHandler;
	initState(controller, SET_TIME);
	initTimer();
}

static void initEventHandler(UpdateEventHandler* eventHandler){
	UpdateEvent setTime = {False, False, 0, CONFIRM_SET_TIME};
	UpdateEvent time = {False, False, 0, UPDATE_TIME};
	UpdateEvent temp = {False, False, 0, UPDATE_TEMP};
	UpdateEvent humid = {False, False, 0, UPDATE_HUMID};
	UpdateEvent multiState = {False, False, 0, SWITCH_DISPLAY};
	eventHandler->setTime = setTime;
	eventHandler->time = time;
	eventHandler->temp = temp;
	eventHandler->humidity = humid;
	eventHandler->multiState = multiState;
}

void update(Controller* controller){
	if (controller->btnHandler.btnPressedFlag == True || controller->eventHandler.updateFlag == True) {
		updateValues(controller);
		switch (controller->currentState) {
		case SET_TIME:
			stateSetTime(controller);
			break;
		case TIME:
			stateTime(controller);
			break;
		case TIME_TEMP:
			stateTimeTemp(controller);
			break;
		case HUMIDITY_TEMP:
			stateHumidTemp(controller);
			break;
		case MULTI_STATE:
			stateMultiState(controller);
			break;
		}
		resetButtonHandler(&controller->btnHandler);
		resetUpdateHandler(&controller->eventHandler);
	}
}


static void initState(Controller* controller, enum State state){
	clearDisplay();
	switch (state) {
		case SET_TIME:
			controller->eventHandler.time.ignore = True;
			controller->currentState = state;
			controller->cursorPos = H1;
			printSetTimeDescr(controller);
			printSetTime(controller);
			enableCursor();
			setCursorPos(SECOND_ROW, getCurserPos(controller));
			break;
		case TIME:
			controller->currentState = state;
			controller->eventHandler.time.ignore = False;
			printTime(controller);
			break;
		case TIME_TEMP:
			controller->currentState = state;
			printTime(controller);
			printTemp(controller);
			break;
		case HUMIDITY_TEMP:
			controller->currentState = state;
			printHumidity(controller);
			printTemp(controller);
			break;
		case MULTI_STATE:
			controller->currentState = state;
			controller->multiState = TIME;
			controller->eventHandler.multiState.timerCount = 0;
			printTime(controller);
			break;
	}
}

static void incrTimerCounter(UpdateEvent* event){
	if(event->ignore == False){
		event->timerCount = (event->timerCount+1) % event->maxTimerCount;
		if(event->timerCount == 0) event->ready = True;
	}
}

void onTimerUpdate(Controller* controller){
	UpdateEventHandler* eventHandler = &controller->eventHandler;
	eventHandler->updateFlag = True;
	switch (controller->currentState) {
		case SET_TIME:
			incrTimerCounter(&eventHandler->setTime);
			break;
		case MULTI_STATE:
			incrTimerCounter(&eventHandler->multiState);
			break;
		default:
			break;
	}
	incrTimerCounter(&eventHandler->time);
	incrTimerCounter(&eventHandler->temp);
	incrTimerCounter(&eventHandler->humidity);
}

static void updateValues(Controller* controller){
	UpdateEventHandler* eventHandler = &controller->eventHandler;
	if(eventHandler->time.ignore == False && eventHandler->time.ready == True){
		tick(&controller->currentTime);
	}
	if(eventHandler->temp.ignore == False && eventHandler->temp.ready == True){
		controller->temp = getTemp();
	}
	if(eventHandler->humidity.ignore == False && eventHandler->humidity.ready == True){
		controller->humidity = getRelADCValue();
	}
}

static void stateTime(Controller* controller){
	if(controller->eventHandler.time.ready) printTime(controller);
	if(controller->btnHandler.btnPressedFlag == True){
		if(controller->btnHandler.btnUP == True) initState(controller, SET_TIME);
		if(controller->btnHandler.btnRIGHT == True) initState(controller, TIME_TEMP);
		if(controller->btnHandler.btnLEFT == True) initState(controller, MULTI_STATE);
	}
}

static void stateTimeTemp(Controller* controller){
	if(controller->eventHandler.time.ready) printTime(controller);
	if(controller->eventHandler.temp.ready) printTemp(controller);
	if(controller->btnHandler.btnPressedFlag == True){
		if(controller->btnHandler.btnUP == True) initState(controller, SET_TIME);
		if(controller->btnHandler.btnRIGHT == True) initState(controller, HUMIDITY_TEMP);
		if(controller->btnHandler.btnLEFT == True) initState(controller, TIME);
	}
}

static void stateHumidTemp(Controller* controller){
	if(controller->eventHandler.humidity.ready) printHumidity(controller);
	if(controller->eventHandler.temp.ready) printTemp(controller);
	if(controller->btnHandler.btnPressedFlag == True){
		if(controller->btnHandler.btnUP == True) initState(controller, SET_TIME);
		if(controller->btnHandler.btnRIGHT == True) initState(controller, MULTI_STATE);
		if(controller->btnHandler.btnLEFT == True) initState(controller, TIME_TEMP);
	}
}

static void stateSetTime(Controller* controller){
	if(controller->btnHandler.btnPressedFlag == True){
		controller->eventHandler.setTime.timerCount = 0;
		if(controller->btnHandler.btnUP == True){
			timeInc(controller->cursorPos, 1, &controller->currentTime);
			printSetTime(controller);
		}
		if(controller->btnHandler.btnDOWN == True){
			timeInc(controller->cursorPos, -1, &controller->currentTime);
			printSetTime(controller);
		}
		if(controller->btnHandler.btnRIGHT == True) controller->cursorPos = INCR(controller->cursorPos);
		if(controller->btnHandler.btnLEFT == True) controller->cursorPos = DECR(controller->cursorPos);
		setCursorPos(SECOND_ROW, getCurserPos(controller));
	}
	if(controller->eventHandler.setTime.ready == True){
		disableCursor();
		initState(controller, TIME);
	}
}

static uint8_t getCurserPos(Controller* controller){
	switch (controller->cursorPos) {
		case M1:
			return (uint8_t)M1 + 1;
		case M2:
			return (uint8_t)M2 + 1;
		case S1:
			return (uint8_t)S1 + 2;
		case S2:
			return (uint8_t)S2 + 2;
		default:
			return (uint8_t)controller->cursorPos;
	}
}

void onButtonPressed(ButtonHandler* btnHandler, enum Button buttonPressed){
	btnHandler->btnPressedFlag = True;
	switch (buttonPressed) {
		case UP:
			btnHandler->btnUP = True;
			break;
		case DOWN:
			btnHandler->btnDOWN = True;
			break;
		case LEFT:
			btnHandler->btnLEFT = True;
			break;
		case RIGHT:
			btnHandler->btnRIGHT = True;
			break;
		default:
			break;
	}
}

static void resetButtonHandler(ButtonHandler* btnHandler){
	btnHandler->btnPressedFlag = False;
	btnHandler->btnUP = False;
	btnHandler->btnDOWN = False;
	btnHandler->btnRIGHT = False;
	btnHandler->btnLEFT = False;
}

static void resetUpdateHandler(UpdateEventHandler* eventHandler){
	eventHandler->updateFlag = False;
	eventHandler->time.ready = False;
	eventHandler->temp.ready = False;
	eventHandler->humidity.ready = False;
	eventHandler->setTime.ready = False;
	eventHandler->multiState.ready = False;
}

static void printSetTimeDescr(Controller* controller){
	const static char template[] = "set time";
	snprintf(controller->firstRowBuffer, DISPLAY_ROW_LENGTH+1, template);
	sendString(FIRST_ROW, controller->firstRowBuffer);
}

static void printTime(Controller* controller){
	toString(&controller->currentTime, controller->timeStrBuffer);
	const static char template[] = "%s      M%d";
	snprintf(controller->firstRowBuffer, DISPLAY_ROW_LENGTH+1, template, controller->timeStrBuffer, controller->currentState);
	sendString(FIRST_ROW, controller->firstRowBuffer);
}

static void printTemp(Controller* controller){
	const static char template[] = "Temp:%6.1f%cC";
	snprintf(controller->secondRowBuffer, DISPLAY_ROW_LENGTH+1, template,
				controller->temp, 223);
	sendString(SECOND_ROW, controller->secondRowBuffer);
}

static void printSetTime(Controller* controller){
	toString(&controller->currentTime, controller->timeStrBuffer);
	const static char template[] = "%s";
	snprintf(controller->firstRowBuffer, DISPLAY_ROW_LENGTH+1, template, controller->timeStrBuffer);
	sendString(SECOND_ROW, controller->firstRowBuffer);
}

static void printHumidity(Controller* controller){
	const static char template[] = "Humidity:%3d%% M%d";
	snprintf(controller->firstRowBuffer, DISPLAY_ROW_LENGTH+1, template, controller->humidity, controller->currentState);
	sendString(FIRST_ROW, controller->firstRowBuffer);
}

static void stateMultiState(Controller* controller){
	switch (controller->multiState) {
		case TIME:
			if(controller->eventHandler.multiState.ready == True) {				//Prüfe ob die Ansicht gewechselt werden soll.
				controller->multiState = HUMIDITY_TEMP;
				clearDisplay();
				printHumidity(controller);
				printTemp(controller);
			}else if(controller->eventHandler.time.ready) printTime(controller);//Sonst prüfe ob die Werte ausgegeben werden sollen.
			break;
		case HUMIDITY_TEMP:
			if(controller->eventHandler.multiState.ready == True) {				//Prüfe ob die Ansicht gewechselt werden soll.
				controller->multiState = TIME;
				clearDisplay();
				printTime(controller);
			}else{																//Sonst prüfe ob die Werte ausgegeben werden sollen.
				if(controller->eventHandler.humidity.ready) printHumidity(controller);
				if(controller->eventHandler.temp.ready) printTemp(controller);
			}
			break;
		default:
			break;
	}
	if(controller->btnHandler.btnPressedFlag == True){
		if(controller->btnHandler.btnUP == True) initState(controller, SET_TIME);
		if(controller->btnHandler.btnRIGHT == True) initState(controller, TIME);
		if(controller->btnHandler.btnLEFT == True) initState(controller, HUMIDITY_TEMP);
	}
}

