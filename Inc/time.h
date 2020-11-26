#ifndef __TIME_H
#define __TIME_H

/**
 * HEADER
 *
 * Modul zur Erzeugung/Bearbeitung der Uhrzeit.
 * @Author: Dmitrij Bauer
 */

#include <stdio.h>
#include "utils.h"

/**********öffentliche Defines**********/

/**
 * Ziffer-Position im Zeitformat: H1H2:M1M2:S1S2
 */
enum TimePos{H1=0, H2=1, M1=2, M2=3, S1=4, S2=5};

/**Anzahl der Positionen im Zeitformat H1H2:M1M2:S1S2 */
#define TIME_SIZE 6

/**Länge des String, der die Uhrzeit im Format H1H2:M1M2:S1S2 enthält.*/
#define TIME_STRING_LEN 8

/**********öffentliche Makros**********/

/**
 * Bewegt den Cursor um eine Position nach rechts.
 * Inkrementiert die gegebene Position um 1.
 */
#define INCR(pos)(MOD(((int8_t)pos+1), TIME_SIZE))

/**
 * Bewegt den Cursor um eine Position nach links.
 * Dekrementiert die gegebene Position um 1.
 */
#define DECR(pos)(MOD(((int8_t)pos-1), TIME_SIZE))

/**********Deklaration der öffentlichen Datentypen**********/

/**
 * Datenstruktur zur Verwaltung der Uhrzeit.
 */
typedef struct time_{
  int8_t h1;
  int8_t h2;
  int8_t m1;
  int8_t m2;
  int8_t s1;
  int8_t s2;
}Time;


/**********Deklaration von öffentlichen Funktionen**********/

/**
 * Erhöht die gegebene Zeit um eine Sekunde.
 * @param time	Pointer der Uhrzeit, die verändert werden soll (!=NULL).
 */
void tick(Time* time);

/*
 * Inkrementiert die gegebene Zeit an der Stelle pos um n.
 * @param pos	Position der Ziffer, die erhöht werden soll (0<=pos<=5).
 * @param n		Wert, um den der aktuelle Werte erhöht werden soll.
 * @param time	Pointer der Uhrzeit, die verändert werden soll (!=NULL).
 */
void timeInc(enum TimePos pos, int n, Time* time);

/**
 * @param time		Uhrzeit, die als String ausgegeben werden soll (!=NULL).
 * @param buffer	Puffer für den String, der die Uhrzeit enthalten soll (!=NULL).
 */
void toString(Time* time, char* buffer);

#endif
