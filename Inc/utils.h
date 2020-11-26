#ifndef __UTILS_H
#define __UTILS_H

/**
 * Author: Dmitrij Bauer
 */

/**********öffentliche Makros**********/

/**
 * Modulo-Operation, die auch für negative Werte
 * einen korrekten Rest liefert.
 * ------
 * Quelle: https://www.lemoda.net/c/modulo-operator/
 * ------
 * @param x		erstes Argument.
 * @param y 	zweites Argument (y>0).
 * @return 		Rest der Division von x und y.
 */
#define MOD(x, y)(((x%y)+y)%y)

/**
 * Liefert das n-te Bit des Bytes.
 * @param byte	Byte, welches ausgelesen werden soll.
 * @param pos	Bit-Position im Byte, die ausgelesen werden soll (pos>=0).
 * @return		das Bit aus dem Byte 'byte' auf Position 'pos'.
 */
#define BIT_ON_POS(byte, pos)(((byte & (1 << pos)) >> pos))

/**
 * Berechnet den Anteil von n in max (in Prozent).
 * @param n		(0<=n<=1).
 * @param max
 */
#define PERCENT(n, max)((uint8_t)(((float)n / max) * 100))

/**********öffentliche Defines**********/

enum Bool{True = 1, False = 0};

#endif
