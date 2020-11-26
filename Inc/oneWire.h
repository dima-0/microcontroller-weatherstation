#ifndef __ONEWIREMANAGER_H
#define __ONEWIREMANAGER_H

/**
 * HEADER
 *
 * Modul zur Kommunikation mit einem Temperatursensor
 * des Typs D18S20 mittels OneWire-Bus.
 * @Author: Tim Wegner
 */

#include "main.h"
#include "utils.h"

/**
 * Liefert die aktuelle Temperatur, welche der Temperatursensor bestimmt hat.
 * ------
 * Sollte kein Temperatursensor an dem OneWire-Bus angeschlossen sein,
 * wird eine Debug-Ausgabe generiert.
 * ------
 * Diese Methode funktioniert nur, wenn maximal ein einziges Gerät an den
 * OneWire-Bus angeschlossen ist.
 */
float getTemp();

#endif
