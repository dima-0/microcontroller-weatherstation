
/**
 * HEADER
 *
 * Modul zur Erzeugung/Bearbeitung der Uhrzeit.
 * @Author: Dmitrij Bauer
 */

#include "time.h"
#include "controller.h"


/**********Definition von Funktionen**********/

void tick(Time* time){
	timeInc(S2, 1, time);
	if (time->s2 == 0) {
		timeInc(S1, 1, time);
		if (time->s1 == 0) {
			timeInc(M2, 1, time);
			if (time->m2 == 0) {
				timeInc(M1, 1, time);
				if (time->m1 == 0) {
					timeInc(H2, 1, time);
					if (time->h2 == 0) {
						timeInc(H1, 1, time);
					}
				}
			}
		}
	}
}

void timeInc(enum TimePos pos, int n, Time* time){
	switch(pos){
	    case H1:
	      time->h1 = MOD((time->h1 + n), 3);
	      if(time->h1 == 2) time->h2 = MOD((time->h2), 4);
	      break;
	    case H2:
	      time->h2 = MOD((time->h2 + n), (time->h1 >= 2 ? 4 : 10));
	      break;
	    case M1:
	      time->m1 = MOD((time->m1 + n) , 6);
	      break;
	    case M2:
	      time->m2 = MOD((time->m2 + n) , 10);
	      break;
	    case S1:
	      time->s1 = MOD((time->s1 + n) , 6);
	      break;
	    case S2:
	      time->s2 = MOD((time->s2 + n) , 10);
	      break;
	  }
}

void toString(Time* time, char* buffer){
	const static char template[] = "%d%d:%d%d:%d%d";
	snprintf(buffer, TIME_STRING_LEN + 1, template,
			time->h1, time->h2,
			time->m1,time->m2,
			time->s1, time->s2);
}
