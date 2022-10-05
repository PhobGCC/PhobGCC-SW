#ifndef EXTRAS_H
#define EXTRAS_H

/* Plugins and extra features outside the normal scope of
 * the PhobGCC can be found here. Uncomment each #define
 * to include the features they provide in a build then
 * set the configuration slot it uses (if it needs configuration)
 * 
 */
//------------------------------------------------------
//#define EXTRAS_ESS
//------------------------------------------------------

#include "ess.h"

typedef void(*ExtrasConfigFn)(IntOrFloat config[], Buttons&);//Used for making configuration changes
static ExtrasConfigFn extrasConfigureFunctions[EXTRAS_SIZE] = {NULL};

void extrasInit() {
	/*----------------------------------------------------------
	 * Configuration slots for extras, the available options are:
	 *	- EXTRAS_UP
	 *	- EXTRAS_DOWN
	 *	- EXTRAS_LEFT
	 *	- EXTRAS_RIGHT
	 */
	ess::extrasEssConfigSlot = EXTRAS_UNSET;
	//----------------------------------------------------------

#ifdef EXTRAS_ESS
	if (ess::extrasEssConfigSlot == EXTRAS_UNSET) {
		Serial.println("Extra: ESS configuration slot not set, aborting...");
	} else {
		extrasConfigureFunctions[ess::extrasEssConfigSlot] = ess::configure;
		Serial.println("Extra: ESS functionality enabled.");
	}
#endif

}

#endif //EXTRAS_H