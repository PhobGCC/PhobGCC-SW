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

#ifdef EXTRAS_ESS
#include "ess.h"
#endif

typedef void(*ExtrasConfigFn)(IntOrFloat config[], Buttons&);//Used for making configuration changes
static ExtrasConfigFn extrasConfigureFunctions[EXTRAS_SIZE] = {NULL};

void extraConfigAssignPrint(ExtrasSlot slot, ExtrasConfigFn configFn){
	if(configFn) {
		Serial.println("Extra: Warning! Configuration slot was already in use, previous feature will be inaccessible.");
	}
	switch(slot){
		case EXTRAS_UP:
			Serial.println("Extra: Configuration assigned to Up.");
			break;
		case EXTRAS_DOWN:
			Serial.println("Extra: Configuration assigned to Down.");
			break;
		case EXTRAS_LEFT:
			Serial.println("Extra: Configuration assigned to Left.");
			break;
		case EXTRAS_RIGHT:
			Serial.println("Extra: Configuration assigned to Right.");
			break;
		case EXTRAS_UNSET:
			Serial.println("Extra: Configuration slot not set, feature will be inaccessible.");
			break;
		default:
			break;
	}
}

void extrasInit() {
	/*----------------------------------------------------------
	 * Configuration slots for extras, the available options are:
	 *	- EXTRAS_UP
	 *	- EXTRAS_DOWN
	 *	- EXTRAS_LEFT
	 *	- EXTRAS_RIGHT
	 * Make sure to set the slot from EXTRAS_UNSET to one of the
	 * above for the extras below if applicable!
	 */
#ifdef EXTRAS_ESS
	//-----------------------------------------
	ess::extrasEssConfigSlot = EXTRAS_UNSET;
	//-----------------------------------------
	Serial.println("Extra: Enabling ESS functionality...");
	extraConfigAssignPrint(ess::extrasEssConfigSlot, extrasConfigureFunctions[ess::extrasEssConfigSlot]);
	if (ess::extrasEssConfigSlot != EXTRAS_UNSET) {
		extrasConfigureFunctions[ess::extrasEssConfigSlot] = ess::configure;
	}
#endif

}

#endif //EXTRAS_H
