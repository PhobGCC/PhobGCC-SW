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

typedef bool(*ExtrasToggleFn)(IntOrFloat config[]);//Used for toggling extras
typedef void(*ExtrasConfigFn)(IntOrFloat config[], Cardinals dpad);//Used for configuring extras with the Dpad

struct ExtrasFunctions{
	ExtrasToggleFn toggleFn;
	ExtrasConfigFn configFn;
};

static ExtrasFunctions extrasFunctions[EXTRAS_SIZE] = {NULL};

void extrasConfigAssign(ExtrasSlot slot, ExtrasToggleFn toggleFn, ExtrasConfigFn configFn){
	switch(slot){
		case EXTRAS_UP:
#ifdef ARDUINO
			Serial.println("Extra: Setting configuration to Up...");
#endif //ARDUINO
			break;
		case EXTRAS_DOWN:
#ifdef ARDUINO
			Serial.println("Extra: Setting configuration to Down...");
#endif //ARDUINO
			break;
		case EXTRAS_LEFT:
#ifdef ARDUINO
			Serial.println("Extra: Setting configuration to Left...");
#endif //ARDUINO
			break;
		case EXTRAS_RIGHT:
#ifdef ARDUINO
			Serial.println("Extra: Setting configuration to Right...");
#endif //ARDUINO
			break;
		case EXTRAS_UNSET:
#ifdef ARDUINO
			Serial.println("Extra: Configuration slot not set, feature will be inaccessible.");
#endif //ARDUINO
			return;
		default:
#ifdef ARDUINO
			Serial.println("Extra: Invalid configuration slot requested, feature will be inaccessible.");
#endif //ARDUINO
			return;
	}
	ExtrasFunctions &fns = extrasFunctions[slot];
	if(fns.toggleFn || fns.configFn) {
#ifdef ARDUINO
		Serial.println("Extra: Warning! Configuration slot was already in use, previous feature will be inaccessible.");
#endif //ARDUINO
	}
	fns.toggleFn = toggleFn;
	fns.configFn = configFn;
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
#ifdef ARDUINO
	Serial.println("Extra: Enabling ESS functionality...");
#endif //ARDUINO
	extrasConfigAssign(ess::extrasEssConfigSlot, ess::toggle, NULL);
#endif

}

#endif //EXTRAS_H
