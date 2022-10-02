#ifndef EXTRAS_H
#define EXTRAS_H

/* Plugins and extra features outside the normal scope of
 * the PhobGCC can be found here. Uncomment each #define
 * to include the features they provide in a build.
 * ------------------------------------------------------
 */
#define EXTRAS_ESS
// ------------------------------------------------------

#include "../common/phobGCC.h"
#include "ess.h"

ExtrasConfig _extrasConfig {
	.essEnable = EXTRAS_ESS_DISABLED
};

//Currently supports 10 extras, can easily be changed however.
static const int _extrasMax = 10;
static int extrasRegistered = 0;
static Extra extras[_extrasMax];

void extrasRegister(Extra extra){
	if (extrasRegistered >= _extrasMax){
		Serial.println("Too many extras registered! Aborting...");
		return;
	}
	extras[extrasRegistered] = extra;
	extrasRegistered++;
}

//Common callback function types that should be used for the different ways we interact with extras
typedef bool(*ExtrasPostNotchRemapFn)(float*, float*, const ExtrasConfig&); //Returns if we should skip hyst
typedef bool(*ExtrasCheckButtonsFn)(Buttons&, HardwareButtons&); //Returns if a configuration button combination is pressed
typedef void(*ExtrasConfigurationFn)(ExtrasConfig&, Buttons&); //Used for making configuration changes

//Different hooks for extras are defined below; more can be added.

//For post-notch remapping, certain extras might want to disable hyst processing, so the
//function type used in the callback should return whether or not it wishes to do so.
bool extrasPostNotchRemappingA(float* Ax, float* Ay, const ExtrasConfig &extrasConfig){
	bool skipHyst = false;
	for(int i=0; i<extrasRegistered; i++){
		Extra extra = extras[i];
		if (extra.hook == HOOK_POST_NOTCH_REMAPPING_A){
			ExtrasPostNotchRemapFn fn = (ExtrasPostNotchRemapFn)(extra.hookFn);
			if (fn(Ax, Ay, extrasConfig)) {
				skipHyst = true;
			}
		}
	}
	return skipHyst;
}

bool extrasPostNotchRemappingC(float* Cx, float* Cy, const ExtrasConfig &extrasConfig){
	bool skipHyst = false;
	for(int i=0; i<extrasRegistered; i++){
		Extra extra = extras[i];
		if (extra.hook == HOOK_POST_NOTCH_REMAPPING_C){
			ExtrasPostNotchRemapFn fn = (ExtrasPostNotchRemapFn)extra.hook;
			if (fn(Cx, Cy, extrasConfig)) {
				skipHyst = true;
			}
		}
	}
	return skipHyst;
}

bool extrasCheckConfigurationButtons(Buttons &btn, HardwareButtons &hardware, ExtrasConfig &extrasConfig){
	for(int i=0; i<extrasRegistered; i++){
		Extra extra = extras[i];
		ExtrasCheckButtonsFn buttonsFn = (ExtrasCheckButtonsFn)(extra.checkButtonsFn);
		if (buttonsFn(btn, hardware)) {
			ExtrasConfigurationFn configFn = (ExtrasConfigurationFn)(extra.configureFn);
			configFn(extrasConfig, btn);
			return true; //clear buttons on return
		}
	}
	return false;
}

//Each extra and plugin can define how they integrate into the code here.
void extrasInit() {
	Serial.println("Extra: Setting up extras!");
	struct Extra extra; //Can keep reusing due to pass-by-value

#ifdef EXTRAS_ESS
	extra.hook           = HOOK_POST_NOTCH_REMAPPING_A;
	extra.hookFn         = (void*)extrasEss::hook;
	extra.checkButtonsFn = (void*)extrasEss::checkButtons;
	extra.configureFn    = (void*)extrasEss::toggleEss;
	extrasRegister(extra);
	Serial.println("Extra: ESS functionality enabled.");
#endif

}

#endif //EXTRAS_H