#ifndef EXTRAS_H
#define EXTRAS_H

/* Plugins and extra features outside the normal scope of
 * the PhobGCC can be found here. Uncomment each #define
 * to include the features they provide in a build.
 * ------------------------------------------------------
 */
#define EXTRAS_ESS
// ------------------------------------------------------

#include "extrasStructsAndEnums.h"
#include "extrasSettings.h"

#include "ess/ess.h"

//Add in user-defined configuration options here
ExtrasConfig _extrasConfig {
	.essEnable = EXTRAS_ESS_DISABLED
};

//Common callback function types that should be used for the different ways we interact with extras
typedef bool(*ExtrasPostNotchRemapFn)(float*, float*, const ExtrasConfig&); //Returns if we should skip hyst
typedef void(*ExtrasConfigureFn)(ExtrasConfig&, Buttons&); //Used for making configuration changes

static const int _hooksMax = 16;
static int extrasHooksRegistered = 0;
static ExtrasHook extrasHooks[_hooksMax];

static ExtrasConfigureFn configureFunctions[EXTRAS_BTNCONFIG_SIZE] = {NULL};

void extrasRegisterHook(ExtrasHookType hookType, void* hookFn){
	if (extrasHooksRegistered >= _hooksMax){
		Serial.println("Extras: Too many extras hooks registered! Aborting...");
		return;
	}
	extrasHooks[extrasHooksRegistered].hookType = hookType;
	extrasHooks[extrasHooksRegistered].hookFn = hookFn;
	extrasHooksRegistered++;
}

void extrasRegisterConfig(ExtrasConfigButtonCombo buttonCombo, ExtrasConfigureFn configureFn){
	if (configureFunctions[buttonCombo]){
		Serial.println("Extras: Button combo already registered! Aborting...");
		return;
	}
	configureFunctions[buttonCombo] = configureFn;
}

//Different hooks for extras are defined below; more can be added.

//For post-notch remapping, certain extras might want to disable hyst processing, so the
//function type used in the callback should return whether or not it wishes to do so.
bool extrasPostNotchRemappingA(float* Ax, float* Ay, const ExtrasConfig &extrasConfig){
	bool skipHyst = false;
	for(int i=0; i<extrasHooksRegistered; i++){
		ExtrasHook hook = extrasHooks[i];
		if (hook.hookType == HOOK_POST_NOTCH_REMAPPING_A){
			ExtrasPostNotchRemapFn hookFn = (ExtrasPostNotchRemapFn)(hook.hookFn);
			if (hookFn(Ax, Ay, extrasConfig)) {
				skipHyst = true;
			}
		}
	}
	return skipHyst;
}

bool extrasPostNotchRemappingC(float* Cx, float* Cy, const ExtrasConfig &extrasConfig){
	bool skipHyst = false;
	for(int i=0; i<extrasHooksRegistered; i++){
		ExtrasHook hook = extrasHooks[i];
		if (hook.hookType == HOOK_POST_NOTCH_REMAPPING_C){
			ExtrasPostNotchRemapFn hookFn = (ExtrasPostNotchRemapFn)(hook.hookFn);
			if (hookFn(Cx, Cy, extrasConfig)) {
				skipHyst = true;
			}
		}
	}
	return skipHyst;
}

//We provide 4 different button combinations to open up a user-defined configuration function.
//The combination is A + Dpad Down with both the Analog and C-Sticks pointing in one of 4 cardinal directions.
bool extrasCheckConfigurationButtons(Buttons &btn, HardwareButtons &hardware, ExtrasConfig &extrasConfig){
	ExtrasConfigureFn configureFn;
	// Up
	if (btn.Ay > (_intOrigin+48) && btn.Cy > (_intOrigin+48) && btn.A && btn.Dd) {
		configureFn = configureFunctions[EXTRAS_BTNCONFIG_STICKS_UP];
		if (configureFn) {
			configureFn(extrasConfig, btn);
			return true;
		}
	// Down
	} else if (btn.Ay < (_intOrigin-48) && btn.Cy < (_intOrigin-48) && btn.A && btn.Dd) {
		configureFn = configureFunctions[EXTRAS_BTNCONFIG_STICKS_DOWN];
		if (configureFn) {
			configureFn(extrasConfig, btn);
			return true;
		}
	// Left
	} else if (btn.Ax < (_intOrigin-48) && btn.Cx < (_intOrigin-48) && btn.A && btn.Dd) {
		configureFn = configureFunctions[EXTRAS_BTNCONFIG_STICKS_LEFT];
		if (configureFn) {
			configureFn(extrasConfig, btn);
			return true;
		}
	// Right
	} else if (btn.Ax > (_intOrigin+48) && btn.Cx > (_intOrigin+48) && btn.A && btn.Dd) {
		configureFn = configureFunctions[EXTRAS_BTNCONFIG_STICKS_RIGHT];
		if (configureFn) {
			configureFn(extrasConfig, btn);
			return true;
		}
	}
	return false;
}

//Each extra can define how they integrate into the code here.
//Make sure two extras don't have the same registered configuration button combo!
void extrasInit() {
	Serial.println("Extra: Setting up extras!");

#ifdef EXTRAS_ESS
	extrasRegisterHook(HOOK_POST_NOTCH_REMAPPING_A, (void*)extrasEss::hook);
	extrasRegisterConfig(EXTRAS_BTNCONFIG_STICKS_RIGHT, extrasEss::toggleEss);
	Serial.println("Extra: ESS functionality enabled.");
#endif

}

#endif //EXTRAS_H