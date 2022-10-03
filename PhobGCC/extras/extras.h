#ifndef EXTRAS_H
#define EXTRAS_H

/* Plugins and extra features outside the normal scope of
 * the PhobGCC can be found here. Uncomment each #define
 * to include the features they provide in a build.
 * ------------------------------------------------------
 */
#define EXTRAS_ESS
// ------------------------------------------------------

#include "ess/ess.h"

//Add in user-defined configuration options here
ExtrasConfig _extrasConfig {
	.essEnable = EXTRAS_ESS_DISABLED
};

//Currently supports 20 extras, can easily be changed however.
static const int _extrasMax = 20;

static int extrasHooksRegistered = 0;
static ExtrasHook extrasHooks[_extrasMax];

static int extrasConfigFunctionsRegistered = 0;
static ExtrasConfigFunction extrasConfigFunctions[_extrasMax];

void extrasRegisterHook(ExtrasHook hook){
	if (extrasHooksRegistered >= _extrasMax){
		Serial.println("Too many extras hooks registered! Aborting...");
		return;
	}
	extrasHooks[extrasHooksRegistered] = hook;
	extrasHooksRegistered++;
}

void extrasRegisterConfig(ExtrasConfigFunction configFunction){
	if (extrasConfigFunctionsRegistered >= _extrasMax){
		Serial.println("Too many extras configs registered! Aborting...");
		return;
	}
	extrasConfigFunctions[extrasConfigFunctionsRegistered] = configFunction;
	extrasConfigFunctionsRegistered++;
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
	for(int i=0; i<extrasHooksRegistered; i++){
		ExtrasHook hook = extrasHooks[i];
		if (hook.hook == HOOK_POST_NOTCH_REMAPPING_A){
			ExtrasPostNotchRemapFn fn = (ExtrasPostNotchRemapFn)(hook.hookFn);
			if (fn(Ax, Ay, extrasConfig)) {
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
		if (hook.hook == HOOK_POST_NOTCH_REMAPPING_C){
			ExtrasPostNotchRemapFn fn = (ExtrasPostNotchRemapFn)hook.hook;
			if (fn(Cx, Cy, extrasConfig)) {
				skipHyst = true;
			}
		}
	}
	return skipHyst;
}

bool extrasCheckConfigurationButtons(Buttons &btn, HardwareButtons &hardware, ExtrasConfig &extrasConfig){
	for(int i=0; i<extrasConfigFunctionsRegistered; i++){
		ExtrasConfigFunction configFunction = extrasConfigFunctions[i];
		ExtrasCheckButtonsFn buttonsFn = (ExtrasCheckButtonsFn)(configFunction.checkButtonsFn);
		if (buttonsFn(btn, hardware)) {
			ExtrasConfigurationFn configFn = (ExtrasConfigurationFn)(configFunction.configureFn);
			configFn(extrasConfig, btn);
			return true; //clear buttons on return
		}
	}
	return false;
}

//Each extra and plugin can define how they integrate into the code here.
void extrasInit() {
	Serial.println("Extra: Setting up extras!");
	struct ExtrasHook hook;
	struct ExtrasConfigFunction configFunction;

#ifdef EXTRAS_ESS
	hook.hook   = HOOK_POST_NOTCH_REMAPPING_A;
	hook.hookFn = (void*)extrasEss::hook;
	configFunction.checkButtonsFn = (void*)extrasEss::checkButtons;
	configFunction.configureFn    = (void*)extrasEss::toggleEss;
	extrasRegisterHook(hook);
	extrasRegisterConfig(configFunction);
	Serial.println("Extra: ESS functionality enabled.");
#endif

}

#endif //EXTRAS_H