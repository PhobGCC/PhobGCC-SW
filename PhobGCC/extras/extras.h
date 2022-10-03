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

static int extrasConfigActionsRegistered = 0;
static ExtrasConfigAction extrasConfigActions[_extrasMax];

void extrasRegisterHook(ExtrasHookType hookType, void* hookFn){
	if (extrasHooksRegistered >= _extrasMax){
		Serial.println("Too many extras hooks registered! Aborting...");
		return;
	}
	extrasHooks[extrasHooksRegistered].hookType = hookType;
	extrasHooks[extrasHooksRegistered].hookFn = hookFn;
	extrasHooksRegistered++;
}

void extrasRegisterConfig(void* checkButtonsFn, void* configureFn){
	if (extrasConfigActionsRegistered >= _extrasMax){
		Serial.println("Too many extras configs registered! Aborting...");
		return;
	}
	extrasConfigActions[extrasConfigActionsRegistered].checkButtonsFn = checkButtonsFn;
	extrasConfigActions[extrasConfigActionsRegistered].configureFn = configureFn;
	extrasConfigActionsRegistered++;
}

//Common callback function types that should be used for the different ways we interact with extras
typedef bool(*ExtrasPostNotchRemapFn)(float*, float*, const ExtrasConfig&); //Returns if we should skip hyst
typedef bool(*ExtrasCheckButtonsFn)(Buttons&, HardwareButtons&); //Returns if a configuration button combination is pressed
typedef void(*ExtrasConfigureFn)(ExtrasConfig&, Buttons&); //Used for making configuration changes

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

bool extrasCheckConfigurationButtons(Buttons &btn, HardwareButtons &hardware, ExtrasConfig &extrasConfig){
	for(int i=0; i<extrasConfigActionsRegistered; i++){
		ExtrasConfigAction configAction = extrasConfigActions[i];
		ExtrasCheckButtonsFn checkButtonsFn = (ExtrasCheckButtonsFn)(configAction.checkButtonsFn);
		if (checkButtonsFn(btn, hardware)) {
			ExtrasConfigureFn configureFn = (ExtrasConfigureFn)(configAction.configureFn);
			configureFn(extrasConfig, btn);
			return true; //clear buttons on return
		}
	}
	return false;
}

//Each extra and plugin can define how they integrate into the code here.
void extrasInit() {
	Serial.println("Extra: Setting up extras!");

#ifdef EXTRAS_ESS
	extrasRegisterHook(HOOK_POST_NOTCH_REMAPPING_A, (void*)extrasEss::hook);
	extrasRegisterConfig((void*)extrasEss::checkButtons, (void*)extrasEss::toggleEss);
	Serial.println("Extra: ESS functionality enabled.");
#endif

}

#endif //EXTRAS_H