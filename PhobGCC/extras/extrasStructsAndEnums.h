#ifndef EXTRAS_ENUMS_H
#define EXTRAS_ENUMS_H

//Structs and Enums for user-defined extras

enum ExtrasEssConfig {
	EXTRAS_ESS_DISABLED,
	EXTRAS_ESS_ENABLED
};

//Structs and Enums for the extras framework

enum ExtrasConfigButtonCombo{
	EXTRAS_BTNCONFIG_STICKS_UP,
	EXTRAS_BTNCONFIG_STICKS_DOWN,
	EXTRAS_BTNCONFIG_STICKS_LEFT,
	EXTRAS_BTNCONFIG_STICKS_RIGHT,
	EXTRAS_BTNCONFIG_SIZE
};

enum ExtrasHookType{
	HOOK_DEFAULT, //unused
	HOOK_POST_NOTCH_REMAPPING_A,
	HOOK_POST_NOTCH_REMAPPING_C,
};

struct ExtrasConfig{
	ExtrasEssConfig essEnable;
};

struct ExtrasHook{
	ExtrasHookType hookType;
	void* hookFn;
};

#endif //EXTRAS_ENUMS_H