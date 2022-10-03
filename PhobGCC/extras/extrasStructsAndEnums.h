#ifndef EXTRAS_ENUMS_H
#define EXTRAS_ENUMS_H

//Structs and Enums for user-defined extras

enum ExtrasEssConfig {
	EXTRAS_ESS_DISABLED,
	EXTRAS_ESS_ENABLED
};

//Structs and Enums for the extras framework

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

struct ExtrasConfigAction{
	void* checkButtonsFn;
	void* configureFn;
};

#endif //EXTRAS_ENUMS_H