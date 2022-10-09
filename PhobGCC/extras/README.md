# Developer's Reference for Extras

This document (hopefully) intends to be a helpful guide for developers who wish to add additional features
to the PhobGCC called Extras that, while might be out of scope for normal operation, are officially supported 
and can be enabled manually by users if they uncomment out a `#define` directive and assign the Extra to one of
4 directional configurations.

# Overview

There is a loose framework for Extras written into the PhobGCC codebase for where to write your code, how to
hook your Extra into the main logic of the PhobGCC, and toggling / configuration. 

The general idea is that you write the logic for it as its own header file in the 'extras' folder while 
giving it its own namespace, write hooks to your code wrapped in `#ifdef`s in the main logic of the
PhobCC, then give function pointers for configuration and toggling your Extra to the PhobGCC 
Extras registration code in 'extras.h'.

We'll be going through all of these in this document.

# Data Structures

There's various data structures that you'll be utilizing as you write your Extra:

## Configuration Structures 

### ExtrasSlot
`ExtrasSlot` is an enum that is used as an index to reference which configuration direction slot an
Extra is using. You'll want to save a reference to which slot your configuration is using somewhere
in your Extra's header file since you'll need it to save your configuration settings.
```c++
enum ExtrasSlot{
	EXTRAS_UP,
	EXTRAS_DOWN,
	EXTRAS_LEFT,
	EXTRAS_RIGHT,
	EXTRAS_SIZE,
	EXTRAS_UNSET
};
```

### ExtrasConfig and IntOrFloat
`IntOrFloat` is a union that stores, well, either an `int` or a `float`. This is used as a part of the
Extras configuration struct `ExtrasConfig`, where each Extra gets 4 words worth of storage in the
EEPROM for storing and loading configuration settings. 
```c++
union IntOrFloat{
	int intValue;
	float floatValue;
};

struct ExtrasConfig{
	IntOrFloat config[4];
};
```

This is stored at the end of the `ControlConfig` struct which holds the general 
live configuration of the PhobGCC. 

```c++
/* ... */
	const int snapbackDefault;
	const float smoothingMin;
	const float smoothingMax;
	ExtrasConfig extras[EXTRAS_SIZE];
};
```

### Cardinals

`Cardinals` is a simple struct consisting of 4 variables, one for each direction. This is
passed in during the configuration of your Extra where each direction correlates to which
dpad button was pressed during the configuration button combo so you can handle what happens
with each direction accordingly.

```c++
struct Cardinals{
	uint8_t l : 1;
	uint8_t r : 1;
	uint8_t u : 1;
	uint8_t d : 1;
};
```

## EEPROM Setters

There's two functions you'll be using related to the configuration of your extra,
`setExtrasSettingInt` and `getExtrasSettingFloat`. These are used to write your Extra's
configuration to EEPROM so that they'll be automatically restored when the PhobGCC is
powered on after being unplugged.

```c++
void setExtrasSettingInt(const ExtrasSlot slot, const int offset, const int value);
void setExtrasSettingFloat(const ExtrasSlot slot, const int offset, const float value);
```

`slot` is the directional configuration that your Extra is 
  assigned to, you will have a reference to this in your
  Extra's header file. 
  
`offset` is a number 0-3 which determines which of the Extra's 
  4 configuration words your value will be written to. 
  
`value` is the value you're passing in to be written to EEPROM.

## Function Typedefs

There's two functions for toggling and configuring your Extra. The way to format these functions
are standardized with a `typedef` for each.

```c++
typedef bool(*ExtrasToggleFn)(IntOrFloat config[]);//Used for toggling extras
typedef void(*ExtrasConfigFn)(IntOrFloat config[], Cardinals dpad);//Used for configuring extras with the Dpad
```

### Toggling

The toggle function is called when the user is not in safe mode and uses the button combination:

* A + B + Both sticks pointed in the directional configuration

The toggle function takes in the configuration for your Extra so you can write and read from it, and returns
a `bool` that signals if you enabled (`true`) or disabled (`false`) your Extra. This return value
is used to freeze both sticks facing up-right or down-left as a visual indicator for the user
to determine if the Extra is now enabled or disabled. 

### Configuration

The config function is called when the user is not in safe mode and uses the button combination:

* A + Dpad (any direction) + Both sticks pointed in the directional configuration

The config function takes in the configuration for your Extra so you can write and read from it, but it 
also reads in the dpad as a `Cardinals` struct. In your config function, you can determine what happens depending
on what dpad button is currently being pressed.

# Writing the Hook

Depending on what you're trying to do, you'll write one or more hooks to your code's logic somewhere in
the main codebase of the PhobGCC. If you're creating some sort of new mapping or want to modify the control
sticks or analog trigger sliders, you'll probably write a hook to your code from `readSticks`. 

For example, here's the hook for the ESS Adapter Extra:

```c++
/* ... */
	//Clamp values from -125 to +125
	remappedAx = min(125, max(-125, remappedAx));
	remappedAy = min(125, max(-125, remappedAy));
	remappedCx = min(125, max(-125, remappedCx+controls.cXOffset));
	remappedCy = min(125, max(-125, remappedCy+controls.cYOffset));

	bool skipAHyst = false;
#ifdef EXTRAS_ESS
	//ESS adapter functionality for Ocarina of Time on WiiVC if enabled
	skipAHyst = ess::remap(&remappedAx, &remappedAy, controls.extras[ess::extrasEssConfigSlot].config);
#endif

	float hystVal = 0.3;
	//assign the remapped values to the button struct
	if(readA){
		if (!skipAHyst) {
/* ... */
```

Here I'm modifying the control sticks by passing them by reference to my Extra where they're
modified before logic returns back to here. 

Notice that I had to add this `skipAHyst` boolean outside of the `#ifdef`. You can add external
variables but only if needed. But ideally the only thing added is your `#ifdef` and a single
line to your header file in the 'extras' folder. 

More than likely you'll want to pass in the configuration options for your Extra as well since
the standard way to access an Extra's configuration is by passing it in from the hook where you'll
have access to `ControlConfig &controls` which contains `ExtrasConfig extras[EXTRAS_SIZE]`.

# Formatting your Extra's Header

Your Extra's header is where you have the freedom to do just about whatever you want with the
information you pass into it. There's a basic structure you'll want to follow however.

```c++
#ifndef EXTRAS_EXAMPLE_H
#define EXTRAS_EXAMPLE_H

namespace example {
  ExtrasSlot configSlot = EXTRAS_UNSET;
  
  // write your hook code here
  
  bool toggle(IntOrFloat config[]) {
    // toggle Extra and probably write enabled/disabled to EEPROM
  }

  void config(IntOrFloat config[], Cardinals dpad) {
    // config Extra and probably store it to EEPROM
  }
}

#endif //EXTRAS_EXAMPLE_H
```

Your Extra's config slot will be set from 'extras.h'. You'll use it if / when you need to save
your configuration by using the `setExtrasSettingInt` or `setExtrasSettingFloat` functions.

Again, the toggle and config functions need to follow the typdefs for `ExtrasToggleFn` and `ExtrasConfigFn`
since we'll be referencing them when we register the Extra in 'extras.h'.

# Adding your Extra to 'extras.h'

There's three things that need to be done to 'extras.h' when adding your Extra to the codebase:
* Adding a `#define` directive that can be uncommented to enable your Extra 
* Wraping the inclusion of your Extra's header file in `#ifdef`s
* Adding your Extra to the initialization function `extrasInit()`.

The first two are pretty straight forward, just add a new line under the existing Extras at the top
of the file and then just under that add your header file wrapped in the `#define` you made.

```c++
/* Plugins and extra features outside the normal scope of
 * the PhobGCC can be found here. Uncomment each #define
 * to include the features they provide in a build then
 * set the configuration slot it uses (if it needs configuration)
 * 
 */
//------------------------------------------------------
//#define EXTRAS_ESS
//#define EXTRAS_EXAMPLE <--- your new Extra
//------------------------------------------------------

#ifdef EXTRAS_ESS
#include "ess.h"
#endif

// your new Extra
#ifdef EXTRAS_EXAMPLE
#include "example.h" 
#endif
```

For the third one, you can essentially copy and paste an 
existing `#ifdef`'d blocks and modify it as needed. 

You should not add anything extra to these lines, only modify
the text and change the function references to point to the 
ones in your Extra. 

`extrasConfigAssign` takes in 3 parameters: 

* `slot` is the directional configuration slot that the users sets.
* `toggleFn` is the function pointer to your Extra's toggle function.
* `configFn` is the function pointer to your Extra's config function.

Either or both function pointers can be safely set to `NULL` if not used.

```c++
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
	extrasConfigAssign(ess::extrasEssConfigSlot, ess::toggle, NULL);
#endif

// your new Extra
#ifdef EXTRAS_EXAMPLE
	//-----------------------------------------
	example::configSlot = EXTRAS_UNSET;
	//-----------------------------------------
	Serial.println("Extra: Enabling Example functionality...");
	extrasConfigAssign(example::configSlot, example::toggle, example::config);
#endif

}
```

# Closing Comments

This is about it with regards to developing Extras for the PhobGCC firmware. If you have any questions
about how something works or need help figuring out how to format your code, or anything else related
to developing Extras, please stop by the Discord and ask for help. 

[PhobGCC Discord](https://discord.gg/yrpUu7mgzm)
