#ifndef MENUSTRINGS_H
#define MENUSTRINGS_H

const char arrowPointer[] = ">";
const char bToSave[] = "Press B to save";
const char ud_only[] = "Dpad U/D changes setting.";
const char lr_ud[] = "Dpad L/R selects setting, U/D changes setting.";
const char currentSetting[] = "Current setting:";

const char splashWelcome[] = "Welcome to PhobVision";
const char splashPress[] = "Please press A";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char autoinit1[]       = "By default you have to press B to get the sticks";
const char autoinit2[]       = " working, which is important for calibration on";
const char autoinit3[]       = " Dolphin. Once it's calibrated you can turn this";
const char autoinit4[]       = " on which starts the sticks automatically.";

const char stickdbgARaw[] = "A raw values";
const char stickdbgCRaw[] = "C raw values";
const char stickdbgALin[] = "A linearized values";
const char stickdbgCLin[] = "C linearized values";
const char stickdbgAMap[] = "A remapped values";
const char stickdbgCMap[] = "C remapped values";
const char stickdbgAXfit[] = "A fit coeffs X";
const char stickdbgAYfit[] = "A fit coeffs Y";
const char stickdbgCXfit[] = "C fit coeffs X";
const char stickdbgCYfit[] = "C fit coeffs Y";
const char stickdbgAaff[] = "A affine transform";
const char stickdbgCaff[] = "C affine transform";
const char stickdbgAang[] = "A bounds angles";
const char stickdbgCang[] = "C bounds angles";

const char set_overAutoOn[] = "Autoinit on";
const char set_overAutoOff[] = "Autoinit off";
const char set_overJumpDf[] = "Normal Jump";
const char set_overJumpXZ[] = "X Z-jump";
const char set_overJumpYZ[] = "Y Z-jump";
const char set_overJumpXL[] = "X L-jump";
const char set_overJumpXR[] = "X R-jump";
const char set_overJumpYL[] = "Y L-jump";
const char set_overJumpYR[] = "Y R-jump";
const char set_overJumpBr[] = "Broken jump config";

const char leftStickX[]      = "Left stick X:";
const char leftStickY[]      = "Left stick Y:";
const char rightStickX[]     = "Right stick X:";
const char rightStickY[]     = "Right stick Y:";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char asnapback1[]      = "Set so that snapback < 23 for Melee.";
const char asnapback2[]      = "0 disables the filter. >0 shortens rise time.";
const char asnapback3[]      = "Higher makes the stick return to center slower.";
const char asnapback4[]      = "Min: 0  Max: 10  Default: 4";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char awave1[]          = "This makes the stick stop during fast inputs.";
const char awave2[]          = "Higher values reduce the speed threshold.";
const char awave3[]          = "It affects pivots, DBOOC, dashback, and more.";
const char awave4[]          = "Min: 0  Max: 15  Default: 0";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char asmooth1[]        = "This averages the stick position with the past.";
const char asmooth2[]        = "It can be used to make one axis lag the other.";
const char asmooth3[]        = "Use X=5 to make down-and-in ledgedashes work.";
const char asmooth4[]        = "Min: 0  Max: 9  Default: 0";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
//const char csmooth1[]        = "This averages the stick position with the past.";//same as asmooth1
const char csmooth2[]        = "Make these 5 or 6 to reduce snapback.";
const char csmooth3[]        = "Make one axis higher to make it less responsive.";
const char csmooth4[]        = "Min: 0  Max: 9  Default: 0";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
//const char cwave1[]          = "This makes the stick stop during fast inputs.";//same as awave
//const char cwave2[]          = "Higher values reduce the speed threshold.";
const char cwave3[]          = "We're not sure what it can be used for here.";
//const char cwave4[]          = "Min: 0  Max: 15  Default: 0";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char coffset1[]        = "This changes the default origin of the C-Stick.";
const char coffset2[]        = "This was for down-angled fsmash but it's better";
const char coffset3[]        = " done by notch cal so this might be removed.";
const char coffset4[]        = "Min: 127  Max: -127  Default: 0";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char remap1[]          = "This swaps one of L, R, or Z with either X or Y.";
const char remap2[]          = "When L or R are swapped, they are digital only.";
const char remap3[]          = "Use Trigger Mode 5 to get an analog trigger";
const char remap4[]          = "value on a face button for lightshielding.";
const char remapDf[]         = "Default";
const char remapXZ[]         = "Swap X and Z";
const char remapYZ[]         = "Swap Y and Z";
const char remapXL[]         = "Swap X and L";
const char remapYL[]         = "Swap Y and L";
const char remapXR[]         = "Swap X and R";
const char remapYR[]         = "Swap Y and R";
const char remapBr[]         = "Broken Config";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char rumble1[]         = "This controls the strength of rumble feedback.";
const char rumble2[]         = "0 is off, the max is 11, and the default is 9.";
const char rumble3[]         = "9 is ~OEM strength. <5 won't spin OEM motors.";
const char rumble4[]         = "Use values 5 or lower for 3V cellphone rumble.";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char trigger1[]        = "L Mode:";
const char trigger2[]        = "R Mode:";
const char trigger3[]        = "L Offset:";
const char trigger4[]        = "R Offset:";
const char trigger5[]        = "Mode 5 conflict; R will be inoperable";
const char trigger6[]        = "Mode 5 conflict; L will be inoperable";

//Guide for 30 offset        =        100       200       300       400       500 //510 is the last char
const char ltrigger1[]       = "This changes the behavior of the L trigger.";
const char rtrigger1[]       = "This changes the behavior of the R trigger.";
const char l5conflict[]      = "Conflict with R Trigger Mode 5";
const char r5conflict[]      = "Conflict with L Trigger Mode 5";
const char lrultimate[]      = "Increase offset to enable shield in Ultimate.";
const char lrtrigger2[]      = "Mode:";
const char lrtrigger3[]      = "Offset:";
const char lrtrigger4[]      = "This is the default trigger behavior.";
const char lrtrigger5[]      = "This disables analog triggers (lightshield).";
const char lrtrigger6[]      = "You won't be able to shield in S4 and Ultimate.";
const char lrtrigger7[]      = "This disables the hard press but preserves LRAS.";
const char lrtrigger8[]      = "In Melee you cannot airdodge or tech with this.";
const char lrtrigger9[]      = "This caps the maximum analog value at";
const char lrtrigger10[]     = "In Melee, you get a large lightshield for much";
const char lrtrigger11[]     = " of the slider travel, then a hardshield.";
const char lrtrigger12[]     = "This outputs an analog value when hard pressed.";
const char lrtrigger13[]     = "In the Melee ruleset, you cannot use this and";
const char lrtrigger14[]     = " have an analog trigger input active.";
const char lrtrigger15[]     = "This outputs both an analog value and a digital";
const char lrtrigger16[]     = "  output when hard pressed.";
const char lrtrigger17[]     = "In Melee this is the same as mode 2 but it lets";
const char lrtrigger18[]     = " you shield in Ultimate when the offset > 79.";
const char lrtrigger19[]     = "This increases the sensitivity of the analog by";
const char lrtrigger20[]     = " a multiplier that changes with the offset.";
const char lrtrigger21[]     = "This makes it respond quicker in Ultimate.";
const char lrtrigger22[]     = "Multiplier:";
const char lrtrigger23[]     = "Trigger Inputs:";
const char lrtrigger24[]     = "Trigger Outputs:";

#endif //MENUSTRINGS_H
