#ifndef MENUSTRINGS_H
#define MENUSTRINGS_H

const char arrowRight[] = ">";
const char arrowLeft[] = "<";
const char bToSave[] = "Press B to save";
const char ud_only[] = "Dpad U/D changes setting.";
const char lr_ud[] = "Dpad L/R selects setting, U/D changes setting.";
const char currentSetting[] = "Current setting:";

const char splashWelcome[] = "Welcome to PhobVision";
const char splashPress[] = "Please press A";

//For continuity, begin the character on the x coordinate of the number above the last character
//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
//Guide relative to offset   =  10  50   100  150

//Guide relative to offset   =  10  50   100  150  200  250  300
const char stickCal0[]       = "Measurement Phase";
const char stickCal1[]       = "Notch Adjust Phase";
const char stickCal2[]       = "Step:";
const char stickCal3[]       = "Let go of the stick";
const char stickCal4[]       = "and press A/L/R.";
const char stickCal5[]       = "Press Z to go back.";
const char stickCal6[]       = "Hold stick into notch";
const char stickCal7[]       = "If no notch exists,";
const char stickCal8[]       = "leave stick centered.";
const char stickCal9[]       = "You can now preview the";
const char stickCal10[]      = "stick motion. Put the";
const char stickCal11[]      = "stick in the indicated";
const char stickCal12[]      = "notch and press X/Y to";
const char stickCal13[]      = "move its output CW/CCW.";
const char stickCal14[]      = "Press A/L/R to move on.";
//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char stickCal15[]      = "Raw Values:      Stick Coords:    Melee Coords:";
const char stickCal16[]      = "(0 to 1)         (-127 to 127)    (-1 to 1)";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
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
const char stickdbgARnd[] = "A rounded values";
const char stickdbgCRnd[] = "C rounded values";
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

//Guide relative to offset   =  10  50   100  150
const char leftStick[]       = "Left stick:";
const char rightStick[]      = "Right stick:";
const char leftStickX[]      = "Left stick X:";
const char leftStickY[]      = "Left stick Y:";
const char rightStickX[]     = "Right stick X:";
const char rightStickY[]     = "Right stick Y:";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char asnapback1[]      = "Set so that snapback < 23 for Melee.";
const char asnapback2[]      = "0 disables the filter. >0 shortens rise time.";
const char asnapback3[]      = "<0 doesn't shorten rise time. Greater magnitude";
const char asnapback4[]      = "makes the stick return to center slower.";
const char asnapback5[]      = "Min: -10  Max: 10  Default: 4";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char awave1[]          = "This makes the stick stop during fast inputs.";
const char awave2[]          = "Higher values reduce the speed threshold.";
const char awave3[]          = "It affects pivots, DBOOC, dashback, and more.";
const char awave4[]          = "Min: 0  Max: 15  Default: 0";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char asmooth1[]        = "This averages the stick position with the past.";
const char asmooth2[]        = "It can be used to make one axis lag the other.";
const char asmooth3[]        = "Use X=5 to make down-and-in ledgedashes work.";
const char asmooth4[]        = "Min: 0  Max: 9  Default: 0";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
//const char csmooth1[]        = "This averages the stick position with the past.";//same as asmooth1
const char csmooth2[]        = "Make these 5 or 6 to reduce snapback.";
const char csmooth3[]        = "Make one axis higher to make it less responsive.";
const char csmooth4[]        = "Min: 0  Max: 9  Default: 0";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
//const char cwave1[]          = "This makes the stick stop during fast inputs.";//same as awave
//const char cwave2[]          = "Higher values reduce the speed threshold.";
const char cwave3[]          = "We're not sure what it can be used for here.";
//const char cwave4[]          = "Min: 0  Max: 15  Default: 0";

/*
//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char coffset1[]        = "This changes the default origin of the C-Stick.";
const char coffset2[]        = "This was for down-angled fsmash but it's better";
const char coffset3[]        = " done by notch cal so this might be removed.";
const char coffset4[]        = "Min: 127  Max: -127  Default: 0";
*/

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char cardinals1[]      = "This sets how far the stick snaps to the";
const char cardinals2[]      = "cardinal axes. 0 disables snapping, and -1";
const char cardinals3[]      = "disables 1.0 cardinals even on UCF 0.84+.";
const char cardinals4[]      = "Min: -1  Max: 6  Default: 6";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char radius1[]         = "This sets how far from the center the stick can";
const char radius2[]         = "reach. A normal GCC can get to roughly 100-103.";
const char radius3[]         = "Melee only sees <=80, and max possible is 127.";
const char radius4[]         = "Min: 82  Max: 125  Default: 100";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
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

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char rumble1[]         = "This controls the strength of rumble feedback.";
const char rumble2[]         = "0 is off, the max is 11, and the default is 9.";
const char rumble3[]         = "9 is ~OEM strength. <5 won't spin OEM motors.";
const char rumble4[]         = "Use values 5 or lower for 3V cellphone rumble.";
const char rumble5[]         = "Press Z to test rumble.";

//Guide relative to offset   =  10  50   100  150
const char trigger1[]        = "L Mode:";
const char trigger2[]        = "R Mode:";
const char trigger3[]        = "L Offset:";
const char trigger4[]        = "R Offset:";
//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char trigger5[]        = "Mode 5 conflict; R will be inoperable";
const char trigger6[]        = "Mode 5 conflict; L will be inoperable";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char ltrigger1[]       = "This changes the behavior of the L trigger.";
const char rtrigger1[]       = "This changes the behavior of the R trigger.";
const char l5conflict[]      = "Conflict with R Trigger Mode 5";
const char r5conflict[]      = "Conflict with L Trigger Mode 5";
const char lrultimate[]      = "Increase offset to enable shield in Ultimate.";
//Guide relative to offset   =  10  50   100  150
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
//Guide relative to offset   =  10  50   100  150
const char lrtrigger22[]     = "Multiplier:";
const char lrtrigger23[]     = "Trigger Inputs:";
const char lrtrigger24[]     = "Trigger Outputs:";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char tourney1[]        = "This prevents misinputs during tournaments by";
const char tourney2[]        = "delaying or disabling the start button, which";
const char tourney3[]        = "pauses the match, and/or dpad-up, which taunts.";
const char tourney4[]        = "You must hold delayed buttons for 1 second.";
const char tourn0[]          = "Tourney Mode Off";
const char tourn1[]          = "Taunt Delayed";
const char tourn2[]          = "Taunt Disabled";
const char tourn3[]          = "Start Delayed";
const char tourn4[]          = "Start and Taunt Delayed";
const char tourn5[]          = "Start Delayed, Taunt Disabled";
const char tournBr[]         = "Broken Tourney Mode Config";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char reset1[]          = "Press U/D to select which, and press A to reset.";
const char reset2[]          = "This resets everything to defaults except for";
const char reset3[]          = " stick calibration, which is untouched.";
const char reset4[]          = "This resets everything including stick cal.";
const char reset5[]          = "You'll need to perform stick calibration again.";
const char reset6[]          = "Are you sure you want to reset? Cancel with B.";
const char reset7[]          = "Soft Reset";
const char reset8[]          = "Hard Reset";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char inputview1[]      = "Hardware Inputs:";
const char inputview2[]      = "Controller Outputs:";
const char inputview3[]      = "Left      Left Melee      Right     Right Melee";
const char inputview4[]      = "Coords:   Coords:         Coords:   Coords:";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char xyscope0[]        = "Waiting for Motion...";
const char xyscope1[]        = "Stickmap:";
const char stickmap0[]       = "None";
const char stickmap1[]       = "Deadzones";
const char stickmap2[]       = "Wait Attacks";
const char stickmap3[]       = "Wait Movement";
const char stickmap4[]       = "Crouch";
const char stickmap5[]       = "Left Ledge";
const char stickmap6[]       = "Right Ledge";
const char xyscope2[]        = "Which Stick:";
const char leftright0[]      = "Left Stick";
const char leftright1[]      = "Right Stick";
const char xyscope3[]        = "Highlight Sample #:";
const char xyscope4[]        = "Pressed Buttons:";
const char xyscope5[]        = "Press Start to Record";
const char xyscope6[]        = "Unfilt    Melee           Filtered  Melee";
/*
const char inputview4[]      = "Coords:   Coords:         Coords:   Coords:";
*/

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char timescope0[]      = "Move Stick or Trigger";
//Guide relative to offset   =  10  50   100  150
const char timescope1[]      = "Input:";
const char timescope2[]      = "Measurement:";
const char timescope3[]      = "Snapback";
const char timescope4[]      = "Dashback";
const char timescope5[]      = "Pivots";
const char timescope6[]      = "Trigger";
//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char timescope7[]      = "Filtered:";
//Guide relative to offset   =  10  50   100  150
const char timescope8[]      = "Unfiltered:";
const char timescope9[]      = "Positive:";
const char timescope10[]     = "Negative:";
const char timescope11[]     = "Dashback:";
const char timescope12[]     = "Empty";
const char timescope13[]     = "Pivot:";
const char timescope14[]     = "No Turn:";
const char timescope15[]     = "D PS:";
const char timescope16[]     = "ADT PS:";
const char timescope17[]     = "No PS:";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char presstime1[]      = "Press Start to initiate 200ms of recording,";
const char presstime2[]      = "starting when you press ABXYLRZ or move a stick.";
const char presstime3[]      = "Waiting for input...";
//Guide relative to offset   =  10  50   100  150
const char presstime4[]      = "Auto-Repeat:";
const char presstime5[]      = "On";
const char presstime6[]      = "Off";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char reaction1[]       = "Press start to begin the reaction test.";
const char reaction2[]       = "You can press any button or move any stick.";
const char reaction3[]       = "Set stick & trigger thresholds with the D-pad.";
//Guide relative to offset   =  10  50   100  150
const char reaction4[]       = "Stick:";
const char reaction5[]       = "Trigger:";
const char reaction6[]       = "ms:";
const char reaction7[]       = "Frames:";
const char reaction8[]       = "Wait...";

//Guide for 30 offset        =  40    100       200       300       400       500 //510 is the last char
const char vision1[]         = "This controls the alignment of the interlacing.";
const char vision2[]         = "Adjust it to make the diagonal lines less jaggy.";
const char vision3[]         = "Interlacing offset:";

#endif //MENUSTRINGS_H
