#include <cmath>
#include "cvideo.h"
#include "menu.h"
#include "storage/pages/storage.h"

void navigateMenu(unsigned char bitmap[],
		unsigned int &menu,
		int &itemIndex,
		bool &redraw,
		bool &changeMade,
		volatile bool &pleaseCommit,
		Buttons &hardware,
		ControlConfig &controls) {
	const uint8_t buttonLockout = 10;// 1/6 of a second of ignoring button bounce
	const uint8_t dpadLockout = 15;// 1/4 of a second of ignoring button bounce
	static uint8_t aLockout = 0;
	static uint8_t duLockout = 0;
	static uint8_t ddLockout = 0;
	static uint8_t dlLockout = 0;
	static uint8_t drLockout = 0;
	//only decrement the lockout if the button/direction is released
	//it'll be unlocked after it hits zero
	if(!hardware.A && aLockout > 0) {
		aLockout--;
	}
	duLockout = fmax(0, duLockout - 1);
	ddLockout = fmax(0, ddLockout - 1);
	dlLockout = fmax(0, dlLockout - 1);
	drLockout = fmax(0, drLockout - 1);

	static uint8_t duCounter = 0;//for controls that go faster when you hold them
	static uint8_t ddCounter = 0;//for controls that go faster when you hold them
	if(MenuIndex[menu][1] == 0) {
		if(hardware.A) {
			aLockout = buttonLockout;
			menu = MenuIndex[menu][2];
			itemIndex = 0;
			redraw = true;
			return;
		}
	} else if(MenuIndex[menu][1] > 0) {
		static uint8_t backAccumulator = 0;
		if(hardware.B && !hardware.S) {//back, but not hitting the start button (save)
			backAccumulator++;
		} else {
			if(backAccumulator > 0) {
				backAccumulator--;
			}
		}
		if(backAccumulator >= 30) {
			backAccumulator = 0;
			aLockout = 0;//make A available immediately after backing out
			menu = MenuIndex[menu][0];
			itemIndex = 0;
			changeMade = false;
			redraw = true;
			return;
		}

		if(MenuIndex[menu][1] <= 6) {
			//if it's a submenu, handle a, dup, and ddown
			if(hardware.A) {
				if(aLockout == 0) {
					aLockout = buttonLockout;
					menu = MenuIndex[menu][itemIndex + 2];
					itemIndex = 0;
					changeMade = false;
					redraw = true;
					return;
				}
			}
			if(hardware.Du && duLockout == 0) {
				duLockout = dpadLockout;//a quarter of a second
				ddLockout = 0;
				itemIndex = fmax(0, itemIndex-1);
				redraw = true;
				return;
			}
			if(hardware.Dd && ddLockout == 0) {
				ddLockout = dpadLockout;//a quarter of a second
				duLockout = 0;
				itemIndex = fmin(MenuIndex[menu][1]-1, itemIndex+1);
				redraw = true;
				return;
			}
		} else {
			//Big switch case for controls for all the bottom level items
			static int tempInt1 = 0;
			static int tempInt2 = 0;
			switch(menu) {
				case MENU_STICKDBG:
					//index 0 is live stick output so always refresh
					if(itemIndex == 0) {
						redraw = true;
					}
					//A cycles through pages
					if(hardware.A) {
						if(aLockout == 0) {
							aLockout = 10;
							itemIndex++;
							if(itemIndex > 3) {
								itemIndex = 0;
							}
							redraw = true;
						}
					}
					return;
				case MENU_ASNAPBACK:
					if(!changeMade) {
						tempInt1 = controls.xSnapback;
						tempInt2 = controls.ySnapback;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.xSnapback = fmin(controls.snapbackMax, controls.xSnapback+1);
						} else {//itemIndex == 1
							controls.ySnapback = fmin(controls.snapbackMax, controls.ySnapback+1);
						}
						changeMade = (controls.xSnapback != tempInt1) || (controls.ySnapback != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.xSnapback = fmax(controls.snapbackMin, controls.xSnapback-1);
						} else {//itemIndex == 1
							controls.ySnapback = fmax(controls.snapbackMin, controls.ySnapback-1);
						}
						changeMade = (controls.xSnapback != tempInt1) || (controls.ySnapback != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						setXSnapbackSetting(controls.xSnapback);
						setYSnapbackSetting(controls.ySnapback);
						tempInt1 = controls.xSnapback;
						tempInt2 = controls.ySnapback;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					}
					return;
				case MENU_AWAVE:
					if(!changeMade) {
						tempInt1 = controls.axWaveshaping;
						tempInt2 = controls.ayWaveshaping;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.axWaveshaping = fmin(controls.waveshapingMax, controls.axWaveshaping+1);
						} else {//itemIndex == 1
							controls.ayWaveshaping = fmin(controls.waveshapingMax, controls.ayWaveshaping+1);
						}
						changeMade = (controls.axWaveshaping != tempInt1) || (controls.ayWaveshaping != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.axWaveshaping = fmax(controls.waveshapingMin, controls.axWaveshaping-1);
						} else {//itemIndex == 1
							controls.ayWaveshaping = fmax(controls.waveshapingMin, controls.ayWaveshaping-1);
						}
						changeMade = (controls.axWaveshaping != tempInt1) || (controls.ayWaveshaping != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						setWaveshapingSetting(controls.axWaveshaping, ASTICK, XAXIS);
						setWaveshapingSetting(controls.ayWaveshaping, ASTICK, YAXIS);
						tempInt1 = controls.axWaveshaping;
						tempInt2 = controls.ayWaveshaping;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					}
					return;
				case MENU_ASMOOTH:
					if(!changeMade) {
						tempInt1 = controls.axSmoothing;
						tempInt2 = controls.aySmoothing;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.axSmoothing = fmin(controls.smoothingMax, controls.axSmoothing+1);
						} else {//itemIndex == 1
							controls.aySmoothing = fmin(controls.smoothingMax, controls.aySmoothing+1);
						}
						changeMade = (controls.axSmoothing != tempInt1) || (controls.aySmoothing != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.axSmoothing = fmax(controls.smoothingMin, controls.axSmoothing-1);
						} else {//itemIndex == 1
							controls.aySmoothing = fmax(controls.smoothingMin, controls.aySmoothing-1);
						}
						changeMade = (controls.axSmoothing != tempInt1) || (controls.aySmoothing != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						setXSmoothingSetting(controls.axSmoothing);
						setYSmoothingSetting(controls.aySmoothing);
						tempInt1 = controls.axSmoothing;
						tempInt2 = controls.aySmoothing;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					}
					return;
				case MENU_CSNAPBACK:
					if(!changeMade) {
						tempInt1 = controls.cxSmoothing;
						tempInt2 = controls.cySmoothing;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.cxSmoothing = fmin(controls.smoothingMax, controls.cxSmoothing+1);
						} else {//itemIndex == 1
							controls.cySmoothing = fmin(controls.smoothingMax, controls.cySmoothing+1);
						}
						changeMade = (controls.cxSmoothing != tempInt1) || (controls.cySmoothing != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.cxSmoothing = fmax(controls.smoothingMin, controls.cxSmoothing-1);
						} else {//itemIndex == 1
							controls.cySmoothing = fmax(controls.smoothingMin, controls.cySmoothing-1);
						}
						changeMade = (controls.cxSmoothing != tempInt1) || (controls.cySmoothing != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						setCxSmoothingSetting(controls.cxSmoothing);
						setCySmoothingSetting(controls.cySmoothing);
						tempInt1 = controls.cxSmoothing;
						tempInt2 = controls.cySmoothing;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					}
					return;
				case MENU_CWAVE:
					if(!changeMade) {
						tempInt1 = controls.cxWaveshaping;
						tempInt2 = controls.cyWaveshaping;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.cxWaveshaping = fmin(controls.waveshapingMax, controls.cxWaveshaping+1);
						} else {//itemIndex == 1
							controls.cyWaveshaping = fmin(controls.waveshapingMax, controls.cyWaveshaping+1);
						}
						changeMade = (controls.cxWaveshaping != tempInt1) || (controls.cyWaveshaping != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						if(itemIndex == 0) {
							controls.cxWaveshaping = fmax(controls.waveshapingMin, controls.cxWaveshaping-1);
						} else {//itemIndex == 1
							controls.cyWaveshaping = fmax(controls.waveshapingMin, controls.cyWaveshaping-1);
						}
						changeMade = (controls.cxWaveshaping != tempInt1) || (controls.cyWaveshaping != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						setWaveshapingSetting(controls.cxWaveshaping, CSTICK, XAXIS);
						setWaveshapingSetting(controls.cyWaveshaping, CSTICK, YAXIS);
						tempInt1 = controls.cxWaveshaping;
						tempInt2 = controls.cyWaveshaping;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					}
					return;
				case MENU_COFFSET:
					if(!changeMade) {
						tempInt1 = controls.cXOffset;
						tempInt2 = controls.cYOffset;
					}
					if(hardware.Dl && dlLockout == 0) {
						dlLockout = dpadLockout;
						drLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						//clear the repetition counters
						duCounter = 0;
						ddCounter = 0;
						itemIndex = 0;
						redraw = true;
					} else if(hardware.Dr && drLockout == 0) {
						drLockout = dpadLockout;
						dlLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						ddLockout = dpadLockout;
						duLockout = dpadLockout;
						//clear the repetition counters
						duCounter = 0;
						ddCounter = 0;
						itemIndex = 1;
						redraw = true;
					} else if(hardware.Du && duLockout == 0) {
						duLockout = dpadLockout;
						ddLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						//Go by 10 after 10 steps
						if(duCounter <= 10) {
							duCounter++;
						}
						ddCounter = 0;
						const uint8_t change = duCounter > 10 ? 10 : 1;
						if(itemIndex == 0) {
							controls.cXOffset = fmin(controls.cMax, controls.cXOffset+change);
						} else {//itemIndex == 1
							controls.cYOffset = fmin(controls.cMax, controls.cYOffset+change);
						}
						changeMade = (controls.cXOffset != tempInt1) || (controls.cYOffset != tempInt2);
						redraw = true;
					} else if(hardware.Dd && ddLockout == 0) {
						ddLockout = dpadLockout;
						duLockout = 0;
						//also lock out perpendicular directions to prevent misinputs
						dlLockout = dpadLockout;
						drLockout = dpadLockout;
						//Go by 10 after 10 steps
						if(ddCounter <= 10) {
							ddCounter++;
						}
						duCounter = 0;
						const uint8_t change = ddCounter > 10 ? 10 : 1;
						if(itemIndex == 0) {
							controls.cXOffset = fmax(controls.cMin, controls.cXOffset-change);
						} else {//itemIndex == 1
							controls.cYOffset = fmax(controls.cMin, controls.cYOffset-change);
						}
						changeMade = (controls.cXOffset != tempInt1) || (controls.cYOffset != tempInt2);
						redraw = true;
					} else if(backAccumulator > 5 && changeMade) {
						//clear the repetition counters
						duCounter = 0;
						ddCounter = 0;
						setCxOffsetSetting(controls.cXOffset);
						setCyOffsetSetting(controls.cYOffset);
						tempInt1 = controls.cXOffset;
						tempInt2 = controls.cYOffset;
						changeMade = false;
						redraw = true;
						pleaseCommit = true;//ask the other thread to commit settings to flash
					} else {
						//clear the repetition counters
						if(duLockout == 0) {
							duCounter = 0;
						}
						if(ddLockout == 0) {
							ddCounter = 0;
						}
					}
					return;
				default:
					//do nothing
					return;
			}
		}
	}
}

