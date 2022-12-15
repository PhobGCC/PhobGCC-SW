#include <cmath>
#include "cvideo.h"
#include "menu.h"
#include "storage/pages/storage.h"

void navigateMenu(unsigned char bitmap[],
		unsigned int &menu,
		int &itemIndex,
		bool &redraw,
		bool &changeMade,
		Buttons &hardware,
		ControlConfig &controls) {
	static uint8_t aLockout = 0;
	static uint8_t duLockout = 0;
	static uint8_t ddLockout = 0;
	static uint8_t dlLockout = 0;
	static uint8_t drLockout = 0;
	if(MenuIndex[menu][1] == 0) {
		if(hardware.A) {
			aLockout = 10;
			menu = MenuIndex[menu][2];
			itemIndex = 0;
			redraw = true;
			return;
		}
	} else if(MenuIndex[menu][1] > 0) {
		static uint8_t backAccumulator = 0;
		if(hardware.B) {//back
			backAccumulator++;
		} else {
			if(backAccumulator > 0) {
				backAccumulator--;
			}
		}
		if(backAccumulator >= 30) {
			backAccumulator = 0;
			aLockout = 0;
			menu = MenuIndex[menu][0];
			itemIndex = 0;
			redraw = true;
			return;
		}

		if(MenuIndex[menu][1] <= 6) {
			//if it's a submenu, handle a, dup, and ddown
			if(hardware.A) {
				if(aLockout == 0) {
					aLockout = 10;
					menu = MenuIndex[menu][itemIndex + 2];
					itemIndex = 0;
					redraw = true;
					return;
				}
			} else {
				//only decrement the lockout if A is released
				//it'll be unlocked after 1/6 of a second unpressed
				aLockout = fmax(0, aLockout-1);
			}
			duLockout = fmax(0, duLockout-1);
			if(hardware.Du) {
				if(duLockout == 0) {
					duLockout = 15;//a quarter of a second
					ddLockout = 0;
					itemIndex = fmax(0, itemIndex-1);
					redraw = true;
					return;
				}
			}
			ddLockout = fmax(0, ddLockout-1);
			if(hardware.Dd) {
				if(ddLockout == 0) {
					ddLockout = 15;//a quarter of a second
					duLockout = 0;
					itemIndex = fmin(MenuIndex[menu][1]-1, itemIndex+1);
					redraw = true;
					return;
				}
			}
		} else {
			//Big switch case for controls for all the bottom level items
			static int tempInt1 = 0;
			static int tempInt2 = 0;
			switch(menu) {
				case MENU_STICKDBG:
					if(itemIndex == 0) {
						redraw = true;
					}
					if(hardware.A) {
						if(aLockout == 0) {
							aLockout = 10;
							itemIndex++;
							if(itemIndex > 3) {
								itemIndex = 0;
							}
							redraw = true;
							return;
						}
					} else {
						//only decrement the lockout if A is released
						//it'll be unlocked after 1/6 of a second unpressed
						aLockout = fmax(0, aLockout-1);
						if(itemIndex == 3) {
							redraw = true;
							return;
						}
					}
					return;
				case MENU_ASNAPBACK:
					if(!changeMade) {
						tempInt1 = controls.xSnapback;
						tempInt2 = controls.ySnapback;
					}
					return;
				default:
					//do nothing
					return;
			}
		}
	}
}

