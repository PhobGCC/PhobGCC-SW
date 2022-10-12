#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>

#include "../common/stick.h"
#include "../common/structsAndEnums.h"

//Reimplement all the functions in this header for each platform.
//Include the appropriate one in each per-board file.

namespace Eeprom {
	//index values to store data into eeprom
	const int _bytesPerFloat = 4;
	const int _eepromAPointsX = 0;
	const int _eepromAPointsY = _eepromAPointsX+_noOfCalibrationPoints*_bytesPerFloat;
	const int _eepromCPointsX = _eepromAPointsY+_noOfCalibrationPoints*_bytesPerFloat;
	const int _eepromCPointsY = _eepromCPointsX+_noOfCalibrationPoints*_bytesPerFloat;
	const int _eepromxSnapback = _eepromCPointsY+_noOfCalibrationPoints*_bytesPerFloat;
	const int _eepromySnapback = _eepromxSnapback+_bytesPerFloat;
	const int _eepromJump = _eepromySnapback+_bytesPerFloat;
	const int _eepromANotchAngles = _eepromJump+_bytesPerFloat;
	const int _eepromCNotchAngles = _eepromANotchAngles+_noOfNotches*_bytesPerFloat;
	const int _eepromLToggle = _eepromCNotchAngles+_noOfNotches*_bytesPerFloat;
	const int _eepromRToggle = _eepromLToggle+_bytesPerFloat;
	const int _eepromcXOffset = _eepromRToggle+_bytesPerFloat;
	const int _eepromcYOffset = _eepromcXOffset+_bytesPerFloat;
	const int _eepromxSmoothing = _eepromcYOffset+_bytesPerFloat;
	const int _eepromySmoothing = _eepromxSmoothing+_bytesPerFloat;
	const int _eepromLOffset = _eepromySmoothing+_bytesPerFloat;
	const int _eepromROffset = _eepromLOffset+_bytesPerFloat;
	const int _eepromCxSmoothing = _eepromROffset+_bytesPerFloat;
	const int _eepromCySmoothing = _eepromCxSmoothing+_bytesPerFloat;
	const int _eepromRumble = _eepromCySmoothing+_bytesPerFloat;
	const int _eepromAutoInit = _eepromRumble+_bytesPerFloat;
	const int _eepromAxWaveshaping = _eepromAutoInit+_bytesPerFloat;
	const int _eepromAyWaveshaping = _eepromAxWaveshaping+_bytesPerFloat;
	const int _eepromCxWaveshaping = _eepromAyWaveshaping+_bytesPerFloat;
	const int _eepromCyWaveshaping = _eepromCxWaveshaping+_bytesPerFloat;
	const int _eepromExtrasUp = _eepromAutoInit+_bytesPerFloat*4;
	const int _eepromExtrasDown = _eepromExtrasUp+_bytesPerFloat*4;
	const int _eepromExtrasLeft = _eepromExtrasDown+_bytesPerFloat*4;
	const int _eepromExtrasRight = _eepromExtrasLeft+_bytesPerFloat*4;
};

JumpConfig getJumpSetting() {
	JumpConfig output;
	EEPROM.get(Eeprom::_eepromJump, output);
	return output;
};

void setJumpSetting(const JumpConfig jump) {
	EEPROM.put(Eeprom::_eepromJump, jump);
};

int getLSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromLToggle, output);
	return output;
};

void setLSetting(const int L) {
	EEPROM.put(Eeprom::_eepromLToggle, L);
};

int getRSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromRToggle, output);
	return output;
};

void setRSetting(const int R) {
	EEPROM.put(Eeprom::_eepromRToggle, R);
};

int getLOffsetSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromLOffset, output);
	return output;
};

void setLOffsetSetting(const int L) {
	EEPROM.put(Eeprom::_eepromLOffset, L);
};

int getROffsetSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromROffset, output);
	return output;
};

void setROffsetSetting(const int R) {
	EEPROM.put(Eeprom::_eepromROffset, R);
};

int getCXOffsetSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromcXOffset, output);
	return output;
};

void setCXOffsetSetting(const int X) {
	EEPROM.put(Eeprom::_eepromcXOffset, X);
};

int getCYOffsetSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromcYOffset, output);
	return output;
};

void setCYOffsetSetting(const int Y) {
	EEPROM.put(Eeprom::_eepromcYOffset, Y);
};

int getXSnapbackSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromxSnapback, output);
	return output;
};

void setXSnapbackSetting(const int X) {
	EEPROM.put(Eeprom::_eepromxSnapback, X);
};

int getYSnapbackSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromySnapback, output);
	return output;
};

void setYSnapbackSetting(const int Y) {
	EEPROM.put(Eeprom::_eepromySnapback, Y);
};

float getXSmoothingSetting() {
	float output;
	EEPROM.get(Eeprom::_eepromxSmoothing, output);
	return output;
};

void setXSmoothingSetting(const float X) {
	EEPROM.put(Eeprom::_eepromxSmoothing, X);
};

float getYSmoothingSetting() {
	float output;
	EEPROM.get(Eeprom::_eepromySmoothing, output);
	return output;
};

void setYSmoothingSetting(const float Y) {
	EEPROM.put(Eeprom::_eepromySmoothing, Y);
};

float getCxSmoothingSetting() {
	float output;
	EEPROM.get(Eeprom::_eepromCxSmoothing, output);
	return output;
};

void setCxSmoothingSetting(const float X) {
	EEPROM.put(Eeprom::_eepromCxSmoothing, X);
};

float getCySmoothingSetting() {
	float output;
	EEPROM.get(Eeprom::_eepromCySmoothing, output);
	return output;
};

void setCySmoothingSetting(const float Y) {
	EEPROM.put(Eeprom::_eepromCySmoothing, Y);
};

int getRumbleSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromRumble, output);
	return output;
};

void setRumbleSetting(const int rumble) {
	EEPROM.put(Eeprom::_eepromRumble, rumble);
};

int getAutoInitSetting() {
	int output;
	EEPROM.get(Eeprom::_eepromAutoInit, output);
	return output;
};

void setAutoInitSetting(const int autoInit) {
	EEPROM.put(Eeprom::_eepromAutoInit, autoInit);
};

int getWaveshapingSetting(const WhichStick whichStick, const WhichAxis whichAxis) {
	int output;
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			EEPROM.get(Eeprom::_eepromAxWaveshaping, output);
		} else {
			EEPROM.get(Eeprom::_eepromAyWaveshaping, output);
		}
	} else {
		if(whichAxis == XAXIS) {
			EEPROM.get(Eeprom::_eepromCxWaveshaping, output);
		} else {
			EEPROM.get(Eeprom::_eepromCyWaveshaping, output);
		}
	}
	return output;
}

void setWaveshapingSetting(const int waveshaping, const WhichStick whichStick, const WhichAxis whichAxis) {
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			EEPROM.put(Eeprom::_eepromAxWaveshaping, waveshaping);
		} else {
			EEPROM.put(Eeprom::_eepromAyWaveshaping, waveshaping);
		}
	} else {
		if(whichAxis == XAXIS) {
			EEPROM.put(Eeprom::_eepromCxWaveshaping, waveshaping);
		} else {
			EEPROM.put(Eeprom::_eepromCyWaveshaping, waveshaping);
		}
	}
}

//pulls 32 points from eeprom
void getFloatPoints(const int eepromAddress, float array[32]) {
	float tempArray[32];
	EEPROM.get(eepromAddress, tempArray);
	for(int i = 0; i < 32; i++) {
		array[i] = tempArray[i];
	}
}

//writes 32 points to eeprom
void setFloatPoints(const int eepromAddress, const float array[32]) {
	float tempArray[32];
	for(int i = 0; i < 32; i++) {
		tempArray[i] = array[i];
	}
	EEPROM.put(eepromAddress, tempArray);
}

void getPointsSetting(float points[32], const WhichStick whichStick, const WhichAxis whichAxis) {
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			getFloatPoints(Eeprom::_eepromAPointsX, points);
		} else {
			getFloatPoints(Eeprom::_eepromAPointsY, points);
		}
	} else {
		if(whichAxis == XAXIS) {
			getFloatPoints(Eeprom::_eepromCPointsX, points);
		} else {
			getFloatPoints(Eeprom::_eepromCPointsY, points);
		}
	}
};

void setPointsSetting(const float points[32], const WhichStick whichStick, const WhichAxis whichAxis) {
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			setFloatPoints(Eeprom::_eepromAPointsX, points);
		} else {
			setFloatPoints(Eeprom::_eepromAPointsY, points);
		}
	} else {
		if(whichAxis == XAXIS) {
			setFloatPoints(Eeprom::_eepromCPointsX, points);
		} else {
			setFloatPoints(Eeprom::_eepromCPointsY, points);
		}
	}
};

//pulls 16 points from eeprom
void getFloatNotches(const int eepromAddress, float array[16]) {
	float tempArray[16];
	EEPROM.get(eepromAddress, tempArray);
	for(int i = 0; i < 16; i++) {
		array[i] = tempArray[i];
	}
}

//writes 32 points to eeprom
void setFloatNotches(const int eepromAddress, const float array[16]) {
	float tempArray[16];
	for(int i = 0; i < 16; i++) {
		tempArray[i] = array[i];
	}
	EEPROM.put(eepromAddress, tempArray);
}

//combination getter and setter
void getNotchAnglesSetting(float angles[16], const WhichStick whichStick) {
	if (whichStick == ASTICK) {
		getFloatNotches(Eeprom::_eepromANotchAngles, angles);
	} else {
		getFloatNotches(Eeprom::_eepromCNotchAngles, angles);
	}
}

void setNotchAnglesSetting(const float angles[16], const WhichStick whichStick) {
	if (whichStick == ASTICK) {
		setFloatNotches(Eeprom::_eepromANotchAngles, angles);
	} else {
		setFloatNotches(Eeprom::_eepromCNotchAngles, angles);
	}
}

//Extras
int getExtrasSettingInt(const ExtrasSlot slot, const int offset) {
	if (offset < 0 || offset >= 4) {
		return 0;
	}
	int output = 0;
	switch(slot) {
		case EXTRAS_UP:
			EEPROM.get(Eeprom::_eepromExtrasUp+offset, output);
			break;
		case EXTRAS_DOWN:
			EEPROM.get(Eeprom::_eepromExtrasDown+offset, output);
			break;
		case EXTRAS_LEFT:
			EEPROM.get(Eeprom::_eepromExtrasLeft+offset, output);
			break;
		case EXTRAS_RIGHT:
			EEPROM.get(Eeprom::_eepromExtrasRight+offset, output);
			break;
		default:
			break;
	}
	return output;
}

void setExtrasSettingInt(const ExtrasSlot slot, const int offset, const int value) {
	if (offset < 0 || offset >= 4) {
		return;
	}
	switch(slot) {
		case EXTRAS_UP:
			EEPROM.put(Eeprom::_eepromExtrasUp+offset, value);
			break;
		case EXTRAS_DOWN:
			EEPROM.put(Eeprom::_eepromExtrasDown+offset, value);
			break;
		case EXTRAS_LEFT:
			EEPROM.put(Eeprom::_eepromExtrasLeft+offset, value);
			break;
		case EXTRAS_RIGHT:
			EEPROM.put(Eeprom::_eepromExtrasRight+offset, value);
			break;
		default:
			break;
	}
}

float getExtrasSettingFloat(const ExtrasSlot slot, const int offset) {
	if (offset < 0 || offset >= 4) {
		return 0;
	}
	float output = 0;
	switch(slot) {
		case EXTRAS_UP:
			EEPROM.get(Eeprom::_eepromExtrasUp+offset, output);
			break;
		case EXTRAS_DOWN:
			EEPROM.get(Eeprom::_eepromExtrasDown+offset, output);
			break;
		case EXTRAS_LEFT:
			EEPROM.get(Eeprom::_eepromExtrasLeft+offset, output);
			break;
		case EXTRAS_RIGHT:
			EEPROM.get(Eeprom::_eepromExtrasRight+offset, output);
			break;
		default:
			break;
	}
	return output;
}

void setExtrasSettingFloat(const ExtrasSlot slot, const int offset, const float value) {
	if (offset < 0 || offset >= 4) {
		return;
	}
	switch(slot) {
		case EXTRAS_UP:
			EEPROM.put(Eeprom::_eepromExtrasUp+offset, value);
			break;
		case EXTRAS_DOWN:
			EEPROM.put(Eeprom::_eepromExtrasDown+offset, value);
			break;
		case EXTRAS_LEFT:
			EEPROM.put(Eeprom::_eepromExtrasLeft+offset, value);
			break;
		case EXTRAS_RIGHT:
			EEPROM.put(Eeprom::_eepromExtrasRight+offset, value);
			break;
		default:
			break;
	}
}

#endif //SETTINGS_H
