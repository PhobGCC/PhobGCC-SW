#include "storage/pages/storage.h"
#include "storage/functions.hpp"

static Persistence::Pages::Storage _storage;
static bool fresh = false;

//Helper function for initializing the storage struct
void getStoragePage() {
	if(!fresh) {
		Persistence::Pages::Storage temp = Persistence::clone<Persistence::Pages::Storage>();
		_storage.settings.jump = temp.settings.jump;
		_storage.settings.l = temp.settings.l;
		_storage.settings.r = temp.settings.r;
		_storage.settings.lOffset = temp.settings.lOffset;
		_storage.settings.rOffset = temp.settings.rOffset;
		_storage.settings.cxOffset = temp.settings.cxOffset;
		_storage.settings.cyOffset = temp.settings.cyOffset;
		_storage.settings.xSnapback = temp.settings.xSnapback;
		_storage.settings.ySnapback = temp.settings.ySnapback;
		_storage.settings.xSmoothing = temp.settings.xSmoothing;
		_storage.settings.ySmoothing = temp.settings.ySmoothing;
		_storage.settings.cxSmoothing = temp.settings.cxSmoothing;
		_storage.settings.cySmoothing = temp.settings.cySmoothing;
		_storage.settings.rumble = temp.settings.rumble;
		_storage.settings.autoInit = temp.settings.autoInit;
		_storage.settings.axWaveshaping = temp.settings.axWaveshaping;
		_storage.settings.ayWaveshaping = temp.settings.ayWaveshaping;
		_storage.settings.cxWaveshaping = temp.settings.cxWaveshaping;
		_storage.settings.cyWaveshaping = temp.settings.cyWaveshaping;
		for(int i=0; i < 32; i++) {
			_storage.settings.axPoints[i] = temp.settings.axPoints[i];
			_storage.settings.ayPoints[i] = temp.settings.ayPoints[i];
			_storage.settings.cxPoints[i] = temp.settings.cxPoints[i];
			_storage.settings.cyPoints[i] = temp.settings.cyPoints[i];
		}
		for(int i=0; i < 16; i++) {
			_storage.settings.aAngles[i] = temp.settings.aAngles[i];
			_storage.settings.cAngles[i] = temp.settings.cAngles[i];
		}
		for(int i=0; i < 4; i++) {
			_storage.settings.uExtras[i] = temp.settings.uExtras[i];
			_storage.settings.dExtras[i] = temp.settings.dExtras[i];
			_storage.settings.lExtras[i] = temp.settings.lExtras[i];
			_storage.settings.rExtras[i] = temp.settings.rExtras[i];
		}
		fresh = true;
	}
}

//Write back to persistent storage
void commitSettings() {
	Persistence::commit(_storage);
}

JumpConfig getJumpSetting() {
	getStoragePage();
	return _storage.settings.jump;
}
void setJumpSetting(const JumpConfig jump) {
	getStoragePage();
	_storage.settings.jump = jump;
}

int getLSetting() {
	getStoragePage();
	return _storage.settings.l;
}
void setLSetting(const int L) {
	getStoragePage();
	_storage.settings.l = L;
}

int getRSetting() {
	getStoragePage();
	return _storage.settings.r;
}
void setRSetting(const int R) {
	getStoragePage();
	_storage.settings.r = R;
}

int getLOffsetSetting() {
	getStoragePage();
	return _storage.settings.lOffset;
}
void setLOffsetSetting(const int lOffset) {
	getStoragePage();
	_storage.settings.lOffset = lOffset;
}

int getROffsetSetting() {
	getStoragePage();
	return _storage.settings.rOffset;
}
void setROffsetSetting(const int rOffset) {
	getStoragePage();
	_storage.settings.rOffset = rOffset;
}

int getCxOffsetSetting() {
	getStoragePage();
	return _storage.settings.cxOffset;
}
void setCOffsetSetting(const int cxOffset) {
	getStoragePage();
	_storage.settings.cxOffset = cxOffset;
}

int getCyOffsetSetting() {
	getStoragePage();
	return _storage.settings.cyOffset;
}
void setCyOffsetSetting(const int cyOffset) {
	getStoragePage();
	_storage.settings.cyOffset = cyOffset;
}

int getXSnapbackSetting() {
	getStoragePage();
	return _storage.settings.xSnapback;
}
void setXSnapbackSetting(const int xSnapback) {
	getStoragePage();
	_storage.settings.xSnapback = xSnapback;
}

int getYSnapbackSetting() {
	getStoragePage();
	return _storage.settings.ySnapback;
}
void setYSnapbackSetting(const int ySnapback) {
	getStoragePage();
	_storage.settings.ySnapback = ySnapback;
}

float getXSmoothingSetting() {
	getStoragePage();
	return _storage.settings.xSmoothing;
}
void setXSmoothingSetting(const float xSmoothing) {
	getStoragePage();
	_storage.settings.xSmoothing = xSmoothing;
}

float getYSmoothingSetting() {
	getStoragePage();
	return _storage.settings.ySmoothing;
}
void setYSmoothingSetting(const float ySmoothing) {
	getStoragePage();
	_storage.settings.ySmoothing = ySmoothing;
}

float getCxSmoothingSetting() {
	getStoragePage();
	return _storage.settings.cxSmoothing;
}
void setCxSmoothingSetting(const float cxSmoothing) {
	getStoragePage();
	_storage.settings.cxSmoothing = cxSmoothing;
}

float getCySmoothingSetting() {
	getStoragePage();
	return _storage.settings.cySmoothing;
}
void setCySmoothingSetting(const float cySmoothing) {
	getStoragePage();
	_storage.settings.cySmoothing = cySmoothing;
}

int getRumbleSetting() {
	getStoragePage();
	return _storage.settings.rumble;
}
void setRumbleSetting(const int rumble) {
	getStoragePage();
	_storage.settings.rumble = rumble;
}

int getAutoInitSetting() {
	getStoragePage();
	return _storage.settings.autoInit;
}
void setAutoInitSetting(const int autoInit) {
	getStoragePage();
	_storage.settings.autoInit = autoInit;
}

int getWaveshapingSetting(const WhichStick whichStick, const WhichAxis whichAxis) {
	getStoragePage();
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			return _storage.settings.axWaveshaping;
		} else {
			return _storage.settings.ayWaveshaping;
		}
	} else {
		if(whichAxis == XAXIS) {
			return _storage.settings.cxWaveshaping;
		} else {
			return _storage.settings.cyWaveshaping;
		}
	}
}

void setWaveshapingSetting(const int waveshaping, const WhichStick whichStick, const WhichAxis whichAxis) {
	getStoragePage();
	if(whichStick == ASTICK) {
		if(whichAxis == XAXIS) {
			_storage.settings.axWaveshaping = waveshaping;
		} else {
			_storage.settings.ayWaveshaping = waveshaping;
		}
	} else {
		if(whichAxis == XAXIS) {
			_storage.settings.cxWaveshaping = waveshaping;
		} else {
			_storage.settings.cyWaveshaping = waveshaping;
		}
	}
}

void getPointsSetting(float points[32], const WhichStick whichStick, const WhichAxis whichAxis) {
	getStoragePage();
	for(int i=0; i<32; i++) {
		if(whichStick == ASTICK) {
			if(whichAxis == XAXIS) {
				points[i] = _storage.settings.axPoints[i];
			} else {
				points[i] = _storage.settings.ayPoints[i];
			}
		} else {
			if(whichAxis == XAXIS) {
				points[i] = _storage.settings.cxPoints[i];
			} else {
				points[i] = _storage.settings.cyPoints[i];
			}
		}
	}
}

void setPointsSetting(const float points[32], const WhichStick whichStick, const WhichAxis whichAxis) {
	getStoragePage();
	for(int i=0; i<32; i++) {
		if(whichStick == ASTICK) {
			if(whichAxis == XAXIS) {
				_storage.settings.axPoints[i] = points[i];
			} else {
				_storage.settings.ayPoints[i] = points[i];
			}
		} else {
			if(whichAxis == XAXIS) {
				_storage.settings.cxPoints[i] = points[i];
			} else {
				_storage.settings.cyPoints[i] = points[i];
			}
		}
	}
}

void getNotchAnglesSetting(float angles[16], const WhichStick whichStick) {
	getStoragePage();
	for(int i=0; i<16; i++) {
		if(whichStick == ASTICK) {
			angles[i] = _storage.settings.aAngles[i];
		} else {
			angles[i] = _storage.settings.cAngles[i];
		}
	}
}

void setNotchAnglesSetting(const float angles[16], const WhichStick whichStick) {
	getStoragePage();
	for(int i=0; i<16; i++) {
		if(whichStick == ASTICK) {
			_storage.settings.aAngles[i] = angles[i];
		} else {
			_storage.settings.cAngles[i] = angles[i];
		}
	}
}

//Extras
int getExtrasSettingInt(const ExtrasSlot slot, const int offset) {
	if(offset < 0 || offset > 3) {
		return 0;
	}
	getStoragePage();
	switch(slot) {
		case EXTRAS_UP:
			return _storage.settings.uExtras[offset].intValue;
		case EXTRAS_DOWN:
			return _storage.settings.dExtras[offset].intValue;
		case EXTRAS_LEFT:
			return _storage.settings.lExtras[offset].intValue;
		case EXTRAS_RIGHT:
			return _storage.settings.rExtras[offset].intValue;
		default:
			return 0;
	}
}
void setExtrasSettingInt(const ExtrasSlot slot, const int offset, const int value) {
	if(offset < 0 || offset > 3) {
		return;
	}
	getStoragePage();
	switch(slot) {
		case EXTRAS_UP:
			_storage.settings.uExtras[offset].intValue = value;
			break;
		case EXTRAS_DOWN:
			_storage.settings.dExtras[offset].intValue = value;
			break;
		case EXTRAS_LEFT:
			_storage.settings.lExtras[offset].intValue = value;
			break;
		case EXTRAS_RIGHT:
			_storage.settings.rExtras[offset].intValue = value;
			break;
		default:
			break;
	}
}
float getExtrasSettingFloat(const ExtrasSlot slot, const int offset) {
	if(offset < 0 || offset > 3) {
		return 0;
	}
	getStoragePage();
	switch(slot) {
		case EXTRAS_UP:
			return _storage.settings.uExtras[offset].floatValue;
		case EXTRAS_DOWN:
			return _storage.settings.dExtras[offset].floatValue;
		case EXTRAS_LEFT:
			return _storage.settings.lExtras[offset].floatValue;
		case EXTRAS_RIGHT:
			return _storage.settings.rExtras[offset].floatValue;
		default:
			return 0;
	}
}
void setExtrasSettingFloat(const ExtrasSlot slot, const int offset, const float value) {
	if(offset < 0 || offset > 3) {
		return;
	}
	getStoragePage();
	switch(slot) {
		case EXTRAS_UP:
			_storage.settings.uExtras[offset].floatValue = value;
			break;
		case EXTRAS_DOWN:
			_storage.settings.dExtras[offset].floatValue = value;
			break;
		case EXTRAS_LEFT:
			_storage.settings.lExtras[offset].floatValue = value;
			break;
		case EXTRAS_RIGHT:
			_storage.settings.rExtras[offset].floatValue = value;
			break;
		default:
			break;
	}
}

