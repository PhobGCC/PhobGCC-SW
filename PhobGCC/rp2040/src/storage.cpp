#include "storage/pages/storage.h"
#include "storage/functions.hpp"

static volatile Persistence::Pages::Storage _storage;
static volatile bool fresh = false;

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
			_storage.settings.uExtras[i].intValue = temp.settings.uExtras[i].intValue;
			_storage.settings.dExtras[i].intValue = temp.settings.dExtras[i].intValue;
			_storage.settings.lExtras[i].intValue = temp.settings.lExtras[i].intValue;
			_storage.settings.rExtras[i].intValue = temp.settings.rExtras[i].intValue;
		}
		_storage.settings.schema = temp.settings.schema;
		_storage.settings.AstickCardinalSnapping = temp.settings.AstickCardinalSnapping;
		_storage.settings.CstickCardinalSnapping = temp.settings.CstickCardinalSnapping;
		_storage.settings.AstickAnalogScaler = temp.settings.AstickAnalogScaler;
		_storage.settings.CstickAnalogScaler = temp.settings.CstickAnalogScaler;
		_storage.settings.interlaceOffset = temp.settings.interlaceOffset;
		_storage.settings.tournamentToggle = temp.settings.tournamentToggle;
		fresh = true;
	}
}

//Write back to persistent storage
void commitSettings(const bool noLock/* = false*/) {
	Persistence::commit(_storage, noLock);
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
void setCxOffsetSetting(const int cxOffset) {
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

int getXSmoothingSetting() {
	getStoragePage();
	return _storage.settings.xSmoothing;
}
void setXSmoothingSetting(const int xSmoothing) {
	getStoragePage();
	_storage.settings.xSmoothing = xSmoothing;
}

int getYSmoothingSetting() {
	getStoragePage();
	return _storage.settings.ySmoothing;
}
void setYSmoothingSetting(const int ySmoothing) {
	getStoragePage();
	_storage.settings.ySmoothing = ySmoothing;
}

int getCxSmoothingSetting() {
	getStoragePage();
	return _storage.settings.cxSmoothing;
}
void setCxSmoothingSetting(const int cxSmoothing) {
	getStoragePage();
	_storage.settings.cxSmoothing = cxSmoothing;
}

int getCySmoothingSetting() {
	getStoragePage();
	return _storage.settings.cySmoothing;
}
void setCySmoothingSetting(const int cySmoothing) {
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

int getSchemaSetting() {
	getStoragePage();
	return _storage.settings.schema;
}

void setSchemaSetting(const int s) {
	getStoragePage();
	_storage.settings.schema = s;
}

int getCardinalSnappingSetting(const WhichStick whichStick) {
	getStoragePage();
	if(whichStick == ASTICK) {
		return _storage.settings.AstickCardinalSnapping;
	} else {
		return _storage.settings.CstickCardinalSnapping;
	}
}

void setCardinalSnappingSetting(const int cardinalSnapping, const WhichStick whichStick) {
	getStoragePage();
	if(whichStick == ASTICK) {
		_storage.settings.AstickCardinalSnapping = cardinalSnapping;
	} else {
		_storage.settings.CstickCardinalSnapping = cardinalSnapping;
	}
}

int getAnalogScalerSetting(const WhichStick whichStick) {
	getStoragePage();
	if(whichStick == ASTICK) {
		return _storage.settings.AstickAnalogScaler;
	} else {
		return _storage.settings.CstickAnalogScaler;
	}
}

void setAnalogScalerSetting(const int analogScaler, const WhichStick whichStick) {
	getStoragePage();
	if(whichStick == ASTICK) {
		_storage.settings.AstickAnalogScaler = analogScaler;
	} else {
		_storage.settings.CstickAnalogScaler = analogScaler;
	}
}

int getInterlaceOffsetSetting() {
	getStoragePage();
	return _storage.settings.interlaceOffset;
}

void setInterlaceOffsetSetting(const int o) {
	getStoragePage();
	_storage.settings.interlaceOffset = o;
}

int getTournamentToggleSetting() {
	getStoragePage();
	return _storage.settings.tournamentToggle;
}

void setTournamentToggleSetting(const int tournamentToggle) {
	getStoragePage();
	_storage.settings.tournamentToggle = tournamentToggle;
}
