#ifndef STORAGE_H
#define STORAGE_H

#include "storage/page_indexes.hpp"
#include "stick.h"
#include "structsAndEnums.h"

namespace Persistence {
namespace Pages {

struct Storage {
	static const int index = (int) PageIndexes::STORAGE;

	struct __attribute__((packed)) Settings {
		JumpConfig jump;
		int l;
		int r;
		int lOffset;
		int rOffset;
		int cxOffset;
		int cyOffset;
		int xSnapback;
		int ySnapback;
		float xSmoothing;
		float ySmoothing;
		float cxSmoothing;
		float cySmoothing;
		int rumble;
		int autoInit;
		int axWaveshaping;
		int ayWaveshaping;
		int cxWaveshaping;
		int cyWaveshaping;
		float axPoints[32];
		float ayPoints[32];
		float cxPoints[32];
		float cyPoints[32];
		float aAngles[16];
		float cAngles[16];
		IntOrFloat uExtras[4];
		IntOrFloat dExtras[4];
		IntOrFloat lExtras[4];
		IntOrFloat rExtras[4];
	} settings;
};

}
}

//the code must commit settings after making any change or eles it won't get written
void commitSettings();

JumpConfig getJumpSetting();
void setJumpSetting(const JumpConfig);

int  getLSetting();
void setLSetting(const int);

int  getRSetting();
void setRSetting(const int);

int  getLOffsetSetting();
void setLOffsetSetting(const int);

int  getROffsetSetting();
void setROffsetSetting(const int);

int  getCxOffsetSetting();
void setCxOffsetSetting(const int);

int  getCyOffsetSetting();
void setCyOffsetSetting(const int);

int  getXSnapbackSetting();
void setXSnapbackSetting(const int);

int  getYSnapbackSetting();
void setYSnapbackSetting(const int);

float getXSmoothingSetting();
void  setXSmoothingSetting(const float);

float getYSmoothingSetting();
void  setYSmoothingSetting(const float);

float getCxSmoothingSetting();
void  setCxSmoothingSetting(const float);

float getCySmoothingSetting();
void  setCySmoothingSetting(const float);

int  getRumbleSetting();
void setRumbleSetting(const int);

int  getAutoInitSetting();
void setAutoInitSetting(const int);

int  getWaveshapingSetting(const WhichStick, const WhichAxis);
void setWaveshapingSetting(const int, const WhichStick, const WhichAxis);

void getPointsSetting(float [32], const WhichStick, const WhichAxis);
void setPointsSetting(const float [32], const WhichStick, const WhichAxis);

void getNotchAnglesSetting(float [16], const WhichStick);
void setNotchAnglesSetting(const float [16], const WhichStick);

int   getExtrasSettingInt(const ExtrasSlot, const int);
void  setExtrasSettingInt(const ExtrasSlot, const int, const int);
float getExtrasSettingFloat(const ExtrasSlot, const int);
void  setExtrasSettingFloat(const ExtrasSlot, const int, const float);

#endif //STORAGE_H

