#ifndef STORAGE_H
#define STORAGE_H

#include "storage/page_indexes.hpp"
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
		int xSmoothing;
		int ySmoothing;
		int cxSmoothing;
		int cySmoothing;
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
		int schema;
		int AstickCardinalSnapping;
		int CstickCardinalSnapping;
		int AstickAnalogScaler;
		int CstickAnalogScaler;
		int interlaceOffset;
		int tournamentToggle;
	} settings;
};

}
}

//the code must commit settings after making any change or else it won't get written
void commitSettings(const bool noLock = false);

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

int  getXSmoothingSetting();
void setXSmoothingSetting(const int);

int  getYSmoothingSetting();
void setYSmoothingSetting(const int);

int  getCxSmoothingSetting();
void setCxSmoothingSetting(const int);

int  getCySmoothingSetting();
void setCySmoothingSetting(const int);

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

int  getSchemaSetting();
void setSchemaSetting(const int);

int  getCardinalSnappingSetting(const WhichStick);
void setCardinalSnappingSetting(const int, const WhichStick);

int  getAnalogScalerSetting(const WhichStick);
void setAnalogScalerSetting(const int, const WhichStick);

int getInterlaceOffsetSetting();
void setInterlaceOffsetSetting(const int);

int getTournamentToggleSetting();
void setTournamentToggleSetting(const int);

#endif //STORAGE_H
