#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>

#include "../common/stick.h"

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
};

#include "../common/structsAndEnums.h"

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

#endif //SETTINGS_H
