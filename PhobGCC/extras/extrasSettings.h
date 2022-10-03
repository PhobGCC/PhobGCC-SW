#ifndef EXTRAS_SETTINGS_H
#define EXTRAS_SETTINGS_H

namespace EepromExtras {
	//index values to store data into eeprom, you have 40 words / 160 bytes to work with
	const int _bytesPerFloat = 4;
	//add in extras settings below
	const int _eepromEssEnable = 0;
	const int _eepromDummyValue = _eepromEssEnable+_bytesPerFloat;//Example usage, will remove later
};

//Extras framework functions for resetting defaults and reading EEPROM 

void extrasResetDefaults(HardReset reset, ExtrasConfig &extrasConfig) {
	//ESS
	extrasConfig.essEnable = EXTRAS_ESS_DISABLED;
	setExtrasSettingInt(EepromExtras::_eepromEssEnable, extrasConfig.essEnable);

	if(reset == HARD){
		//pass
	}
}

int extrasReadEEPROM(ExtrasConfig &extrasConfig) {
	int numberOfNaN = 0;

	//ESS
	extrasConfig.essEnable = (ExtrasEssConfig)getExtrasSettingInt(EepromExtras::_eepromEssEnable);
	if (extrasConfig.essEnable < EXTRAS_ESS_DISABLED){
		extrasConfig.essEnable = EXTRAS_ESS_DISABLED;
		numberOfNaN++;
	} else if (extrasConfig.essEnable > EXTRAS_ESS_ENABLED){
		extrasConfig.essEnable = EXTRAS_ESS_DISABLED;
		numberOfNaN++;
	}

	return numberOfNaN;
}

#endif //EXTRAS_SETTINGS_H