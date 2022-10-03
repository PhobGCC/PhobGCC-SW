#ifndef EXTRAS_SETTINGS_H
#define EXTRAS_SETTINGS_H

namespace EepromExtras {
	//index values to store data into eeprom
	const int _bytesPerFloat = 4;
	const int _eepromStart = Eeprom::_eepromExtras;
	//add in extras settings below
	const int _eepromEssEnable = _eepromStart;
	const int _eepromDummyValue = _eepromEssEnable+_bytesPerFloat; //Example usage, will remove later
};

//ESS settings
void setEssSetting(const ExtrasEssConfig enable){
	EEPROM.put(EepromExtras::_eepromEssEnable, enable);
}

ExtrasEssConfig getEssSetting(){
	ExtrasEssConfig output;
	EEPROM.get(EepromExtras::_eepromEssEnable, output);
	return output;
}

//Extras framework functions for resetting defaults and reading EEPROM 

void extrasResetDefaults(HardReset reset, ExtrasConfig &extrasConfig) {
	//ESS
	extrasConfig.essEnable = EXTRAS_ESS_DISABLED;
	setEssSetting(extrasConfig.essEnable);

	if(reset == HARD){
		//pass
	}
}

int extrasReadEEPROM(ExtrasConfig &extrasConfig) {
	int numberOfNaN = 0;

	//ESS
	extrasConfig.essEnable = getEssSetting();
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