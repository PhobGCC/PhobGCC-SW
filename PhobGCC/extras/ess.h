#ifndef EXTRAS_ESS_H
#define EXTRAS_ESS_H

namespace ess {
	/* This namespace uses GPLv3 Licensed code from Skuzee's ESS Adapter 
	 * code to reverse WiiVC mapping in Ocarina of Time. Please check out      
	 * the source for that here: https://github.com/Skuzee/ESS-Adapter 
	 */

	// ALL COMMENTS REGARDING HOW THIS WORKS HAVE BEEN REMOVED.
	// Please refer to the original code for notes. This code is
	// likely never changing unless we implement a float interpretation.

	ExtrasSlot extrasEssConfigSlot = EXTRAS_UNSET;

	enum EssSettings{
		ESS_SETTING_ENABLE,
		ESS_SETTING_UNUSED1,
		ESS_SETTING_UNUSED2,
		ESS_SETTING_UNUSED3
	};

	enum EssSettingEnable{
		ESS_DISABLED,
		ESS_ENABLED
	};

	const char one_dimensional_map[] = "\x00\x00\x10\x10\x11\x11\x12\x12\x13\x13\x14\x14\x15\x15\x16\x16\x16\x17\x17\x17\x18\x18\x19\x19\x1a\x1a\x1a\x1b\x1b\x1b\x1c\x1c\x1d\x1d\x1d\x1e\x1e\x1e\x1f\x1f  !!!\"\"\"###$$$%%%&&&'''((()))***+++,,,,---...///00001111222333344445555666677778888899999::::;;;;;<<<<<=====>>>>>??????@@@";
	const char triangular_map[] = ",,-,.,.,/,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9,:,:,;,;,<,<,<,=,=,>,>,>,?,?,?,@,--.-.-/-0-0-1-1-2-2-3-3-4-4-5-5-6-6-7-7-8-8-9-9-9-:-:-;-;-<-<-<-=-=->->->-?-?-?-@,..../.0.0.1.1.2.2.3.3.4.4.5.5.6.6.7.7.8.8.9.9.9.:.:.;.;.<.<.<.=.=.>.>.>.?-?-?-?-../.0.0.1.1.2.2.3.3.4.4.5.5.6.6.7.7.8.8.9.9.9.:.:.;.;.<.<.<.=.=.>.>.>.?-?-?-?-//0/0/1/1/2/2/3/3/4/4/5/5/6/6/7/7/8/8/9/9/9/:/:/;/;/</</</=/=/>/>/>/>/>/?-?-000010102020303040405050606070708080909090:0:0;0;0<0<0<0=0=0>/>/>/>/>/>/>/0010102020303040405050606070708080909090:0:0;0;0<0<0<0=0=0=0>/>/>/>/>/>/11112121313141415151616171718181919191:1:1;1;1<1<1<1=0=0=0>/>/>/>/>/>/112121313141415151616171718181919191:1:1;1;1<1<1<1<1<1=0=0>/>/>/>/>/2222323242425252626272728282929292:2:2;2;2<1<1<1<1<1<1=0=0>/>/>/>/22323242425252626272728282929292:2:2;2;2;2<1<1<1<1<1<1<1=0=0>/>/333343435353636373738383939393:3:3;3;3;3;3<1<1<1<1<1<1<1=0=0>/3343435353636373738383939393:3:3;3;3;3;3;3<1<1<1<1<1<1<1<1=044445454646474748484949494:4:4:4;3;3;3;3;3<1<1<1<1<1<1<1<1445454646474748484949494:4:4:4:4;3;3;3;3;3;3<1<1<1<1<1<1555565657575858595959595:4:4:4:4;3;3;3;3;3;3<1<1<1<1<1556565757585859595959595:4:4:4:4;3;3;3;3;3;3<1<1<1<1666676768686869595959595:4:4:4:4;3;3;3;3;3;3;3<1<1667676868686959595959595:4:4:4:4:4;3;3;3;3;3;3<1777777868686959595959595:4:4:4:4:4;3;3;3;3;3;3777777868686869595959595:4:4:4:4:4;3;3;3;3;377777786868686959595959595:4:4:4:4:4;3;3;377777786868686959595959595:4:4:4:4:4;3;377777786868686959595959595:4:4:4:4:4;377777786868686959595959595:4:4:4:4:477777786868686959595959595:4:4:4:47777778686868695959595959595:4:47777778686868695959595959595:4777777868686869595959595959577777786868686959595959595777777868686869595959595777777868686869595959577777786868686869595777777868686868695777777868686868677777786868686777777868686777777868677777786777777777777";

	const uint32_t OOT_MAX = 80;
	const uint32_t BOUNDARY = 39;

	void gc_to_n64(uint8_t coords[2]) {
		uint32_t scale = 5L * coords[0] + 2L * coords[1];
		if (scale > 525) {
			scale = 16UL * 525 * 525 * 525 / scale;
		} else {
			scale = 16 * scale * scale;
		}
		scale *= coords[1];
		scale = scale * 2 / 115;
		scale += 25565300; 
		coords[0] = (coords[0] * scale + 16774000) >> 24;
		coords[1] = (coords[1] * scale + 16774000) >> 24;
	}

	uint16_t triangular_to_linear_index(uint8_t row, uint8_t col, uint8_t size) {
		return (size * (size - 1) / 2) - (size - row) * ((size - row) - 1) / 2 + col;
	}

	void invert_vc(uint8_t coords[2]) {
		if (coords[0] > 2 * OOT_MAX) coords[0] = 2 * OOT_MAX;
		if (coords[1] > 2 * OOT_MAX) coords[1] = 2 * OOT_MAX;
		if (coords[0] >= 2 * BOUNDARY && coords[1] >= 2 * BOUNDARY) {
			uint8_t remainder = OOT_MAX + 1 - BOUNDARY;
			coords[0] = (coords[0] / 2) - BOUNDARY;
			coords[1] = (coords[1] / 2) - BOUNDARY;
			uint16_t index = triangular_to_linear_index(coords[1], coords[0], remainder);
			coords[0] = pgm_read_byte(triangular_map + 2 * index);
			coords[1] = pgm_read_byte(triangular_map + 2 * index + 1);
		} else {
			coords[0] = pgm_read_byte(one_dimensional_map + coords[0]);
			coords[1] = pgm_read_byte(one_dimensional_map + coords[1]);
		}
	}

	void invert_vc_gc(uint8_t coords[2]) {
		int x_positive = 0;
		int y_positive = 0;
		int swap = 0;

		if (coords[0] >= 128) {
			x_positive = 1;
			coords[0] -= 128;
		} else {
			if (coords[0] == 0) coords[0] = 127;
			else coords[0] = 128 - coords[0];
		}

		if (coords[1] >= 128) {
			y_positive = 1;
			coords[1] -= 128;
		} else {
			if (coords[1] == 0) coords[1] = 127;
			else coords[1] = 128 - coords[1];
		}

		if (coords[1] > coords[0]) {
			swap = 1;
			uint8_t temp = coords[0];
			coords[0] = coords[1];
			coords[1] = temp;
		}

		gc_to_n64(coords);
		invert_vc(coords);

		if (swap) {
			uint8_t temp = coords[0];
			coords[0] = coords[1];
			coords[1] = temp;
		}

		if (x_positive) coords[0] += 128;
		else coords[0] = 128 - coords[0];
		if (y_positive) coords[1] += 128;
		else coords[1] = 128 - coords[1];

		// fix bug in LUT, keep origin as origin
		if (coords[0] == _intOrigin + 1)
			coords[0] = _intOrigin;
		if (coords[1] == _intOrigin + 1)
			coords[1] = _intOrigin;
	}

	bool remap(float* Ax, float* Ay, const IntOrFloat config[]){
		if (config[ESS_SETTING_ENABLE].intValue != ESS_ENABLED) {
			return false; //do nothing, keep hyst enabled
		}
		uint8_t remappedESS[2] = {
			(uint8_t) (*Ax + 128),
			(uint8_t) (*Ay + 128)
		};
		invert_vc_gc(remappedESS);
		*Ax = (float) (remappedESS[0] - _intOrigin);
		*Ay = (float) (remappedESS[1] - _intOrigin);
		return true; //disable hyst
	}

	bool toggle(IntOrFloat config[]) {
		int& enabled = config[ESS_SETTING_ENABLE].intValue;
		if (enabled != ESS_DISABLED){
			enabled = ESS_DISABLED;
		} else {
			enabled = ESS_ENABLED;
		}
		setExtrasSettingInt(extrasEssConfigSlot, ESS_SETTING_ENABLE, enabled);
		return (enabled == ESS_ENABLED);
	}
}

#endif //EXTRAS_ESS_H
