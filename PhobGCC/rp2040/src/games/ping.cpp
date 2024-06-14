#include <cmath>
using std::min;
using std::max;

#include "cvideo.h"
#include "cvideo_variables.h"
#include "structsAndEnums.h"
#include "games/ping.h"

#define FIELDX 512
#define FIELDY 300
#define FIELDYOFFSET 83
#define PADDLEWIDTH 25
#define BALLSIZE 4
#define PADDLESPEED 18.0f

#define OUTSIDEMARGIN 12
#define INSIDEMARGIN 20

#define MINLEFTX (OUTSIDEMARGIN)
#define MAXLEFTX (FIELDX/2 - INSIDEMARGIN)
#define MINRIGHTX (FIELDX/2 + INSIDEMARGIN)
#define MAXRIGHTX (FIELDX - OUTSIDEMARGIN)
#define MINPADDLEY (PADDLEWIDTH/2.0F)
#define MAXPADDLEY (FIELDY - PADDLEWIDTH/2.0F)


float clamp(const float minimum, const float maximum, const float input) {
    return max(minimum, min(maximum, input));
}

void runPing(unsigned char *bitmap, const Buttons hardware, const RawStick raw, const ControlConfig config, volatile uint8_t &pleaseCommit) {
	//process raw sticks so that they all are scaled equally, with the max being 1
	//use raw because we don't want cardinal snapping
	//snapback should not be a concern
	const float aScale = 1.0f / config.astickAnalogScaler;
	const float cScale = 1.0f / config.cstickAnalogScaler;
	const float axRaw = raw.axUnfiltered * aScale;
	const float ayRaw = raw.ayUnfiltered * aScale;
	const float cxRaw = raw.cxUnfiltered * cScale;
	const float cyRaw = raw.cyUnfiltered * cScale;
	//add just a center deadzone of 3 units radius
    const float threshold = (3.0f*3.0f)/(100.0f*100.0f);
	const bool aCentered = (axRaw*axRaw + ayRaw*ayRaw) < threshold;
	const bool cCentered = (cxRaw*cxRaw + cyRaw*cyRaw) < threshold;
	const float ax = aCentered ? 0 : axRaw;
	const float ay = aCentered ? 0 : ayRaw;
	const float cx = cCentered ? 0 : cxRaw;
	const float cy = cCentered ? 0 : cyRaw;

    //paddle locations
    static float leftX = MINLEFTX;
    static float leftY = FIELDY/2;
    static float rightX = MAXRIGHTX;
    static float rightY = FIELDY/2;
    
    //erase old paddles
    static uint16_t lx = round(leftX);
    static uint16_t ly0 = round(leftY + FIELDYOFFSET);
    static uint16_t ly1 = round(leftY + FIELDYOFFSET + PADDLEWIDTH);
    static uint16_t rx = round(rightX);
    static uint16_t ry0 = round(rightY + FIELDYOFFSET);
    static uint16_t ry1 = round(rightY + FIELDYOFFSET + PADDLEWIDTH);
    drawLine(bitmap, lx, ly0, lx, ly1, BLACK);
    drawLine(bitmap, rx, ry0, rx, ry1, BLACK);

    //ball location
    static float ballX = MINLEFTX

    //update paddle locations
    leftX = clamp(MINLEFTX, MAXLEFTX, leftX + PADDLESPEED * ax);
    leftY = clamp(MINPADDLEY, MAXPADDLEY, leftY + PADDLESPEED * -ay);
    rightX = clamp(MINRIGHTX, MAXRIGHTX, rightX + PADDLESPEED * cx);
    rightY = clamp(MINPADDLEY, MAXPADDLEY, rightY + PADDLESPEED * -cy);

    //offset and round to integer locations
    lx = round(leftX);
    ly0 = round(leftY + FIELDYOFFSET - PADDLEWIDTH/2);
    ly1 = round(leftY + FIELDYOFFSET + PADDLEWIDTH/2);
    rx = round(rightX);
    ry0 = round(rightY + FIELDYOFFSET - PADDLEWIDTH/2);
    ry1 = round(rightY + FIELDYOFFSET + PADDLEWIDTH/2);

    //draw paddles
    drawLine(bitmap, lx, ly0, lx, ly1, WHITE);
    drawLine(bitmap, rx, ry0, rx, ry1, WHITE);
}

