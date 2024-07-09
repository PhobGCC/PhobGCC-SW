#include <cmath>
using std::min;
using std::max;

#include "cvideo.h"
#include "cvideo_variables.h"
#include "structsAndEnums.h"
#include "games/ping.h"

#define FIELDX 511
#define FIELDY 300
#define FIELDYOFFSET 83
#define PADDLEWIDTH 40
#define BALLSIZE 2
#define PADDLESPEED 6.0f
#define BALLSPEEDLIMIT 9.0f

#define OUTSIDEMARGIN 12
#define INSIDEMARGIN 20

#define MINLEFTX (OUTSIDEMARGIN)
#define MAXLEFTX (FIELDX/2 - INSIDEMARGIN)
#define MINRIGHTX (FIELDX/2 + INSIDEMARGIN)
#define MAXRIGHTX (FIELDX - OUTSIDEMARGIN)
#define MINPADDLEY (PADDLEWIDTH/2.0f)
#define MAXPADDLEY (FIELDY - PADDLEWIDTH/2.0f)

#define MINBALLX (BALLSIZE/2.0f)
#define MAXBALLX (FIELDX - BALLSIZE/2.0f)
#define MINBALLY (BALLSIZE/2.0f)
#define MAXBALLY (FIELDY - BALLSIZE/2.0f)


float clamp(const float minimum, const float maximum, const float input) {
    return max(minimum, min(maximum, input));
}

void collisionDetectPaddle(
        const float paddleX0,
        const float paddleY0,
        const float paddleX1,
        const float paddleY1,
        const float ballX0,
        const float ballY0,
        float &ballX1,
        float &ballY1,
        float &ballDX,
        float &ballDY,
        float &ballSpin) {
    const float paddleDX = paddleX1 - paddleX0;
    const float paddleDY = paddleY1 - paddleY0;

    //calculate relative position and velocity
    const float relX0 = paddleX0 - ballX0;
    const float relY0 = paddleY0 - ballY0;
    const float relX1 = paddleX1 - ballX1;
    const float relY1 = paddleY1 - ballY1;

    const float relDX = paddleDX - ballDX;
    const float relDY = paddleDY - ballDY;

    //check for whether it crosses the x coordinate of the paddle (rel x crosses zero)
    float crossTime = -1; //[0 1) will count for crossing
    if(relX0 == 0.0f) {
        crossTime = 0;
    } else if(relX0 < 0.0f) {
        if(relX1 > 0.0f) {
            crossTime = relX0 / (relX0 - relX1);
        }
    } else {
        if(relX1 < 0.0f) {
            crossTime = relX0 / (relX0 - relX1);
        }
    }

    //no crossing = no change in trajectory
    if(crossTime == -1) {
        return;
    }

    //calculate the y-intercept and see if it's within the paddle width
    const float relYCross = crossTime*relDY + relY0;
    if(abs(relYCross) > PADDLEWIDTH/2) {
        //it's not within the paddle width
        return;
    }

    //we now know it intersected
    //find the point of contact using the old velocity
    const float crossX = ballX0 + crossTime*ballDX;
    const float crossY = ballY0 + crossTime*ballDY;

    //set up the new velocity
    ballDX = 2*paddleDX - ballDX;
    ballDY = paddleDY + ballDY;

    //cap the speed
    const float speedfactor = min(1.0, BALLSPEEDLIMIT / sqrt(ballDX*ballDX + ballDY*ballDY));
    ballDX *= speedfactor;
    ballDY *= speedfactor;

    //set the position after contact
    ballX1 = crossX + (1-crossTime)*ballDX;
    ballY1 = crossY + (1-crossTime)*ballDY;
}

void collisionDetectWall(
        const float ballX0,
        const float ballY0,
        float &ballX1,
        float &ballY1,
        float &ballDX,
        float &ballDY,
        float &ballSpin) {
    //calculate position relative to the boundary
    const float relY0Top = ballY0 - MINBALLY;
    const float relY1Top = ballY1 - MINBALLY;
    const float relY0Bot = ballY0 - MAXBALLY;
    const float relY1Bot = ballY1 - MAXBALLY;

    //check whether it crosses
    float crossTime = -1;
    if(relY1Top < 0.0f) {
        crossTime = -relY0Top / ballDY;
    } else if(relY1Bot > 0.0f) {
        crossTime = -relY0Bot / ballDY;
    }

    //no crossing = no change in trajectory
    if(crossTime == -1) {
        return;
    }

    //we now know it intersected
    //find the point of contact using the old velocity
    const float crossX = ballX0 + crossTime*ballDX;
    const float crossY = ballY0 + crossTime*ballDY;

    //set up the new velocity
    ballDY = -ballDY;

    //set the position after contact
    ballX1 = crossX + (1-crossTime)*ballDX;
    ballY1 = crossY + (1-crossTime)*ballDY;
}

bool runPing(unsigned char *bitmap, const Buttons hardware, const RawStick raw, const ControlConfig config) {
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

    //ball location
    static float ballX = MINLEFTX + 50;
    static float ballY = FIELDY/2;
    static float ballDX = 0;
    static float ballDY = 0;
    static float ballSpin = 0;

    //handle point sfx
    static uint8_t framesSinceScore = 0;
    static bool gameIsOver = false;
    static uint8_t leftScore = 0;
    static uint8_t rightScore = 0;
    if(framesSinceScore > 1) {
        framesSinceScore--;
        const uint16_t point1x = (60-framesSinceScore) * 1;
        const uint16_t point1y = (60-framesSinceScore) * 3;
        const uint16_t point2x = ((60-framesSinceScore) * 3) << 1;
        const uint16_t point2y = (60 - framesSinceScore) * 2;
        const uint16_t point3x = (60 - framesSinceScore) * 4;
        const uint16_t point3y = (60 - framesSinceScore) * 1;
        const uint16_t point4x = (60 - framesSinceScore) * 2;
        const uint16_t point4y = ((framesSinceScore - 60) * 3) << 1;
        const uint16_t point5x = (60 - framesSinceScore) * 3;
        const uint16_t point5y = ((framesSinceScore - 60) * 7) << 2;
        const uint16_t point6x = ((60 - framesSinceScore) * 5) << 1;
        const uint16_t point6y = ((framesSinceScore - 60) * 1) << 1;
        const uint16_t ballXint = round(min(511.0f, ballX));
        const uint16_t ballYint = round(ballY + FIELDYOFFSET);
        if(ballX < FIELDX/2) {
            //draw rightward lines from ball position
            drawLine(bitmap, ballXint, ballYint, ballXint + point1x, ballYint + point1y, WHITE);
            drawLine(bitmap, ballXint, ballYint, ballXint + point2x, ballYint + point2y, WHITE);
            drawLine(bitmap, ballXint, ballYint, ballXint + point3x, ballYint + point3y, WHITE);
            drawLine(bitmap, ballXint, ballYint, ballXint + point4x, ballYint + point4y, WHITE);
            drawLine(bitmap, ballXint, ballYint, ballXint + point5x, ballYint + point5y, WHITE);
            drawLine(bitmap, ballXint, ballYint, ballXint + point6x, ballYint + point6y, WHITE);
        } else {
            //draw leftward lines from ball position
            drawLine(bitmap, ballXint - point1x, ballYint + point1y, ballXint, ballYint, WHITE);
            drawLine(bitmap, ballXint - point2x, ballYint + point2y, ballXint, ballYint, WHITE);
            drawLine(bitmap, ballXint - point3x, ballYint + point3y, ballXint, ballYint, WHITE);
            drawLine(bitmap, ballXint - point4x, ballYint + point4y, ballXint, ballYint, WHITE);
            drawLine(bitmap, ballXint - point5x, ballYint + point5y, ballXint, ballYint, WHITE);
            drawLine(bitmap, ballXint - point6x, ballYint + point6y, ballXint, ballYint, WHITE);
        }
        return false;//don't accept any input until this is done
    } else if(framesSinceScore == 1) {
        framesSinceScore--;
        //erase everything
        eraseRows(bitmap, 0, 384);
        //reset ball
        if(ballX < FIELDX/2) {
            ballX = MINLEFTX + 50;
            ballY = FIELDY/2;
            ballDX = 0;
            ballDY = 0;
            ballSpin = 0;
        } else {
            ballX = MAXRIGHTX - 50;
            ballY = FIELDY/2;
            ballDX = 0;
            ballDY = 0;
            ballSpin = 0;
        }
        //reset paddles
        leftX = MINLEFTX;
        leftY = FIELDY/2;
        rightX = MAXRIGHTX;
        rightY = FIELDY/2;
        if(gameIsOver) {
            gameIsOver = false;
            leftScore = 0;
            rightScore = 0;
        }
    }//otherwise continue
    
    //erase old paddles
    static uint16_t lx = round(leftX);
    static uint16_t ly0 = round(leftY + FIELDYOFFSET);
    static uint16_t ly1 = round(leftY + FIELDYOFFSET + PADDLEWIDTH);
    static uint16_t rx = round(rightX);
    static uint16_t ry0 = round(rightY + FIELDYOFFSET);
    static uint16_t ry1 = round(rightY + FIELDYOFFSET + PADDLEWIDTH);
    drawLine(bitmap, lx, ly0, lx, ly1, BLACK);
    drawLine(bitmap, rx, ry0, rx, ry1, BLACK);

    //erase old ball
    static uint16_t bx0 = round(ballX - BALLSIZE/2);
    static uint16_t bx1 = round(ballX + BALLSIZE/2);
    static uint16_t by0 = round(ballY + FIELDYOFFSET - BALLSIZE/2);
    static uint16_t by1 = round(ballY + FIELDYOFFSET + BALLSIZE/2);
    drawLine(bitmap, bx0, by0, bx0, by1, BLACK);
    drawLine(bitmap, bx1, by0, bx1, by1, BLACK);

    //update paddle locations
    //save old paddle locations for collision detection
    const float oldLeftX = leftX;
    const float oldLeftY = leftY;
    const float oldRightX = rightX;
    const float oldRightY = rightY;
    leftX = clamp(MINLEFTX, MAXLEFTX, leftX + PADDLESPEED * ax);
    leftY = clamp(MINPADDLEY, MAXPADDLEY, leftY + PADDLESPEED * -ay);
    rightX = clamp(MINRIGHTX, MAXRIGHTX, rightX + PADDLESPEED * cx);
    rightY = clamp(MINPADDLEY, MAXPADDLEY, rightY + PADDLESPEED * -cy);

    //update ball location
    //save old ball locations for collision detection
    const float oldBallX = ballX;
    const float oldBallY = ballY;
    ballX = ballX + ballDX;
    ballY = ballY + ballDY;

    //check for paddle collision
    collisionDetectPaddle(oldLeftX, oldLeftY, leftX, leftY, oldBallX, oldBallY, ballX, ballY, ballDX, ballDY, ballSpin);
    collisionDetectPaddle(oldRightX, oldRightY, rightX, rightY, oldBallX, oldBallY, ballX, ballY, ballDX, ballDY, ballSpin);

    //check for wall collision
    collisionDetectWall(oldBallX, oldBallY, ballX, ballY, ballDX, ballDY, ballSpin);

    //handle point scoring
    if(ballX < MINBALLX) {
        //erase old score
        eraseRows(bitmap, 20, 30);
        //update score
        rightScore++;
        framesSinceScore = 60;
    } else if(ballX > MAXBALLX) {
        //erase old score
        eraseRows(bitmap, 20, 30);
        //update score
        leftScore++;
        framesSinceScore = 60;
    }

    if((framesSinceScore == 60) && (leftScore >= 11) && (leftScore > (rightScore + 1))) {
        //left wins
        drawString2x(bitmap, 165, 30, 15, "Left Wins!");
        gameIsOver = true;
    } else if((framesSinceScore == 60) && (rightScore >= 11) && rightScore > (leftScore + 1)) {
        //right wins
        drawString2x(bitmap, 155, 30, 15, "Right Wins!");
        gameIsOver = true;
    }

    //offset and round to integer locations
    lx = round(leftX);
    ly0 = round(leftY + FIELDYOFFSET - PADDLEWIDTH/2);
    ly1 = round(leftY + FIELDYOFFSET + PADDLEWIDTH/2);
    rx = round(rightX);
    ry0 = round(rightY + FIELDYOFFSET - PADDLEWIDTH/2);
    ry1 = round(rightY + FIELDYOFFSET + PADDLEWIDTH/2);
    bx0 = round(ballX - BALLSIZE/2);
    bx1 = round(ballX + BALLSIZE/2);
    by0 = round(ballY + FIELDYOFFSET - BALLSIZE/2);
    by1 = round(ballY + FIELDYOFFSET + BALLSIZE/2);

    //draw scores
    drawInt2x(bitmap,   0, 20, 10, 0, leftScore);
    drawInt2x(bitmap, 420, 20, 10, 2, rightScore);

    //draw play field boundary
    drawLine(bitmap, 0, FIELDYOFFSET-1, FIELDX-1, FIELDYOFFSET-1, GRAY);//top
    drawLine(bitmap, MAXLEFTX, FIELDYOFFSET, MAXLEFTX, VHEIGHT-1, GRAY);//left "net"
    drawLine(bitmap, MINRIGHTX, FIELDYOFFSET, MINRIGHTX, VHEIGHT-1, GRAY);//right "net"

    //draw paddles
    drawLine(bitmap, lx, ly0, lx, ly1, WHITE);
    drawLine(bitmap, rx, ry0, rx, ry1, WHITE);

    //draw ball
    drawLine(bitmap, bx0, by0, bx0, by1, WHITE);
    drawLine(bitmap, bx1, by0, bx1, by1, WHITE);

    static uint8_t backAccumulator = 0;
    if(hardware.B) {
        backAccumulator++;
    } else {
        backAccumulator = max(0, backAccumulator - 1);
    }

    if(backAccumulator >= 30) {
        leftScore = 0;
        rightScore = 0;
        return true;//request that we go back
    }
    return false;
}

