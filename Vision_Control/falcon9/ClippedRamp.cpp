#include "ClippedRamp.h"

ClippedRamp::ClippedRamp() {
	calcInternal(0,0,1,1);
	clipHighY = 1.0;
	clipLowY = 0.0;
}

ClippedRamp::ClippedRamp(float x1, float y1, float x2, float y2, float clipLowY, float clipHighY){
	this->clipLowY = clipLowY;
	this->clipHighY = clipHighY;
	calcInternal(x1,y1, x2,y2);
}

ClippedRamp::~ClippedRamp() {
	// TODO Auto-generated destructor stub
}

void ClippedRamp::calcInternal(float x1, float y1, float x2, float y2)
{
	m = (y2-y1) / (x2-x1);
	b = y1 - m*x1;
}

float ClippedRamp::getValue(float x)
{
	float ret = m*x + b;
	ret = (ret > clipHighY ? clipHighY : ret);
	ret = (ret < clipLowY ? clipLowY : ret);
	return ret;
}

