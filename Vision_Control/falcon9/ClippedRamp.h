#ifndef CLIPPEDRAMP_H_
#define CLIPPEDRAMP_H_

/**
 * Creates a clipped ramp profile, given 2 points of the line and maximum and minimum clipping values
 */
class ClippedRamp {
public:
	ClippedRamp();
	ClippedRamp(float x1, float y1, float x2, float y2, float clipLowY, float clipHighY);
	virtual ~ClippedRamp();

	float getValue(float x);

private:
	void calcInternal(float x1, float y1, float x2, float y2);
	float m; // slope
	float b; // offset
	float clipLowY, clipHighY; // clip values
};

#endif /* CLIPPEDRAMP_H_ */
