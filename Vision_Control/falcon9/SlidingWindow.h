#ifndef SLIDINGWINDOW_H_
#define SLIDINGWINDOW_H_

#include <deque>

class SlidingWindow
{
public:
	SlidingWindow(int n);
	SlidingWindow(std::deque<float> samples,unsigned int n);
	virtual ~SlidingWindow();
	void addValue(float v);
	void resetSamples(unsigned int n);
	float sum(void);
	float mean(void);
	float var(void);
	float min(void);
	float max(void);

	/**
	 * order = 1, normal difference
	 * order = 2, calculate the max differences every 2 samples
	 * order = 3, ...
	 */
	//float max_diff(unsigned int order = 1);

	unsigned int getNumSamples();
	float getSample(unsigned int index);
	void setSample(unsigned int index, float v);
	std::deque<float> getSamples();
	void clear(void);
    unsigned int getMaxSamples() const;
    bool isFull(void);

private:
	const unsigned int maxSamples;
	std::deque<float> samples;

};

#endif /* SLIDINGWINDOW_H_ */
