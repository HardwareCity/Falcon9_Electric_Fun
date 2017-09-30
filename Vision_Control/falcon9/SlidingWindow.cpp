#include "SlidingWindow.h"
#include <cassert>

SlidingWindow::SlidingWindow(int n)
	: maxSamples(n)
{
}

SlidingWindow::SlidingWindow(std::deque<float> samples,unsigned int n)
	: maxSamples(n)
{
	assert(n >= samples.size());
	for (unsigned int i = 0; i < samples.size(); ++i)
		this->samples.push_back(samples[i]);
}

SlidingWindow::~SlidingWindow()
{
}

void SlidingWindow::addValue(float v)
{
	samples.push_back(v);
	if(getNumSamples() > maxSamples)
		samples.pop_front();
}

void SlidingWindow::resetSamples(unsigned int n)
{
	assert(n < maxSamples);
	if(getNumSamples() > n)
		samples.erase(samples.begin(), samples.begin() + samples.size() - n);
	//while(samples.size() > n)
	//	samples.pop_front();
}

float SlidingWindow::sum(void)
{
	float sum = 0.0f;
	for (unsigned int i = 0; i < samples.size(); ++i)
	{
		sum += samples[i];
	}
	return sum;
}

float SlidingWindow::min(void)
{
	if(getNumSamples() == 0)
		return 0.0f;

	float ret = samples[0];
	for (unsigned int i = 1; i < samples.size(); ++i)
	{
		if(samples[i] < ret)
			ret = samples[i];
	}
	return ret;
}

float SlidingWindow::max(void)
{
	if(getNumSamples() == 0)
		return 0.0f;

	float ret = samples[0];
	for (unsigned int i = 1; i < samples.size(); ++i)
	{
		if(samples[i] > ret)
			ret = samples[i];
	}
	return ret;
}

/*
float SlidingWindow::max_diff(unsigned int order)
{
	if(getNumSamples() == 0)
		return 0.0f;

	float ret = samples[0];
	for (unsigned int i = 1; i < samples.size(); ++i)
	{
		if(samples[i] > ret)
			ret = samples[i];
	}
	return ret;
}*/

float SlidingWindow::var(void)
{
	if(getNumSamples() == 0)
		return 0.0f;

	float m = mean();
	float var = 0.0;
	for (unsigned int i = 0; i < samples.size(); ++i)
	{
		var += (samples[i] - m)*(samples[i] - m);
	}
	var /= getNumSamples();
	return var;
}

float SlidingWindow::mean(void)
{
	if(getNumSamples() == 0)
		return 0.0f;

	return sum()/getNumSamples();
}

unsigned int SlidingWindow::getNumSamples()
{
	return samples.size();
}

float SlidingWindow::getSample(unsigned int index)
{
	assert(index < getNumSamples());
	return samples[index];
}

void SlidingWindow::setSample(unsigned int index, float v)
{
	assert(index < getNumSamples());
	samples[index] = v;
}

std::deque<float> SlidingWindow::getSamples()
{
	return samples;
}

void SlidingWindow::clear(void)
{
	samples.clear();
}

unsigned int SlidingWindow::getMaxSamples() const
{
    return maxSamples;
}

bool SlidingWindow::isFull()
{
	return getNumSamples() == getMaxSamples();
}
