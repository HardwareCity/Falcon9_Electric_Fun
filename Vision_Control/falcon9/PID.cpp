#include <syslog.h>
#include "PID.h"
#include "math.h"

PID::PID( float kp , float ki , float kd , float minOut, float maxOut , float maxInt ) 
{
	this->kp		= kp;
	this->ki		= ki; 
	this->kd		= kd;
	this->minOut	= minOut;
	this->maxOut	= maxOut;
	this->maxInt	= maxInt;

	this->lastValue	= 0.0;
	this->integral	= 0.0;
	this->firstTime	= true;
	this->used 		= false;
}

PID::PID( const PID& pid ) 
{
	this->kp		= pid.kp;
	this->ki		= pid.ki; 
	this->kd		= pid.kd;
	this->minOut	= pid.minOut;
	this->maxOut	= pid.maxOut;
	this->maxInt	= pid.maxInt;

	this->lastValue	= pid.lastValue;
	this->integral	= pid.integral;
	this->firstTime	= true;
}

float PID::compensate( float value )
{

	if(isinf(value) || isnan(value)) {
		value = 0.0;
		syslog(LOG_ERR, "Compensator value NAN or INF\n");
	}

	used = true; // this compensator is used this cycle

	float out = 0.0;

	integral += value;

	if(integral > maxInt)
		integral = maxInt;
	if(integral < -maxInt)
		integral = -maxInt;

	if( firstTime )
	{
		integral = 0;
		out = kp*value;
		firstTime = false;
	}
	else
	{
/*
		if(fabs(value) < 0.001)
			out = kp*value + ki*integral;
		else
			out = kp*value + kd*(value - lastValue)/value + ki*integral;
*/
		out = kp*value + kd*(value - lastValue) + ki*integral;

	}

	if( out > maxOut )
		out = maxOut;
	else if( out < -maxOut )
		out = -maxOut;
	

	lastValue = value;

	return out;
}

float PID::compensate2( float value )
{
	used = true; // this compensator is used this cycle

	float out = 0.0;

	if(value < 0.001) return 0.0;

	if( firstTime )
	{
		integral = 0;
		out = kp*value;
		firstTime = false;
	}
	else
		out = kp*value + kd*(value - lastValue) / value;

	if( out > maxOut )
		out = maxOut;
	else
		if( out < -maxOut )
			out = -maxOut;

	if( out < minOut && out > 0.0 )
		out = minOut;
	else
		if( out > -minOut && out < 0.0 )
			out = -minOut;

	lastValue = value;

	return out;
}

void PID::check()
{
	if(!used)
	{
		reset();
	}
	used = false;
}

void PID::reset()
{
	firstTime = true;
}

void PID::display() 
{
	printf("  p(%.2f) i(%.2f) d(%.2f) maxOut(%.2f) maxInt(%.2f)\n", kp, ki, kd, maxOut, maxInt);	
}
