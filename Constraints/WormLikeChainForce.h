/*
 * WormLikeChainForce.h
 *
 *  Created on: 21 апр. 2015 г.
 *      Author: yan
 */

#ifndef WORMLIKECHAINFORCE_H_
#define WORMLIKECHAINFORCE_H_

#include "IConstraint.h"


class WormLikeChainForce: public IConstraint
{
public:
	WormLikeChainForce(size_t a_first, size_t a_second, double a_lengthZero, double a_temp, double a_plength, double a_lengthMultipler)
		: first(a_first)
		, second(a_second)
		, lengthZero(a_lengthZero)
		, temperature(a_temp)
		, plength(a_plength)
		, contourLengthMultipler(a_lengthMultipler) {}

	void solve(double time) override;

private:
	size_t first;
	size_t second;
	double lengthZero;
	double temperature;
	double plength;
	double contourLengthMultipler;
};

#endif /* WORMLIKECHAINFORCE_H_ */
