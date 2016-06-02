/*
 * SpringForce.h
 *
 *  Created on: 14 апр. 2015 г.
 *      Author: yan
 */

#ifndef SPRINGFORCE_H_
#define SPRINGFORCE_H_

#include "IConstraint.h"


class SpringForce: public IConstraint
{
public:
    SpringForce(size_t a_first, size_t a_second, double a_length, double a_stiffness)
        : first(a_first)
		, second(a_second)
		, length(a_length)
		, stiffness(a_stiffness) {}

	void solve(double time) override;

private:
	size_t first;
	size_t second;
    double length;
    double stiffness;
};

#endif /* SPRINGFORCE_H_ */
