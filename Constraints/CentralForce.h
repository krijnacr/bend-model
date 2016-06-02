/*
 * CentralForce.h
 *
 *  Created on: 5 мая 2016 г.
 *      Author: yan
 */

#ifndef CONSTRAINTS_CENTRALFORCE_H_
#define CONSTRAINTS_CENTRALFORCE_H_


#include <vector>
#include "IConstraint.h"


class CentralForce: public IConstraint
{
public:
	CentralForce(const glm::dvec3& a_center, double a_smallRadius, double a_bigRadius, double a_multipler)
		: center(a_center)
		, smallRadius(a_smallRadius)
		, bigRadius(a_bigRadius)
		, multipler(a_multipler) {}

	void solve(double time) override;

private:
	glm::dvec3 center;
	double smallRadius;
	double bigRadius;
	double multipler;
};

#endif /* CONSTRAINTS_CENTRALFORCE_H_ */
