/*
 * SingleNodeConstantForce.h
 *
 *  Created on: 14 апр. 2015 г.
 *      Author: yan
 */

#ifndef SINGLENODECONSTANTFORCE_H_
#define SINGLENODECONSTANTFORCE_H_

#include "IConstraint.h"


class SingleNodeConstantForce: public ISwitchableConstraint
{
public:
	SingleNodeConstantForce(size_t node, glm::dvec3 force)
		: node(node)
		, force(force) {}

	void solve(double time) override;

private:
    size_t node;
    glm::dvec3 force;
};

#endif /* SINGLENODECONSTANTFORCE_H_ */
