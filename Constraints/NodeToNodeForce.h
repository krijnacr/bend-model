/*
 * NodeToNodeForce.h
 *
 *  Created on: 14 апр. 2015 г.
 *      Author: yan
 */

#ifndef NODETONODEFORCE_H_
#define NODETONODEFORCE_H_

#include "IConstraint.h"


class NodeToNodeForce: public ISwitchableConstraint
{
public:
	NodeToNodeForce(size_t from, size_t to, double value)
		: from(from)
		, to(to)
		, value(value) {}

	void solve(double time) override;

private:
	size_t from;
	size_t to;
    double value;
};

#endif /* NODETONODEFORCE_H_ */
