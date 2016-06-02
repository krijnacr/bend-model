/*
 * WormLikeChainForce.cpp
 *
 *  Created on: 21 апр. 2015 г.
 *      Author: yan
 */

#include "WormLikeChainForce.h"


void WormLikeChainForce::solve(double time) {

	if (auto sys = system.lock()) {

		auto firstNode = sys->getNode(first);
		auto secondNode = sys->getNode(second);

	    glm::dvec3 p = firstNode->getPosition() - secondNode->getPosition();

		const double kB = 1.3806488e-23;
		double length = glm::length(p);
		double dL_normalized = (length - lengthZero) / (lengthZero * contourLengthMultipler);
		double f = kB * temperature / plength;

		f = f * (0.25 * pow(1.0 - dL_normalized, -2.0) - 0.25 + dL_normalized);

	    firstNode->applyForce(-f * p/length);
	    secondNode->applyForce(f * p/length);
	}
}
