/*
 * StrainForce.h
 *
 *  Created on: 05 мая 2015 г.
 *      Author: yan
 */

#ifndef STRAINFORCE_H_
#define STRAINFORCE_H_

extern "C" {
#include <gsl/gsl_linalg.h>
}

#include <vector>
#include "IConstraint.h"



class StrainForce: public IConstraint
{
private:
	void setup();

public:
	~StrainForce();
	void solve(double time);

	void addNode(size_t node) {
		nodes.push_back(node);
	}

	void FixAxe(void)
	{
		fixedAxe = true;
	}

	size_t getNumberOfNodes() const {
		return nodes.size();
	}

	size_t getNodeId(size_t n) const {
		return nodes.at(n);
	}

	void setForce(const glm::dvec3& a_force) {
		appliedForce = a_force;
	}

	glm::dvec3 getForce() const {
		return appliedForce;
	}

	glm::dvec3 getMidPoint() const {
		auto avgpos = glm::dvec3(0.0);
		if (auto sys = system.lock())
		for (size_t i = 0; i < nodes.size(); ++i) {
			avgpos += sys->getNode(nodes[i])->getPosition();
		}
		avgpos /= nodes.size();;
		return avgpos;
		}

	size_t getMainId() const {
		if(initialized)
			return mainId;
		else
			return -1;
	}

private:
	bool initialized = false;
	bool fixedAxe = false;

	gsl_matrix* matA;
	gsl_vector* vecX[3];
	gsl_vector* vecB[3];
	gsl_permutation* permutation;

	size_t mainId;
	size_t* otherIds;

	std::vector<size_t> nodes;
	glm::dvec3 appliedForce;
};

#endif /* STRAINFORCE_H_ */
