
#include <glm/glm.hpp>
#include "SpringForce.h"


void SpringForce::solve(double time) {

	if (auto sys = system.lock()) {

		auto firstNode = sys->getNode(first);
		auto secondNode = sys->getNode(second);

	    glm::dvec3 p = firstNode->getPosition() - secondNode->getPosition();
	    glm::dvec3 f = stiffness * (glm::length(p) - length) * p/glm::length(p);

	    firstNode->applyForce(-f);
	    secondNode->applyForce(f);
	}
}
