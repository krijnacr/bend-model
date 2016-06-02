
#include <glm/glm.hpp>
#include "NodeToNodeForce.h"


void NodeToNodeForce::solve(double time) {

	if (!enabled)
		return;

	if (auto sys = system.lock()) {

		auto fromNode = sys->getNode(from);
		auto toNode = sys->getNode(to);
	    glm::dvec3 f = toNode->getPosition() - fromNode->getPosition();
	    fromNode->applyForce(glm::normalize(f) * value);
	}
}
