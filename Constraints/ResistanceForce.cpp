
#include <glm/glm.hpp>
#include "ResistanceForce.h"


void ResistanceForce::solve(double time) {
	if (auto sys = system.lock()) {

		for (size_t i = 0; i < sys->getNumberOfNodes(); ++i ) {
			auto node = sys->getNode(i);
			node->applyForce(-viscosity * node->getVelocity());
		}
	}
}
