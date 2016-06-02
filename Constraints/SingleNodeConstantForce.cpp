
#include <glm/glm.hpp>
#include "SingleNodeConstantForce.h"


void SingleNodeConstantForce::solve(double time) {

	if (!enabled)
		return;

	if (auto sys = system.lock()) {
		auto ptr = sys->getNode(node);
		ptr->applyForce(force);
	}
}
