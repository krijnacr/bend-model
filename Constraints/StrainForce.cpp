/*
 * StrainForce.cpp
 *
 *  Created on: 05 мая 2015 г.
 *      Author: yan
 */

#include "StrainForce.h"


void StrainForce::setup() {

	size_t size = nodes.size();
	if (size < 2)
		return;

	matA = gsl_matrix_alloc(size-1, size-1);
	for (int i = 0; i < 3; ++i) {
		vecX[i] = gsl_vector_alloc(size-1);
		vecB[i] = gsl_vector_alloc(size-1);
	}
	permutation = gsl_permutation_alloc(size-1);
	gsl_matrix_set_all(matA, 1.0);

	if (auto sys = system.lock()) {

		auto avgpos = glm::dvec3(0.0);
		double minDistance = DBL_MAX;

		for (size_t i = 0; i < nodes.size(); ++i) {

			auto node = sys->getNode(nodes[i]);
			avgpos += node->getPosition();
		}
		avgpos /= size;

		for (size_t i = 0; i < size; ++i) {

			auto node = sys->getNode(nodes[i]);
			double distance = glm::length(node->getPosition() - avgpos);

			if (distance < minDistance) {
				minDistance = distance;
				mainId = nodes[i];
			}
		}
		size_t k = 0;
		otherIds = new size_t[size-1];

		for (size_t i = 0; i < size; ++i) {
			if (nodes[i] == mainId)
				continue;
			otherIds[k++] = nodes[i];
		}

		auto mainNode = sys->getNode(mainId);

		for (size_t i = 0; i < size-1; ++i) {
			auto node = sys->getNode(otherIds[i]);
			gsl_matrix_set(matA, i, i, gsl_matrix_get(matA, i, i) + mainNode->getMass() / node->getMass());
		}
		int signum;
		gsl_linalg_LU_decomp(matA, permutation, &signum);
		signum *= 1;
	}
}

StrainForce::~StrainForce()
{
	if (!initialized)
		return;

	gsl_matrix_free(matA);
	for (int i = 0; i < 3; ++i) {
		gsl_vector_free(vecX[i]);
		gsl_vector_free(vecB[i]);
	}
	gsl_permutation_free(permutation);
	delete [] otherIds;
}

void StrainForce::solve(double time) {

	if (nodes.size() < 1)
		return;

	if (nodes.size() < 2) {
		if (auto sys = system.lock()) {
			auto node = sys->getNode(nodes.front());
			node->applyForce(appliedForce);
			return;
		}
	}

	if (!initialized) {
		setup();
		initialized = true;
	}

	if (auto sys = system.lock()) {

		size_t size = nodes.size()-1;
		auto mainNode = sys->getNode(mainId);
		mainNode->applyForce(appliedForce);

		for (size_t i = 0; i < size; ++i) {

			auto node = sys->getNode(otherIds[i]);
			auto right = mainNode->getForce() - mainNode->getMass() / node->getMass() * node->getForce();

			gsl_vector_set(vecB[0], i, right.x);
			gsl_vector_set(vecB[1], i, right.y);
			gsl_vector_set(vecB[2], i, right.z);
		}
		for (int i = 0; i < 3; ++i)
			gsl_linalg_LU_solve(matA, permutation, vecB[i], vecX[i]);

		for (size_t i = 0; i < size; ++i) {

			auto node = sys->getNode(otherIds[i]);
			glm::dvec3 f = {
				gsl_vector_get(vecX[0], i),
				gsl_vector_get(vecX[1], i),
				gsl_vector_get(vecX[2], i),
			};
			node->applyForce(f);
			mainNode->applyForce(-f);
		}

		if(fixedAxe)
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			auto node = sys->getNode(nodes[i]);
			node->FixDirection(appliedForce);
		}
	}
}
