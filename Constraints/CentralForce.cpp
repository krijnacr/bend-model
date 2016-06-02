/*
 * CentralForce.cpp
 *
 *  Created on: 5 мая 2016 г.
 *      Author: yan
 */

#include "CentralForce.h"


void CentralForce::solve(double time) {

	if (auto sys = system.lock()) {
		//
		// проходим по всем узлам и проверяем на попадание в сферы
		//
		for (size_t i = 0; i < sys->getNumberOfNodes(); ++i) {

			// берем узел по индексу i
			auto node = sys->getNode(i);

			// вычисляем расстояние от узла до центра
			double dist = glm::distance(node->getPosition(), center);

			// узел за пределами большой сферы, переходим к следующему
			if (dist > bigRadius)
				continue;

			// узел в пределах маленькой сферы, на него не действует сила
			//if (dist <= smallRadius)
			//	continue;

			// вектор из узла в центр
			glm::dvec3 d = center - node->getPosition();
			glm::dvec3 f = multipler * d / (dist * dist * dist);
			node->applyForce(f);
		}
	}
}
