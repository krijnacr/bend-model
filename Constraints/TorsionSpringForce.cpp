
#include <glm/glm.hpp>
#include "TorsionSpringForce.h"
#include <math.h>


void TorsionSpringForce::solve(double time) {

	if (auto sys = system.lock()) {

		auto leftNode = sys->getNode(left);
		auto centerNode = sys->getNode(center);
		auto rightNode = sys->getNode(right);

		glm::dvec3 p1 = leftNode->getPosition() - centerNode->getPosition();
		glm::dvec3 p2 = rightNode->getPosition() - centerNode->getPosition();

		double cos_theta = glm::dot(p1,p2) / glm::length(p1) / glm::length(p2);

		double theta;
		// провека на nan
		if(glm::abs(cos_theta) <= 1)
		theta = ::acos(cos_theta); // всегда в пределах [0;pi]
		else
		theta = M_PI;
		glm::dvec3 a;
		glm::dvec3 b;
		glm::dvec3 axb; //  первое векторное произведение p1 и p2

		glm::dvec3 c1;  // первое двойное векторное произведение
		glm::dvec3 c2;  // второе двойное векторное произведение


		double m = -stiffness * (defaultAngle - theta); // момент пружины
		axb = glm::cross(p1,p2); // первое векторное произведение
        c1 = glm::cross(axb,p1); // направление возвращающнй силы для узла p1
		c2 = glm::cross(p2,axb); // направление возвращающнй силы для узла p2

		// эквивалент действия пружины кручения на первый узел соотв. p1
	    a = m/(2*glm::length(p1)) * glm::normalize(c1);
	    // эквивалент действия пружины кручения на второй узел соотв. p2
		b = m/(2*glm::length(p2)) * glm::normalize(c2);

		if(!(isnan(a.x) || isnan(b.x))) {
		  leftNode->applyForce(a);
		  rightNode->applyForce(b);
		  centerNode->applyForce(-a - b);
		}
	}
}

// отладочный вывод текущего состояния пружины
// (уже не требуется)
void TorsionSpringForce::LogState(void)
{
	if (auto sys = system.lock()) {
	auto leftNode = sys->getNode(left);
	auto centerNode = sys->getNode(center);
	auto rightNode = sys->getNode(right);

	glm::dvec3 p1 = leftNode->getPosition() - centerNode->getPosition();
	glm::dvec3 p2 = rightNode->getPosition() - centerNode->getPosition();

	double cos_theta = glm::dot(p1,p2) / glm::length(p1) / glm::length(p2);
	double theta;
	if(glm::abs(cos_theta) <= 1)
	theta = ::acos(cos_theta); // всегда в пределах [0;pi]
	else
	theta = M_PI;
	// вывод
	std::cout << "theta=" << (theta*180/M_PI) << "\n";
	glm::dvec3 a;
	glm::dvec3 b;
	glm::dvec3 axb; //
	double m = -stiffness * (defaultAngle - theta);
	axb = glm::cross(p1,p2);
	a = m/(2*glm::length(p1)) * glm::cross(axb,p1)/glm::length(glm::cross(p1,axb));
	b = m/(2*glm::length(p2)) * glm::cross(p2,axb)/glm::length(glm::cross(p2,axb));
	}
}
