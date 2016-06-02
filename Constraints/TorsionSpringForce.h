/*
 * TorsionSpringForce.h
 *
 *  Created on: 14 апр. 2015 г.
 *      Author: yan
 */

#ifndef TORSIONSPRINGFORCE_H_
#define TORSIONSPRINGFORCE_H_

#include "IConstraint.h"

//
// Пружины кручения.
// В функции solve() происходит вычисление возвращающей силы
//

class TorsionSpringForce: public IConstraint
{
public:
	const double angleCompareEpsilon = 0.001 * M_PI;

	TorsionSpringForce(size_t left, size_t center, size_t right, double stiffness, double defaultAngle)
		: left(left)
		, center(center)
		, right(right)
		, stiffness(stiffness)
		, defaultAngle(defaultAngle) {  /*std::cout <<"TS added: "<< system.lock()->getNode(left)->getPosition().x << system.lock()->getNode(left)->getPosition().y << "\n";*/}

	void solve(double time) override;
    void LogState(void) override;
private:
	size_t left;         // индекс левого узла
	size_t center;       // индекс центрального узла
	size_t right;        // индекс правого узла
	double stiffness;    // жесткость пружины
	double defaultAngle; // угол равновесия
};

#endif /* TORSIONSPRINGFORCE_H_ */
