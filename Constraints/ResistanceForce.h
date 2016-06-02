/*
 * ResistanceForce.h
 *
 *  Created on: 14 апр. 2015 г.
 *      Author: yan
 */

#ifndef RESISTANCEFORCE_H_
#define RESISTANCEFORCE_H_

#include "IConstraint.h"


class ResistanceForce: public IConstraint
{
public:
    ResistanceForce(double viscosity)
        : viscosity(viscosity) {}
	void solve(double time) override;
private:
    double viscosity;
};


#endif /* RESISTANCEFORCE_H_ */
