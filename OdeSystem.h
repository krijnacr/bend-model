/*
 * OdeSystem.h
 *
 *  Created on: 28 июня 2014 г.
 *      Author: yan
 */

#ifndef ODESYSTEM_H_
#define ODESYSTEM_H_

#include "VectorMath.h"


class IOdeSystem
{
public:
    virtual ~IOdeSystem() {}

    // calculate derivative vector and store it in outputDerivs
    virtual int getDerivative(double t, gsl::base_const_vector& inputCoords, gsl::base_vector& outputDerivs) = 0;

    // returns the dimension of the system
    virtual size_t getDimension() const = 0;
};

#endif /* ODESYSTEM_H_ */
