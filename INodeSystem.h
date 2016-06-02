/*
 * INodeSystem.h
 *
 *  Created on: 24 авг. 2014 г.
 *      Author: yan
 */

#ifndef INODESYSTEM_H_
#define INODESYSTEM_H_

#include <glm/glm.hpp>
#include "OdeSystem.h"
#include "INode.h"


class IConstraint;
class StrainForce;


class INodeSystem: public IOdeSystem
{
public:
	enum CoordsLayout {
		X, Y, Z,
		U, V, W,
		CoordsNum,
	};

	virtual ~INodeSystem() {}

    virtual size_t getNumberOfNodes() const = 0;
    virtual std::shared_ptr<INode> getNode(size_t id) const = 0;

    virtual size_t getNumberOfLinks() const = 0;
    virtual std::pair<size_t,size_t> getLink(size_t id) const = 0;

    virtual size_t getNumberOfOriginLinks() const = 0;
    virtual std::pair<size_t,size_t> getOriginLink(size_t id) const = 0;

    virtual size_t getNumberOfConstraints() const = 0;
    virtual std::shared_ptr<IConstraint> getConstraint(size_t id) const = 0;
    virtual std::shared_ptr<StrainForce> getGrip() const = 0;

	virtual size_t countNodes(char attr) const = 0;
	virtual void LogState(void) { }; // вывод текущего состояния системы
	virtual void Invalidate(void) { } // обновить дополнительные параметры
    virtual void SetVolumeReduce(void)  {  } // установить эффект уменьшения объема
    virtual void FixPullingDirection(void) {  } // зафиксировать направление оси тяги
};

#endif /* INODESYSTEM_H_ */
