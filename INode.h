/*
 * INode.h
 *
 *  Created on: 26 авг. 2014 г.
 *      Author: yan
 */

#ifndef INODE_H_
#define INODE_H_

#include <glm/glm.hpp>


class INode
{
public:
	virtual ~INode() {}

	virtual size_t getId() const = 0;
	virtual double getMass() const = 0;
	virtual glm::dvec3 getPosition() const = 0;
	virtual glm::dvec3 getVelocity() const = 0;
	virtual glm::dvec3 getAcceleration() const = 0;

	virtual void addAttribute(char attr) = 0;
	virtual void removeAttribute(char attr) = 0;
	virtual bool hasAttribute(char attr) = 0;

	virtual void applyForce(const glm::dvec3& force) = 0;
	virtual void FixDirection(const glm::dvec3& force) = 0;
	virtual auto getForce() const -> glm::dvec3 = 0;
	virtual void fix(bool fixed) = 0;
	virtual bool isFixed() const = 0;
};

#endif /* INODE_H_ */
