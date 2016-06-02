/*
 * IConstraint.h
 *
 *  Created on: 24 авг. 2014 г.
 *      Author: yan
 */

#ifndef ICONSTRAINT_H_
#define ICONSTRAINT_H_

#include <memory>
#include "../INodeSystem.h"


class IConstraint
{
public:
	virtual ~IConstraint() {}
	virtual void solve(double time) = 0;
	virtual void LogState(void) {} // вывод текущего состояния связи

	void assignToSystem(std::weak_ptr<INodeSystem> a_system) {
		system = a_system;
	}
protected:
	std::weak_ptr<INodeSystem> system;
};



class IConstraintSwitcher;

class ISwitchableConstraint: public IConstraint
{
public:
	void enable(bool a_enabled) {
		enabled = a_enabled;
	}

	bool isEnabled() const {
		return enabled;
	}

	void setSwitcher(IConstraintSwitcher* a_switcher) {
		switcher.reset(a_switcher);
	}

	void setSwitcher(std::shared_ptr<IConstraintSwitcher> a_switcher) {
		switcher = a_switcher;
	}

	std::shared_ptr<IConstraintSwitcher> getSwitcher() {
		return switcher;
	}

protected:
	bool enabled = false;
	std::shared_ptr<IConstraintSwitcher> switcher;
};


class IConstraintSwitcher
{
public:
	virtual ~IConstraintSwitcher() {}
	virtual void switchConstraint(double time) = 0;

	void assignToConstraint(std::weak_ptr<ISwitchableConstraint> a_constraint) {
		constraint = a_constraint;
	}

protected:
	std::weak_ptr<ISwitchableConstraint> constraint;
};

#endif /* ICONSTRAINT_H_ */
