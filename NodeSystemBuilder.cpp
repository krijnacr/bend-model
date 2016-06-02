/*
 * NodeSystemBuilder.cpp
 *
 *  Created on: 25 авг. 2014 г.
 *      Author: yan
 */

#include <set>
#include <list>
#include <vector>
#include <memory>
#include "Constraints/SpringForce.h"
#include "Constraints/TorsionSpringForce.h"
#include "Constraints/WormLikeChainForce.h"
#include "Constraints/StrainForce.h"
#include "NodeSystemBuilder.h"


class Node: public INode
{
public:
	Node(size_t id, double mass, glm::dvec3 pos, glm::dvec3 vel)
		: id(id)
		, fixed(false)
		, mass(mass)
		, position(pos)
		, velocity(vel) {}

	size_t getId() const override {
		return id;
	}

	double getMass() const override {
		return mass;
	}

	glm::dvec3 getPosition() const override {
		return position;
	}

	glm::dvec3 getVelocity() const override {
		if (fixed)
			return {0.0, 0.0, 0.0};
		return velocity;
	}

	glm::dvec3 getAcceleration() const override {
		if (fixed)
			return {0.0, 0.0, 0.0};
		return acceleration;
	}

	void fix(bool fixed) override {
		this->fixed = fixed;
	}

	bool isFixed() const override {
		return fixed;
	}

	void applyForce(const glm::dvec3& force) override {
		totalForce += force;
	}

	auto getForce() const -> glm::dvec3 {
		return totalForce;
	}

	void FixDirection(const glm::dvec3& force) override {
		// ограничение тех степени свободы
		// узла для которых проекции тянущей
		// силы равны нулю

		if(force.x == 0)
			totalForce.x = 0;
		if(force.y == 0)
			totalForce.y = 0;
		if(force.z == 0)
			totalForce.z = 0;
	}

	void resetForce() {
		totalForce = glm::dvec3(0.0);
	}

	void updateWithNewState(const glm::dvec3& _position, const glm::dvec3& _velocity) {
		if (fixed)
			return;
		position = _position;
		velocity = _velocity;
		acceleration = totalForce / mass;
	}

	void addAttribute(char attr) override {
		if (attributes.find(attr) == std::string::npos)
			attributes += attr;
	}

	void removeAttribute(char attr) override {
		auto pos = attributes.find(attr);
		if (pos != std::string::npos)
			attributes.erase(pos, 1);
	}

	bool hasAttribute(char attr) override {
		return attributes.find(attr) != std::string::npos;
	}

private:
	size_t id;
	bool fixed;
	double mass;
	glm::dvec3 totalForce;
	glm::dvec3 position, velocity, acceleration;
	std::string attributes;
};


class NodeSystemImpl: public INodeSystem
{
public:
	virtual size_t getNumberOfNodes() const override {
    	return nodes.size();
    }

    virtual std::shared_ptr<INode> getNode(size_t id) const override {
    	return nodes.at(id);
    }

    virtual size_t getNumberOfLinks() const override {
    	return links.size();
    }

    virtual std::pair<size_t,size_t> getLink(size_t id) const override {
    	return links.at(id);
    }

    virtual size_t getNumberOfOriginLinks() const override {
    	return originLinks.size();
    }

    virtual std::pair<size_t,size_t> getOriginLink(size_t id) const override {
    	return originLinks[id];
    }

    virtual size_t getNumberOfConstraints() const override {
    	return userConstraints.size();
    }

    virtual std::shared_ptr<IConstraint> getConstraint(size_t id) const override {
    	return userConstraints[id];
    }

    virtual std::shared_ptr<StrainForce> getGrip() const override {
    	return grip;
    }

    virtual size_t countNodes(char attr) const override {
    	size_t counter = 0;
    	for (auto& node: nodes) {
    		if (node->hasAttribute(attr))
    			counter++;
    	}
    	return counter;
	}

    // calculate derivative vector and store it in outputDerivs
    // getDerivative overriding
    virtual int getDerivative(double t, gsl::base_const_vector& inputCoords, gsl::base_vector& outputDerivs) override {

        // nullifying node forces
        for (auto& node: nodes) {
        	node->resetForce();
        }
        // applying forces
        for (auto& constraint: constraints) {
        	if (auto switchable = std::dynamic_pointer_cast<ISwitchableConstraint>(constraint))
        		switchable->getSwitcher()->switchConstraint(t);
        	constraint->solve(t);
        }

        glm::dvec3 position;
	    glm::dvec3 velocity;

	    size_t stride = INodeSystem::CoordsNum;
	    size_t size = nodes.size();

	    // если включен учет эффекта уменьшения объема
        if(enableVolumeReduce && (t > 0))
		{
        	// средняя точка сети
			auto mp = grip->getMidPoint();
			// тяговая сила
			auto F = grip->getForce();

			for (auto& node: nodes) {
		    // радиус-вектор узла относительно средней точки
			auto nr = node->getPosition() - mp;
			// векторное произведение силы и радиус-вектора
			auto p = glm::cross(F,nr);
			// если вектороное произведение не выраждено
			if(glm::length(p) > 0)
			{
				// вычисляем орт двойного векторного произведения:
				// векторного произведения сил и первого векторного произведения
				auto nFi = glm::normalize(glm::cross(F,p));
                // обновление определений состояния
				// если включен флаг обновления
				UpdateDef();

				// вычисление поправки для учета эффекта
				// уменьшения объема
				if(10*curExtension - curCompression > 0)	{
				nFi *= 10*curExtension - curCompression;
				// применение поправки к узлы
				node->applyForce(nFi);
				}
				//else std::cout << "nFi balance!" << "\n";
			}
			else {
				//std::cout << "cross is undef" << "\n";
				continue;
			}
			}
		}

        grip->solve(t);

        auto x = gsl::vector_const_view(inputCoords, X, stride, size);
        auto y = gsl::vector_const_view(inputCoords, Y, stride, size);
        auto z = gsl::vector_const_view(inputCoords, Z, stride, size);
        auto u = gsl::vector_const_view(inputCoords, U, stride, size);
        auto v = gsl::vector_const_view(inputCoords, V, stride, size);
        auto w = gsl::vector_const_view(inputCoords, W, stride, size);

        auto dx = gsl::vector_view(outputDerivs, X, stride, size);
        auto dy = gsl::vector_view(outputDerivs, Y, stride, size);
        auto dz = gsl::vector_view(outputDerivs, Z, stride, size);
        auto du = gsl::vector_view(outputDerivs, U, stride, size);
        auto dv = gsl::vector_view(outputDerivs, V, stride, size);
        auto dw = gsl::vector_view(outputDerivs, W, stride, size);

        glm::dvec3 derivs;

        for (size_t n = 0; n < nodes.size(); ++n) {

            nodes[n]->updateWithNewState(glm::dvec3(x[n], y[n], z[n]), glm::dvec3(u[n], v[n], w[n]));

        	auto vel = nodes[n]->getVelocity();
        	auto acc = nodes[n]->getAcceleration();

            dx[n] = vel.x;
            dy[n] = vel.y;
            dz[n] = vel.z;

            du[n] = acc.x;
            dv[n] = acc.y;
            dw[n] = acc.z;
        }
        return GSL_SUCCESS;
    }

    // вывод текущего состояния системы
    virtual void LogState(void) override
    {
    	std::cout << "Log sys state:" << "\n";
    	// по всем связям вывод состояния связи
    	//for (auto& constraint: constraints)
    	    //constraint->LogState();
    	// вывод определений состояния
    	LogDefs();
    }

    // сбросить флаг обновления
    // определений состояния
    virtual void Invalidate(void) override  {
            DefUpdated = false;
    }

    // установка учета эффекта уменьшения объема
    virtual void SetVolumeReduce(void)
    {
    	enableVolumeReduce = true;
    }

    // зафиксировать направление движения
    // тяговых узлов
    virtual void FixPullingDirection(void)
	{
		grip->FixAxe();
	}

    // returns the dimension of the system
    virtual size_t getDimension() const override {
    	return nodes.size() * CoordsNum;
    }

    void UpdateDef()
    {
    	// если требуется обновление определений
        if(!DefUpdated)
        {
        	/*  вычисление текущего
        	 *  фактора объема
        	 */
			auto mp = grip->getMidPoint();
			//std::cout << "m_id: " << m_id << "; " << mr.x << " " << mr.y << " " << mr.z << "\n";
			auto F = grip->getForce(); // тяговая сила
			if(glm::length(F) == 0)
			return;
			double l = 0;
			double cMin = DBL_MAX;
			double cMax = -DBL_MAX;
			// начальный фактор объема
			static double InitVolumeFactor;
			for (auto& node: nodes) {
			// радиус-вектор узла относительно
			// самого центрального узла
			auto nr = node->getPosition() - mp;
			double beta;
			// если узел центральный
			// переход к следующему узлу
			if(glm::length(nr) == 0)
				continue;
			// угол между направлением тяги
			// и вектором-указателем на узел
			beta = glm::acos(glm::dot(F,nr)/(glm::length(F)*glm::length(nr)));
            l += glm::length(nr)*glm::sin(beta);
            cMin = glm::min(cMin,glm::length(nr)*glm::cos(beta));
            cMax = glm::max(cMax,glm::length(nr)*glm::cos(beta));
			}
			l /= nodes.size();
			DefUpdated = true;
			// фактор объема
			VolumeFactor = M_PI*l*l*(cMax-cMin);
			//std::cout << "l: " << l << "\n";

            double fsum = 0;

            // орт растягивающей силы
            if(glm::length(F) > 0)
            	FDir = glm::normalize(F);
            // цикл по всем связям:
            // суммирование проекций векторов связей(векторов от первого конца связи до второго)
            // на орт тянущей силы
			for (auto& link: links) {
			auto vlink = getNode(link.first)->getPosition() - getNode(link.second)->getPosition();
			fsum += glm::abs(glm::dot(vlink, FDir));
			}

			// начальный продольный размер
			static double initialSize;

			// текущий продольный размер
			double currentSize = fsum;

			// для начального состояния задаем
			// объем/продольный размер и сбрасываем флаг начального состояния
			if(InitState)
			{
                InitVolumeFactor = VolumeFactor;
                initialSize = currentSize;
                InitState = false;
			}
            // текущие значение хар. растяжения
			curExtension = (currentSize - initialSize) / initialSize;

			// текущий аналог сжатия
			curCompression = InitVolumeFactor/VolumeFactor;
		}
        else return;
    }

    // вывод определений состояния
    void LogDefs(void)
    {
    	std::cout << "VolumeFactor: " << VolumeFactor << "\n";
    	std::cout << "strain = " << curExtension << "			";
    	std::cout << "compression = " << curCompression << "\n";
    }

public:
	std::vector<std::shared_ptr<Node>> nodes;
	std::set<std::shared_ptr<IConstraint>> constraints;
	std::vector<std::shared_ptr<IConstraint>> userConstraints;
	std::vector<std::pair<size_t,size_t>> links;
	std::vector<std::pair<size_t,size_t>> originLinks;
	std::shared_ptr<StrainForce> grip;
	glm::dvec3 FDir;
	bool   enableVolumeReduce = false;
	bool   DefUpdated = false; // обновление определений не требуется
	bool   InitState = true; // флаг начального состония
	double VolumeFactor; // фактор объема
	double curExtension;    // текущая хар. растяжения
	double curCompression;  // текущая хар. поперечной деформации
};


struct BuildNode {

	std::set<size_t> next;
	std::set<size_t> prev;
	size_t id;

	BuildNode(size_t id): id(id) {}
};


class NodeSystemBuilder::Implementation
{
public:
	std::vector<std::shared_ptr<BuildNode>> buildNodes;
	std::set<std::pair<size_t,size_t>> links;
	std::shared_ptr<NodeSystemImpl> model;

	double defaultSubNodeMass = 1.0;
	double defaultSpringStiffness = 1.0;
	double defaultTorsionSpringStiffness = 1.0;
	double defaultTemperature = 300.0; // 300' kelvin ~ 27' celsius
	double defaultPersistanceLength = 1e-23;
	double defaultContourLengthMultipler = 1.0;

	double defaultUnitsPerExtraNode = 1.0;
	size_t defaultExtraNodesPerEdge = 1;

	bool wormLikeEnabled = false;
	bool useExtraNodes = true;
	bool useConstantNumberOfExtraNodes = true;

public:
	void linkBuildNodes(size_t n1, size_t n2) {

		buildNodes[n1]->next.insert(n2);
		buildNodes[n2]->prev.insert(n1);
		model->links.push_back(std::make_pair(n1,n2));
	}

	void putLinearSpring(size_t n1, size_t n2) {

		auto p1 = model->getNode(n1);
		auto p2 = model->getNode(n2);
		double length = glm::length(p1->getPosition() - p2->getPosition());
		auto spring = std::make_shared<SpringForce>(n1, n2, length, defaultSpringStiffness);
		model->constraints.insert(spring);
	}

	void putWormLikeSpring(size_t n1, size_t n2) {

		auto p1 = model->getNode(n1);
		auto p2 = model->getNode(n2);
		double length = glm::length(p1->getPosition() - p2->getPosition());
		auto spring = std::make_shared<WormLikeChainForce>(n1, n2, length, defaultTemperature, defaultPersistanceLength, defaultContourLengthMultipler);
		model->constraints.insert(spring);
	}

	void putSpring(size_t n1, size_t n2) {

		if (wormLikeEnabled)
			return putWormLikeSpring(n1, n2);
		return putLinearSpring(n1, n2);
	}

	void putTorsionSpring(size_t n1, size_t n2, size_t n3) {

		//if (n2 >= 15 && n2 <= 19)
		//	return;

		auto leftNode = model->getNode(n1);
		auto centerNode = model->getNode(n2);
		auto rightNode = model->getNode(n3);

		glm::dvec3 p1 = leftNode->getPosition() - centerNode->getPosition();
		glm::dvec3 p2 = rightNode->getPosition() - centerNode->getPosition();

		double cos_theta = glm::dot(p1,p2) / glm::length(p1) / glm::length(p2);
		double theta = ::acos(cos_theta); // всегда в пределах [0;pi]

		auto torsionSpring = std::make_shared<TorsionSpringForce>(n1, n2, n3, defaultTorsionSpringStiffness, theta);
		model->constraints.insert(torsionSpring);
	}
};


NodeSystemBuilder::NodeSystemBuilder() {
	me = new Implementation;
	me->model.reset(new NodeSystemImpl);
	me->model->grip.reset(new StrainForce);
}

NodeSystemBuilder::~NodeSystemBuilder() {
	delete me;
}

void NodeSystemBuilder::linkNodes(size_t n1, size_t n2) {

	auto link = std::make_pair(n1,n2);
	size_t prevSize = me->links.size();
	me->links.insert(link);

	if (prevSize < me->links.size())
		me->model->originLinks.push_back(link);
}

size_t NodeSystemBuilder::addNode(double mass, glm::dvec3 pos, glm::dvec3 vel) {

	size_t newId = me->buildNodes.size();
	auto buildNode = std::make_shared<BuildNode>(newId);
	auto modelNode = std::make_shared<Node>(newId, mass, pos, vel);

	me->buildNodes.push_back(buildNode);
	me->model->nodes.push_back(modelNode);

	return newId;
}

size_t NodeSystemBuilder::addNode(double mass, glm::dvec3 pos) {
	return addNode(mass, pos, glm::dvec3(0.0));
}

std::list<glm::dvec3> fillSpace(glm::dvec3 from, glm::dvec3 to, size_t subNodes) {

	std::list<glm::dvec3> list;
	glm::dvec3 segment = (to - from) / (subNodes + 1.0);
	for (size_t i = 1; i <= subNodes; ++i)
		list.push_back(from + segment * (double) i);
	return list;
}

void NodeSystemBuilder::addConstraint(std::shared_ptr<IConstraint> constraint) {

	size_t prevCount = me->model->constraints.size();
	me->model->constraints.insert(constraint);

	if (prevCount < me->model->constraints.size()) {
		//const
		me->model->userConstraints.push_back(constraint);
	}
}

void NodeSystemBuilder::gripTheNode(size_t node) {
	me->model->grip->addNode(node);
}

void NodeSystemBuilder::setGripForce(const glm::dvec3& force) {
	me->model->grip->setForce(force);
}

void NodeSystemBuilder::FixPullingDirection(void)
{
	me->model->grip->FixAxe();
}

void NodeSystemBuilder::setDefaultSubNodeMass(double mass) {
	me->defaultSubNodeMass = mass;
}

void NodeSystemBuilder::setDefaultSpringStiffness(double stiffness) {
	me->defaultSpringStiffness = stiffness;
}

void NodeSystemBuilder::setDefaultTorsionSpringStiffness(double stiffness) {
	me->defaultTorsionSpringStiffness = stiffness;
}

void NodeSystemBuilder::setDefaultTemperature(double temperature) {
	me->defaultTemperature = temperature;
}

void NodeSystemBuilder::setDefaultPersistanceLength(double plength) {
	me->defaultPersistanceLength = plength;
}

void NodeSystemBuilder::setDefaultContourLengthMultiper(double multipler) {
	me->defaultContourLengthMultipler = multipler;
}

void NodeSystemBuilder::setDefaultUnitsPerExtraNode(double units) {
	me->defaultUnitsPerExtraNode = units;
}

void NodeSystemBuilder::setDefaultExtraNodesPerEdge(size_t numberOfNodes) {
	me->defaultExtraNodesPerEdge = numberOfNodes;
}

void NodeSystemBuilder::enableWormLikeChains(bool enabled) {
	me->wormLikeEnabled = enabled;
}

void NodeSystemBuilder::usingConstantNumberOfExtraNodes(bool useConstantNumber) {
	me->useConstantNumberOfExtraNodes = useConstantNumber;
}

void NodeSystemBuilder::useExtraNodes(bool use) {
	me->useExtraNodes = use;
}

std::shared_ptr<INodeSystem> NodeSystemBuilder::create() {

	for (auto& link: me->links) {

		if (!me->useExtraNodes) {
			me->linkBuildNodes(link.first, link.second);
			me->putSpring(link.first, link.second);
			continue;
		}

		// both linked nodes
		auto nodeFrom = me->model->getNode(link.first);
		auto nodeTo = me->model->getNode(link.second);

		size_t subNodeCount = 1;

		if (me->useConstantNumberOfExtraNodes) {
			subNodeCount = me->defaultExtraNodesPerEdge;
		}
		else {
			double length = glm::length(nodeTo->getPosition() - nodeFrom->getPosition());
			subNodeCount = (size_t) glm::round(length / me->defaultUnitsPerExtraNode);
		}

		if(subNodeCount == 0)
			std::cout << "subnodes cannot be placed" << "\n";

		// fill space between two nodes by adding a number of extra nodes
		auto points = fillSpace(nodeFrom->getPosition(), nodeTo->getPosition(), subNodeCount);
		std::vector<size_t> subNodeIds;

		for (glm::dvec3& point: points) {
			std::cout << "sn:" << point.x << " " << point.y << "\n";
			subNodeIds.push_back(addNode(me->defaultSubNodeMass, point));
		}

		if(subNodeCount == 0)
		{
			std::cout << "subnodesCount = 0 after fillSpace" << "\n";
			continue;
		}

		// link all nodes to make a chain
		for (size_t i = 0; i < subNodeIds.size() - 1; ++i) {
			me->linkBuildNodes(subNodeIds[i], subNodeIds[i + 1]);
			me->putSpring(subNodeIds[i], subNodeIds[i + 1]);
		}
		// link main nodes with head and tail of list of sub-nodes
		me->linkBuildNodes(link.first, subNodeIds.front());
		me->linkBuildNodes(subNodeIds.back(), link.second);

		std::cout << "connect subnodes to link edges" << "\n";
		me->putSpring(link.first, subNodeIds.front());
		me->putSpring(subNodeIds.back(), link.second);
	}
	std::cout << "Torsion springs mounting" << "\n";
	for (auto& center: me->buildNodes) {
		for (auto& left: center->prev) {
			for (auto& right: center->next) {
				me->putTorsionSpring(left, center->id, right);
			}
		}
	}

	// mount contstraints
	for (auto& constraint: me->model->constraints)
		constraint->assignToSystem(me->model);

	me->model->grip->assignToSystem(me->model);

	return me->model;
}
