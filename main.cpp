
extern "C" {
#include <gsl/gsl_odeiv.h>
#include <gsl/gsl_linalg.h>
}
#include <iomanip>
#include <string>
#include <memory>
#include <fstream>
#include <time.h>
#include <pugixml.hpp>
#include "NodeSystemBuilder.h"
#include "Constraints/ResistanceForce.h"
#include "Constraints/StrainForce.h"
#include "Constraints/CentralForce.h"


enum UsageError {
	UE_NOT_ENOUGH_ARGUMENTS = 1,
	UE_UNKNOWN_ARGUMENT,
	UE_INCOMPLETE_SYSTEM,
};

glm::dvec3 fDir;
bool InitExt = true;
bool LogState = false;
double Extension;
double initialSize;
double lastExtension;
double currentExtension;
double currentSize;
double lastSize;


void printUsage() {
	std::cout << "Usage1: <program-name> [params] <scheme-file>" << std::endl;
}


// проверка условия равновесия
bool CheckBalance(std::shared_ptr<INodeSystem> ns,double veps)
{
	double coordMin = DBL_MAX;
	double coordMax = -DBL_MAX;

	for (size_t id = 0; id < ns->getNumberOfNodes(); ++id) {
		auto pos = ns->getNode(id)->getPosition();
		coordMin = glm::min(coordMin, glm::dot(pos, fDir));
		coordMax = glm::max(coordMax, glm::dot(pos, fDir));
	}

	double currentSize = coordMax - coordMin;

	if (InitExt) {
		initialSize = currentSize;
		lastSize = currentSize;
		InitExt = false;
		return true;
	}

	lastExtension = lastSize / initialSize;
	currentExtension = currentSize / initialSize;

	lastSize = currentSize;
	if (glm::abs(currentExtension - lastExtension) > veps)
	return true;
}


class SimpleSwitcher: public IConstraintSwitcher
{
public:
	SimpleSwitcher(double a_startTime, double a_endTime)
		: startTime(a_startTime), endTime(a_endTime) {}

	void switchConstraint(double time) override
	{
		if (auto target = constraint.lock()) {
			if (time < startTime || (endTime > 0.0 && time > endTime))
				target->enable(false);
			else
				target->enable(true);
		}
	}
private:
	double startTime;
	double endTime;
};


std::shared_ptr<INodeSystem> createNodeSystem(const char* schemeFile)
{
	pugi::xml_document doc;
	pugi::xml_parse_result parseResult = doc.load_file(schemeFile);

	if (!parseResult) {
		std::cout << "parse error in createNodeSystem: " << parseResult.description() << std::endl;
		return nullptr;
	}
	pugi::xml_node root = doc.child("data");
	pugi::xml_node nodes = root.child("nodes");
	pugi::xml_node links = root.child("links");
	pugi::xml_node props = root.child("settings");

	if (!(root && nodes && links)) {
		std::cout << "couldn't find nessesary data\n";
		return false;
	}

	NodeSystemBuilder builder;
	std::vector<size_t> originNodes;

	double mass;
	double x, y, z;
	double defaultMass = nodes.attribute("default-mass").as_double(-1.0);

	for (auto node = nodes.child("node"); node; node = node.next_sibling("node")) {
		mass = node.attribute("mass").as_double(defaultMass);
		if (mass < 0.0) {
			std::cout << "parse error: node mass is undefined\n";
			return false;
		}
		const char* text = node.text().as_string();

		std::cout << text << std::endl;

		if (3 != sscanf(text, "%lf %lf %lf", &x, &y, &z)) {
			std::cout << "parse node error\n";
			return false;
		}
		size_t id = builder.addNode(mass, glm::dvec3(x, y, z));
		originNodes.push_back(id);
	}

	size_t from, to;

	for (auto link = links.child("link"); link; link = link.next_sibling("link")) {
		if (2 != sscanf(link.text().as_string(""), "%lu %lu", &from, &to)) {
			std::cout << "parse link error\n";
			return false;
		}
		builder.linkNodes(from, to);
	}

	double resistance = 1.0;

	if (auto p = props.child("resistance"))
		resistance = p.text().as_double(resistance);

	builder.addConstraint(std::make_shared<ResistanceForce>(resistance));

	if (auto p = props.child("spring-stiffness"))
		builder.setDefaultSpringStiffness(p.text().as_double());

	if (auto p = props.child("torsion-stiffness"))
		builder.setDefaultTorsionSpringStiffness(p.text().as_double());

	if (auto p = props.child("persistance-length"))
		builder.setDefaultPersistanceLength(p.text().as_double());

	if (auto p = props.child("absolute-temperature"))
		builder.setDefaultTemperature(p.text().as_double());

	if (auto p = props.child("contour-length-multipler"))
		builder.setDefaultContourLengthMultiper(p.text().as_double());

	if (auto p = props.child("units-per-extra-node"))
		builder.setDefaultUnitsPerExtraNode(p.text().as_double());

	if (auto p = props.child("extra-nodes-per-edge"))
		builder.setDefaultExtraNodesPerEdge(p.text().as_uint());

	if (auto p = props.child("use-extra-nodes"))
		builder.useExtraNodes(p.text().as_bool());

	if (auto p = props.child("constant-extra-nodes"))
		builder.usingConstantNumberOfExtraNodes(p.text().as_bool());

	if (auto p = props.child("worm-like"))
		builder.enableWormLikeChains(p.text().as_bool());


	pugi::xml_node forceRoot = root.child("force");
	if (forceRoot) {

		//double startTime = force.attribute("start-time").as_double(0.0);
		//double endTime = force.attribute("end-time").as_double(0.0);

		if (3 == sscanf(forceRoot.attribute("value").as_string(), "%lf %lf %lf", &x, &y, &z)) {

			builder.setGripForce(glm::dvec3(x,y,z));
			for (auto node = forceRoot.child("node"); node; node = node.next_sibling("node"))
				builder.gripTheNode(node.text().as_uint());

			/*auto appliedForce = std::make_shared<MultiNodeConstantForce>(nodeList, glm::dvec3(x,y,z));
			auto switcher = std::make_shared<SimpleSwitcher>(startTime, endTime);

			appliedForce->setSwitcher(switcher);
			switcher->assignToConstraint(appliedForce);*/
		}
	}

	pugi::xml_node forces = root.child("forces");
	if (forces) {
		for (auto node: forces.children()) {

			if (std::string("grip") == node.name()) {
				//
				// сила тяги/захвата
				//
				if (3 == sscanf(node.attribute("value").as_string(), "%lf %lf %lf", &x, &y, &z)) {
					auto grip = std::make_shared<StrainForce>();

					grip->setForce(glm::dvec3(x,y,z));
					for (auto n: node.children("node"))
						grip->addNode(n.text().as_uint());

					builder.addConstraint(grip);
				}
			}
			else if (std::string("central") == node.name()) {
				//
				// центральная сила.
				//

				// считываем атрибут "center" и пытаемся сканировать три числа
				// Функция scanf возвращает кол-во удачно считанных аргументов

				if (3 == sscanf(node.attribute("center").as_string(), "%lf %lf %lf", &x, &y, &z)) {

					// считываем два радиуса и жёсткость
					// в скобках записываем параметры по-умолчанию
					double smallRadius = node.attribute("small-radius").as_double(0.0);
					double bigRadius = node.attribute("big-radius").as_double(1.0);
					double multipler = node.attribute("multipler").as_double(1.0);

					// создаём объект CentralForce и добавляем его в общий список сил/ограничений
					auto force = std::make_shared<CentralForce>(glm::dvec3(x,y,z), smallRadius, bigRadius, multipler);
					builder.addConstraint(force);
				}
			}
		}
	}
	auto model = builder.create();

	std::cout << "model builded" << "\n";

	for (size_t i = 0; i < originNodes.size(); ++i)
		model->getNode(i)->addAttribute('A');

	pugi::xml_node fixedRoot = root.child("fixed");
	if (fixedRoot) {
		for (auto node = fixedRoot.child("node"); node; node = node.next_sibling("node"))
			model->getNode(node.text().as_uint())->fix(true);
	}
	return model;
}


std::string generateOutputFileName(std::string inputFileName)
{
	time_t now;
	const int MAX_DATE = 64;
	char theDate[MAX_DATE];
	theDate[0] = '\0';

	now = time(nullptr);

	if (now != -1) {
		strftime(theDate, MAX_DATE, "_%Y.%m.%d_%H-%M-%S", gmtime(&now));
		return inputFileName + theDate;
	}
	return "";
}


int derivativeFunction(double t, const double y[], double dydt[], void* params)
{
    IOdeSystem* sys = static_cast<IOdeSystem*>(params);

    auto inputCoords = gsl::vector_const_view( gsl_vector_const_view_array(y, sys->getDimension()) );
    auto outputDerivs = gsl::vector_view( gsl_vector_view_array(dydt, sys->getDimension()) );

    return sys->getDerivative(t, inputCoords, outputDerivs);
}


class ISolutionStorage
{
public:
	virtual ~ISolutionStorage() {}
	virtual bool storePoint(double time, const gsl::vector& coords) = 0;
	virtual void storeProgress(double percentage) = 0;
};


void solve(std::shared_ptr<INodeSystem> theSystem, double startTime, double endTime, double delta, ISolutionStorage& storage)
{
	size_t dim = theSystem->getDimension();
	auto odeivSystem = gsl_odeiv_system {&derivativeFunction, nullptr, dim, theSystem.get()};
	auto odeivStep = std::shared_ptr<gsl_odeiv_step>(gsl_odeiv_step_alloc(gsl_odeiv_step_rk4, dim), gsl_odeiv_step_free);

	gsl::vector y(theSystem->getDimension());
	gsl::vector yerr(theSystem->getDimension());
	double timeStamp = startTime;

	//
	// Initial conditions
	//
	for (size_t i = 0; i < theSystem->getNumberOfNodes(); ++i)
	{
		auto pos = theSystem->getNode(i)->getPosition();
		auto vel = theSystem->getNode(i)->getVelocity();
        std::cout << pos.x << " " << pos.y << "\n";
		y[INodeSystem::CoordsNum * i + INodeSystem::X] = pos.x;
		y[INodeSystem::CoordsNum * i + INodeSystem::Y] = pos.y;
		y[INodeSystem::CoordsNum * i + INodeSystem::Z] = pos.z;

		y[INodeSystem::CoordsNum * i + INodeSystem::U] = vel.x;
		y[INodeSystem::CoordsNum * i + INodeSystem::V] = vel.y;
		y[INodeSystem::CoordsNum * i + INodeSystem::W] = vel.z;
	}

	while (timeStamp < endTime)
	{
		//std::cout << timeStamp << " " << endTime << std::endl;
		int status = gsl_odeiv_step_apply(odeivStep.get(), timeStamp, delta, y.data(), yerr.data(), nullptr, nullptr, &odeivSystem);
		if (status != GSL_SUCCESS)
			return;
        theSystem->Invalidate();

		// storage.storeProgress((timeStamp - startTime) / (endTime - startTime));
		if (!storage.storePoint(timeStamp, y))
			return;

		timeStamp += delta;
	}
	storage.storeProgress(1.0);
}


class TextFileStorage: public ISolutionStorage
{
public:
	TextFileStorage(std::shared_ptr<INodeSystem> a_system, const std::string& a_fileName, bool logon)
		: system(a_system), output(a_fileName),elog(logon) {}

	void writeHeader(double startTime, double endTime, double delta)
	{
		output << "start_time " << startTime << '\n';
		output << "end_time " << endTime << '\n';
		output << "step " << delta << '\n';

		output << "node_count " << system->getNumberOfNodes() << '\n';
		output << "origin_node_count " << system->countNodes('A') << '\n';

		for (size_t i = 0; i < system->getNumberOfNodes(); ++i) {
			auto node = system->getNode(i);
			if (node->hasAttribute('A'))
				output << ' ' << node->getId();
		}

		output << "\nlink_count " << system->getNumberOfLinks();

		for (size_t i = 0; i < system->getNumberOfLinks(); ++i) {
			auto link = system->getLink(i);
			output << '\n' << link.first << ' ' << link.second;
		}
	}

	bool storePoint(double time, const gsl::vector& coords) override
	{
		glm::dvec3 pos, vel;
		output << "\n\nframe " << time;
        // вывод состояния, если
		// включено логирование
		if(elog)
		system->LogState();
		for (size_t i = 0; i < system->getNumberOfNodes(); ++i) {
			auto node = system->getNode(i);
			output << '\n' << node->getId();

			pos = node->getPosition();
			vel = node->getVelocity();
			for (int k = 0; k < 3; ++k)
				output << ' ' << pos[k];

			for (int k = 0; k < 3; ++k)
				output << ' ' << vel[k];
		}
		return true;
	}

	void storeProgress(double percentage) override
	{
	}

private:
	std::shared_ptr<INodeSystem> system;
	std::ofstream output;
	bool elog = false; // флаг логирования
};


void dumpNodecoords(std::shared_ptr<INodeSystem> system, const std::string& outputFileName, bool dumpExtra) {

	std::ofstream output(outputFileName);
	if (!output)
		return;

	size_t numberOfLinks = dumpExtra ? system->getNumberOfLinks() : system->getNumberOfOriginLinks();

	for (size_t i = 0; i < numberOfLinks; ++i) {
		auto link = dumpExtra ? system->getLink(i) : system->getOriginLink(i);
		output << link.first << ' ' << link.second << '\n';
	}

	for (size_t i = 0; i < system->getNumberOfNodes(); ++i) {

		auto node = system->getNode(i);
		if (!node->hasAttribute('A') && !dumpExtra)
			continue;

		auto pos = node->getPosition();

		if (i > 0) {
			output << '\n';
		}
		output << "//NODECOORD " << node->getId();
		for (int k = 0; k < 3; ++k)
			output << ' ' << glm::round(pos[k] * 10.0) / 10.0;
	}
}


int solve_main(int argc, char** argv) {

	if (argc < 1) {
		printUsage();
		std::cout << "solve_main" << std::endl;
		return UE_NOT_ENOUGH_ARGUMENTS;
	}

	std::cout << "solve_main" << std::endl;

	double startTime = 0.0;
	double endTime = 15.0;
	double delta = 0.015;

	bool dumpNodecoordsEncountered = false;
	bool dumpExtraNodecoordsEncountered = false;
	bool LogState = false; // логирование состояния сети

	for (int i = 0; i < argc-1; ++i) {

		std::string arg = argv[i];
		size_t pos = arg.find('=');

		if (pos == std::string::npos)
		{
			if (arg == "--nodecoords") {
				dumpNodecoordsEncountered = true;
				continue;
			}
			if (arg == "--dumpextra") {
				dumpExtraNodecoordsEncountered = true;
				continue;
			}
			if (arg == "--logstate") {
				//
				LogState = true;
				continue;
			}
			std::cout <<  "UNKNOWN ARG " << arg;
			printUsage();
			return UE_UNKNOWN_ARGUMENT;
		}
		std::string key = arg.substr(0, pos);
		std::string val = arg.substr(pos + 1);

		if (key == "-s") {
			startTime = std::atof(val.c_str());
			continue;
		}
		if (key == "-e") {
			endTime = std::atof(val.c_str());
			continue;
		}
		if (key == "-d") {
			delta = std::atof(val.c_str());
			continue;
		}
		std::cout <<  "UNKNOWN ARG " << arg;
		printUsage();
		return UE_UNKNOWN_ARGUMENT;
	}
	auto system = createNodeSystem(argv[argc-1]);
	auto outputFileName = generateOutputFileName(argv[argc-1]);
	TextFileStorage storage(system, outputFileName + ".sol",LogState);

	if (dumpNodecoordsEncountered)
		dumpNodecoords(system, outputFileName + "_start.layout", dumpExtraNodecoordsEncountered);

	storage.writeHeader(startTime, endTime, delta);
	solve(system, startTime, endTime, delta, storage);

	if (dumpNodecoordsEncountered)
		dumpNodecoords(system, outputFileName + "_finish.layout", dumpExtraNodecoordsEncountered);

	return 0;
}


class ForceDiagramStorage: public ISolutionStorage
{
public:
	ForceDiagramStorage(std::shared_ptr<INodeSystem> a_system, const std::string& a_fileName)
		: system(a_system),
		  statesOutput(a_fileName+".sta"),
		bn(a_fileName)	{

		statesOutput << "node_count " << system->getNumberOfNodes() << '\n';
		statesOutput << "origin_node_count " << system->countNodes('A') << '\n';

		for (size_t i = 0; i < system->getNumberOfNodes(); ++i) {
			auto node = system->getNode(i);
			if (node->hasAttribute('A'))
				statesOutput << ' ' << node->getId();
		}

		statesOutput << "\nlink_count " << system->getNumberOfLinks();

		for (size_t i = 0; i < system->getNumberOfLinks(); ++i) {
			auto link = system->getLink(i);
			statesOutput << '\n' << link.first << ' ' << link.second;
		}
		statesOutput.close();
	}


	bool parametrize(double a_vepsilon, double a_forceStep,bool logon) {

		vepsilon = a_vepsilon;
		forceStep = a_forceStep;
        elog = logon;
		grip = system->getGrip();

		forceMax = glm::length(grip->getForce());
		forceDir = glm::normalize(grip->getForce());
		forceValue = 0.0;

		grip->setForce(forceDir * forceValue);

		return true;
	}

	void setUpdateEachStep(size_t a_updateEachStep) {
		stepInterval = a_updateEachStep;
	}

	bool storePoint(double time, const gsl::vector& coords) override {

		if (!firstStep) {
			++stepCounter;
			if (stepCounter % stepInterval > 0)
				return true;
		}
		//std::cout << stepInterval << "\n";
		double coordMin = DBL_MAX;
		double coordMax = -DBL_MAX;
		double fsum = 0;

		for (size_t id = 0; id < system->getNumberOfLinks(); ++id) {
		auto link = system->getLink(id);
		auto vlink = system->getNode(link.first)->getPosition() - system->getNode(link.second)->getPosition();
		fsum += glm::abs(glm::dot(vlink, forceDir));
		}
		double currentSize = fsum;

		if (firstStep) {
			initialSize = currentSize;
			lastSize = currentSize;
			firstStep = false;
			return true;
		}

		// *** OLD OUTPUT ***

		double lastExtension = (lastSize - initialSize) / initialSize;
		double currentExtension = (currentSize - initialSize) / initialSize;
		lastSize = currentSize;

		std::cout
			<< "t="     << std::setw(9) << std::left << time
			<< "strain="   << std::setw(14) << std::left << currentExtension
			<< "delta=" << std::setw(12) << std::left << glm::abs(currentExtension - lastExtension) << std::endl;

		if (glm::abs(currentExtension - lastExtension) > vepsilon)
			return true;

		//
		// Equilibrium state found, store the result
		//

		statesOutput.open(bn+".sta",std::ofstream::out | std::ofstream::app);
		output.open(bn+".grm",std::ofstream::out | std::ofstream::app);
		statesOutput << "\n\nforce " << forceValue;
		statesOutput << "\nstrain " << currentExtension << "\n";
        // логгирование состояния
		if(elog)
		system->LogState();
		for (size_t i = 0; i < system->getNumberOfNodes(); ++i) {

			auto node = system->getNode(i);
			statesOutput << '\n' << node->getId();

			auto pos = node->getPosition();
			auto vel = node->getVelocity();

			for (int k = 0; k < 3; ++k)
				statesOutput << ' ' << pos[k];

			for (int k = 0; k < 3; ++k)
				statesOutput << ' ' << vel[k];
		}

		output << forceValue << ' ' << currentExtension << std::endl;
		statesOutput.flush();
		output.flush();
		statesOutput.close();
		output.close();

		std::cout << "*** one step completed ***\n\n";

		forceValue += forceStep;
		if (forceValue <= forceMax) {

			grip->setForce(forceDir * forceValue);
			return true;
		}
		return false;
	}

	void storeProgress(double percentage) override {

	}

private:
	std::shared_ptr<INodeSystem> system;
	std::shared_ptr<StrainForce> grip;
	std::ofstream output;
	std::ofstream statesOutput;
	std::string bn;
	glm::dvec3 forceDir;

	size_t stepCounter = 0;
	size_t stepInterval = 10;

	double lastAvgPos = 0.0;
	double lastMainNodesAvgPos = 0.0;

	double vepsilon = 0.0;
	double forceMax = 0.0;
	double forceStep = 0.0;
	double forceValue = 0.0;
	double initialSize = 0.0;
	double lastSize = 0.0;
	bool firstStep = true;
	bool elog = false;
};


int diagram_main(int argc, char** argv) {

	time_t t0,t1;
	t0 = time(0);
	std::cout << "inside diagram_main " << asctime(localtime(&t0)) << std::endl;
	if (argc < 3) {
		std::cout << "not enough parameters" << std::endl;
		printUsage();
		return UE_NOT_ENOUGH_ARGUMENTS;
	}

	std::cout << "diagram_main started" << std::endl;

	double forceStep = 0.0;
	double vepsilon = 0.0;
	double solveDelta = 0.01;
	size_t updateEachStep = 10;

	bool forceStepEncountered = false;
	bool VolumeReduce = false;
	bool FixedDirection = false;
	bool vepsilonEncountered = false;
	bool updateEachStepEncountered = false;
	bool dumpNodecoordsEncountered = false;
	bool dumpExtraNodecoordsEncountered = false;

	for (int i = 0; i < argc-1; ++i) {

		std::string arg = argv[i];
		size_t pos = arg.find('=');

		if (pos == std::string::npos)
		{
			if (arg == "--nodecoords") {
				dumpNodecoordsEncountered = true;
				continue;
			}

			if (arg == "--logstate") {
				LogState = true;
				continue;
			}
			if (arg == "--volumereduce") {
				VolumeReduce = true;
				continue;
			}
			if (arg == "--fixeddir") {
				FixedDirection = true;
				continue;
			}
			if (arg == "--dumpextra") {
				dumpExtraNodecoordsEncountered = true;
				continue;
			}
			std::cout << "UNKNOWN ARG " << arg << std::endl;
			printUsage();
			return UE_UNKNOWN_ARGUMENT;
		}
		std::string key = arg.substr(0, pos);
		std::string val = arg.substr(pos + 1);

		if (key == "-df") {
			forceStep = std::atof(val.c_str());
			forceStepEncountered = true;
			continue;
		}
		if (key == "-eps") {
			vepsilon = std::atof(val.c_str());
			vepsilonEncountered = true;
			continue;
		}
		if (key == "-d") {
			solveDelta = std::atof(val.c_str());
			continue;
		}
		if (key == "--compare_step_interval") {
			updateEachStep = std::atoi(val.c_str());
			updateEachStepEncountered = true;
			continue;
		}
		printUsage();
		return UE_UNKNOWN_ARGUMENT;
	}

	if (!(forceStepEncountered && vepsilonEncountered)) {
		printUsage();
		std::cout << "not enough parameters: df and eps requred" << std::endl;
		return UE_NOT_ENOUGH_ARGUMENTS;
	}

	auto system = createNodeSystem(argv[argc-1]);
	fDir = glm::normalize(system->getGrip()->getForce());
	auto outputFileName = generateOutputFileName(argv[argc-1]);

	//
	if(VolumeReduce)
		system->SetVolumeReduce();

	// фиксировать направление
	if(FixedDirection)
		system->FixPullingDirection();

	ForceDiagramStorage storage(system, outputFileName);
	if (!storage.parametrize(vepsilon, forceStep,LogState)) {
		return UE_INCOMPLETE_SYSTEM;
	}
	if (updateEachStepEncountered)
		storage.setUpdateEachStep(updateEachStep);

	if (dumpNodecoordsEncountered)
		dumpNodecoords(system, outputFileName + "_start.layout", dumpExtraNodecoordsEncountered);

	solve(system, 0.0, DBL_MAX, solveDelta, storage);

	if (dumpNodecoordsEncountered)
		dumpNodecoords(system, outputFileName + "_finish.layout", dumpExtraNodecoordsEncountered);

	t1 = time(0);
	int total,hours,min,sec;
	total = difftime(t1,t0);
	hours = total / 3600;
	min = (total % 3600) / 60;
	sec = (total % 3600) % 60;
	// текущее время расчета
	std::cout << "Total time hh: " << hours << " mm:" << min << " ss:" << sec <<"\n";
	return 0;
}


int main(int argc, char** argv)
{
	std::cout << argc;
	std::cout << std::endl;

	if (argc < 2) {
		printUsage();
		return UE_NOT_ENOUGH_ARGUMENTS;
	}

	//
	// try to choose calculation mode
	//
	std::string mode = argv[1];

	std::cout << mode << std::endl;
	if (mode == "diagram") {
		return diagram_main(argc - 2, argv + 2);
	}
	else if (mode == "animation") {
		return solve_main(argc - 2, argv + 2);
	}
	else {
		return solve_main(argc - 1, argv + 1);
	}
	return 0;
}

// hello world