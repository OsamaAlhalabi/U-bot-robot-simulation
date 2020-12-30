// File:          AvoidObsticels.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <iostream>
#include <vector>
#include <string>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <arm.h>
#include <gripper.h>
#include <base.h>

template<typename... Args>
std::string format_string(const std::string& format, Args... args)
{
	const auto size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
	const auto buffer = std::make_unique<char[]>(size);

	std::snprintf(buffer.get(), size, format.c_str(), args...);

	return std::string(buffer.get(), buffer.get() + size - 1);
}


// All the webots classes are defined in the "webots" namespace
using namespace webots;

void passiveWait(double sec, Robot* robot) {
	int timeStep = (int)robot->getBasicTimeStep();
	double start_time = robot->getTime();
	do {
		robot->step(timeStep);
	} while (start_time + sec > robot->getTime());
}


class KukaRobot : public Robot {
private:
	std::vector<Motor*> wheels;
	std::vector<DistanceSensor*> distanceSensors;
public:
	KukaRobot() : Robot() {
		
	}
	void init() {
		int timeStep = (int)this->getBasicTimeStep();
		for (int i = 0; i < 4; i++) {
			auto wheel = this->getMotor(format_string("wheel%d", i + 1));
			wheel->setPosition(INFINITY);
			wheel->setVelocity(0.f);
		}

		for (int i = 0; i < 3; i++) {
			auto ds = this->getDistanceSensor(format_string("front ds %d", i + 1));
			ds->enable(timeStep);
		}

		this->getDistanceSensor("front right ds")->enable(timeStep);
		this->getDistanceSensor("front left ds")->enable(timeStep);

		base_init();
		arm_init();
		gripper_init();
		passiveWait(2.0, this);
	}
};

int main(int argc, char** argv) {
	// create the Robot instance.
	KukaRobot* kuka = new KukaRobot();
	kuka->init();

	// get the time step of the current world.
	int timeStep = (int)kuka->getBasicTimeStep();
	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (kuka->step(timeStep) != -1) {
		// Read the sensors:
		// Enter here functions to read sensor data, like:
		//  double val = ds->getValue();

		// Process sensor data here.

		// Enter here functions to send actuator commands, like:
		//  motor->setPosition(10.0);
	};

	// Enter here exit cleanup code.

	delete kuka;
	return 0;
}
