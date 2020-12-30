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
#include <webots/Radar.hpp>
#include <webots/Motor.hpp>
#include <arm.h>
#include <gripper.h>
#include <base.h>

#define MAX_SPEED 25.f

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


		//this->getRadar("radar")->enable(timeStep);

		base_init();
		arm_init();
		gripper_init();
		passiveWait(2.0, this);
	}

	Motor* getFrontRightWheel() {
		return this->getMotor("wheel1");
	}

	Motor* getFrontLeftWheel() {
		return this->getMotor("wheel2");
	}

	Motor* getBackRightWheel() {
		return this->getMotor("wheel3");
	}

	Motor* getBackLeftWheel() {
		return this->getMotor("wheel4");
	}

	DistanceSensor* getFrontRightDistanceSensor() {
		return this->getDistanceSensor("front right ds");
	}

	DistanceSensor* getFrontLeftDistanceSensor() {
		return this->getDistanceSensor("front left ds");
	}

	std::vector<DistanceSensor*> getFrontDistanceSensor() {
		std::vector<DistanceSensor*> ve;
		ve.push_back(this->getDistanceSensor("front ds 1"));
		ve.push_back(this->getDistanceSensor("front ds 2"));
		ve.push_back(this->getDistanceSensor("front ds 3"));
		
		return ve;
	}

	Radar* getCenterRadar() {
		return this->getRadar("radar");
	}

	void loadObject(float x, float y, float z) {
		 gripper_release();
		 arm_ik(x, y, z);
		 passiveWait(2.0, this);
		 gripper_release();
		 arm_set_height(ARM_FRONT_FLOOR);
		 passiveWait(5.0, this);
		 gripper_grip();
		 passiveWait(1.0, this);
		 arm_reset();
		 passiveWait(3.0, this);
		 arm_set_height(ARM_BACK_PLATE_HIGH);
		 passiveWait(3.0, this);
		 gripper_release();
		 passiveWait(1.0, this);
		 arm_reset();
	}

};

int main(int argc, char** argv) {
	// create the Robot instance.
	KukaRobot* kuka = new KukaRobot();
	kuka->init();

	//std::cout << kuka->getCenterRadar()->getNumberOfTargets();
	// get the time step of the current world.
	int timeStep = (int)kuka->getBasicTimeStep();
	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (kuka->step(timeStep) != -1) {

		double leftSpeed = 0.5 * MAX_SPEED;
		double rightSpeed = 0.5 * MAX_SPEED;

		auto front_ds = kuka->getFrontDistanceSensor();

		// detect obstacles
		bool right_obstacle =
			front_ds[0]->getValue() < 950.f ||
			front_ds[1]->getValue() < 950.f ||
			front_ds[2]->getValue() < 950.f ||
			kuka->getFrontRightDistanceSensor()->getValue() < 950.f;

		// detect obstacles
		bool left_obstacle =
			front_ds[0]->getValue() < 950.f ||
			front_ds[1]->getValue() < 950.f ||
			front_ds[2]->getValue() < 950.f ||
			kuka->getFrontLeftDistanceSensor()->getValue() < 950.f;

		if (left_obstacle) {
			leftSpeed = 0.5 * MAX_SPEED;
			rightSpeed = -0.5 * MAX_SPEED;
		}
		else {
			if (right_obstacle) {
				leftSpeed = -0.5 * MAX_SPEED;
				rightSpeed = 0.5 * MAX_SPEED;
			}
		}
		
		kuka->getFrontLeftWheel()->setVelocity(leftSpeed);
		kuka->getFrontRightWheel()->setVelocity(rightSpeed);
		kuka->getBackLeftWheel()->setVelocity(leftSpeed);
		kuka->getBackRightWheel()->setVelocity(rightSpeed);
	}


	// Enter here exit cleanup code.

	delete kuka;
	return 0;
}
