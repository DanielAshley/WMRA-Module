#include "WMRA_module.h"
#include "Arm.h"
#include "WmraTypes.h"

using namespace WMRA;
Arm arm;

WMRA_module::WMRA_module(void)
{
	//arm.initialize();
}

bool WMRA_module::initialize()
{
	return arm.initialize();
}

bool WMRA_module::autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking)
{
	return arm.autonomous(dest, crodFr, blocking);
}

bool WMRA_module::teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr)
{
	return arm.teleoperation(dest, cordFr);
}

bool WMRA_module::teleoperation(WMRA::Pose data)
{
	return arm.teleoperation(data);
}

bool WMRA_module::openGripper(bool blocking)
{
	return arm.openGripper(blocking);
}

bool WMRA_module::closeGripper(bool blocking)
{
	return arm.closeGripper(blocking);
}

bool WMRA_module::isGripperOpen()
{
	return arm.isGripperOpen();
}

bool WMRA_module::toReady(bool blocking)
{
	return arm.toReady(blocking);
}

bool WMRA_module::ready2Park(bool blocking)
{
	return arm.ready2Park(blocking);
}

bool WMRA_module::park2Ready(bool blocking)
{
	return arm.park2Ready(blocking);
}

bool WMRA_module::motionComplete()
{
	return arm.motionComplete();
}

bool WMRA_module::moveJoint(int jointNum, double angle, int ref)
{
	return arm.moveJoint(jointNum, angle, ref);
}

WMRA::Pose WMRA_module::getPose()
{
	return arm.getPose();
}

WMRA::JointValueSet WMRA_module::getJointAngles()
{
	return arm.getJointAngles();
}


//void WMRA_module::sendValues()
//{
//	arm.sendValues();
//}

bool WMRA_module::isInitialized()
{
	return arm.isInitialized();
}

bool WMRA_module::setInitialJointAngles(WMRA::JointValueSet& joints)
{
	return arm.setInitialJointAngles(joints);
}

void WMRA_module::sendData(void* aArg)
{
	arm.sendData(aArg);
}

WMRA::JointValueSet WMRA_module::getLastKnownJointPosition()
{
	return arm.getLastKnownJointPosition();
}

WMRA_module::~WMRA_module(void)
{
}
