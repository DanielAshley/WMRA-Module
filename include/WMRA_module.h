#pragma once

namespace WMRA{
	class Pose;
	class Omni_data;
	class JointValueSet;
	enum CordFrame;
};


namespace WMRA{
	class WMRA_module
	{
	public:
		WMRA_module(void);
		bool initialize();
		bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking = true);
		bool teleoperation(WMRA::Pose data);
		bool teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr);
		bool openGripper(bool blocking = true);
		bool closeGripper(bool blocking = true);
		bool isGripperOpen();
		bool toReady(bool blocking = true);
		bool ready2Park(bool blocking = true);
		bool park2Ready(bool blocking = true);
		bool motionComplete();
		bool moveJoint(int jointNum, double angle, int ref);
		WMRA::Pose getPose();
		WMRA::JointValueSet getJointAngles();
		/*void sendValues();*/
		bool isInitialized();
		bool setInitialJointAngles(WMRA::JointValueSet& joints);
		static void sendData(void* aArg);
		WMRA::JointValueSet getLastKnownJointPosition();
		~WMRA_module(void);
	};
};

