#pragma once

#include "Serial.h"

#define SEND_COMMANDS 1

// 500 2047 3581
#define BICEP_MOTOR 2
#define ELBOW_MOTOR 3
// motor 4
#define UP_DOWN_MOTOR 5
#define FORWARD_BACK_MOTOR 6
#define SMOOTHING_FACTOR 0.5

#define BIG_MOTOR 0

namespace MimicArm {
	class Arm
	{
	private:
		double old_position;
		CSerial serialPort;
		void send(int motorIndex, double value);
		void send(int motorIndex, int value);
	public:
		Arm();
		void sendPositions(const Joint *pJoints, const JointOrientation *pJointOrientation);
	};
}
