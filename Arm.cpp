#include "stdafx.h"
#include "Arm.h"

#define _USE_MATH_DEFINES 
#include <cmath>
#include <algorithm>
#include <exception>
#include <sstream>

#include "Serial.h"
#include "resource.h"
#include "Vector3D.h"
#include "Matrix3D.h"

#define DPRINT(w) do {							\
	std::stringstream stream;					\
	stream << w;								\
	OutputDebugString(stream.str().c_str());	\
} while(0)

double to_degrees(double radians)
{
	return radians * 180.0 / M_PI;
}

MimicArm::Vector3D vectorBetween(const Joint* pJoints, JointType a, JointType b)
{
	MimicArm::Vector3D veca(pJoints[a].Position.X, pJoints[a].Position.Y, pJoints[a].Position.Z);
	MimicArm::Vector3D vecb(pJoints[b].Position.X, pJoints[b].Position.Y, pJoints[b].Position.Z);
	return veca - vecb;
}

double AngleBetween(const Joint* pJoints, JointType a, JointType b, JointType c)
{
	TrackingState trackingStates[] = {
		pJoints[a].TrackingState,
		pJoints[b].TrackingState,
		pJoints[c].TrackingState
	};

	if (std::any_of(
			std::begin(trackingStates),
			std::end(trackingStates),
			[](TrackingState ts) { return ts == TrackingState_NotTracked; })) {
		return 0.0;
	}

	if (std::all_of(
			std::begin(trackingStates),
			std::end(trackingStates),
			[](TrackingState ts) { return ts == TrackingState_Inferred; })) {
		return 0.0;
	}

	MimicArm::Vector3D veca(pJoints[a].Position.X, pJoints[a].Position.Y, pJoints[a].Position.Z);
	MimicArm::Vector3D vecb(pJoints[b].Position.X, pJoints[b].Position.Y, pJoints[b].Position.Z);
	MimicArm::Vector3D vecc(pJoints[c].Position.X, pJoints[c].Position.Y, pJoints[c].Position.Z);
	DPRINT("veca " << veca << '\n');
	DPRINT("vecb " << vecb << '\n');
	DPRINT("vecc " << veca << '\n');

	return to_degrees((veca - vecb).AngleBetween(vecb - vecc));
}

MimicArm::Vector3D getNormal(const Joint* pJoints, JointType a, JointType b, JointType c)
{
	MimicArm::Vector3D veca(
		pJoints[a].Position.X,
		pJoints[a].Position.Y,
		pJoints[a].Position.Z);
	MimicArm::Vector3D vecb(
		pJoints[b].Position.X,
		pJoints[b].Position.Y,
		pJoints[b].Position.Z);
	MimicArm::Vector3D vecc(
		pJoints[c].Position.X,
		pJoints[c].Position.Y,
		pJoints[c].Position.Z);
	return (veca - vecb).cross(vecb - vecc);
}

MimicArm::Vector3D getUpperArmNormal(const Joint *pJoints)
{
	return getNormal(pJoints, JointType_SpineMid, JointType_ShoulderRight, JointType_ElbowRight);
}

MimicArm::Vector3D getBodyNormal(const Joint* pJoints)
{
	return getNormal(pJoints, JointType_ShoulderLeft, JointType_SpineMid, JointType_SpineBase);
}

MimicArm::Vector3D getSidwaysNormal(const Joint* pJoints, const MimicArm::Vector3D &bodyNormal)
{
	return bodyNormal.cross(vectorBetween(pJoints, JointType_SpineMid, JointType_SpineShoulder));
}

MimicArm::Vector3D projectionOntoPlane(
	const MimicArm::Vector3D &normal, MimicArm::Vector3D &v)
{
	// Supposedly this works http://robotics.usc.edu/~leedison/SukJinLee%202011%20Summer%20USC%20Provost%20Final%20report.pdf
	//return v * (normal.dot(normal)) - normal * (normal.dot(v));

	return v - ((v.dot(normal) / normal.dot(normal)) * normal);
}


MimicArm::Arm::Arm() :
old_position(0)
{
#if SEND_COMMANDS
	if (!serialPort.Open(3)) {
		throw std::exception();
	}
#endif
}

// 0 cooresponds with 30 degrees
// 511 corresponds with 180 degrees
// 1023 cooresponds with 330 degrees
int angleToMotorPosition(double degrees, double offset = 30.0, const int lowerBound = 0, const int upperBound = 1023)
{
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

	int motorPosition = floor(degrees - offset) / 300 * 1024;
	return std::min(upperBound, std::max(motorPosition, lowerBound));
}

int bigMotorCalc(double degrees)
{
	const int upperBound = 3581;
	const int middle = 2047;
	const int lowerBound = 500;
	//const int numPositions = upperBound - lowerBound + 1;
	//int motorPosition = floor((degrees + 90) / 180 * numPositions) + lowerBound;
	//int motorPosition = (int)(degrees * 17.116666666667) + middle;
	int motorPosition = (int)(degrees * 17.11666666666);
	return std::min(upperBound, std::max(motorPosition, lowerBound));
}

void MimicArm::Arm::send(int motorValue, double value) {
	int angle;
	value = to_degrees(value);
#if BIG_MOTOR
	if (motorValue == BIG_MOTOR) {
		angle = bigMotorCalc(value);
	} else {
#endif
		switch (motorValue) {
#if BICEP_MOTOR
		case BICEP_MOTOR:
			angle = 1023 - angleToMotorPosition(value, 120.0, 0, 1023);
			break;
#endif
#if UP_DOWN_MOTOR
		case UP_DOWN_MOTOR:
			angle = 1023 - angleToMotorPosition(value, -45.0, 0, 1023);
			break;
#endif
#if ELBOW_MOTOR
		case ELBOW_MOTOR:
			angle = 1023 - angleToMotorPosition(value, 30.0, 0, 1023);
			break;
#endif
#if FORWARD_BACK_MOTOR
		case FORWARD_BACK_MOTOR:
			angle = 1023 - angleToMotorPosition(value, -75.0, 0, 1023);
			break;
#endif
		default:
			angle = angleToMotorPosition(value, 30.0, 0, 1023);
			break;
		}
#if BIG_MOTOR
	}
#endif
	MimicArm::Arm::send(motorValue, angle);
}

void MimicArm::Arm::send(int motorValue, int value) {
	char str[100];
	int size = sprintf(str, "%d %d\n", motorValue, value);
	if (size > 0) {
		DPRINT("Sending " << str);
#if SEND_COMMANDS
		serialPort.SendData(str, size);
#endif
	}
}

// t = q / r
Vector4 operator/(const Vector4& q, const Vector4& r)
{
	http://www.mathworks.com/help/aeroblks/quaterniondivision.html
	double mag = r.w*r.w + r.x * r.x + r.y * r.y + r.z * r.z;
	double t0 = (r.w * q.w + r.x * q.x + r.y * q.y + r.z * q.z) / mag;
	double t1 = (r.w * q.x - r.x * q.w - r.y * q.z + r.z * q.y) / mag;
	double t2 = (r.w * q.y + r.x * q.z - r.y * q.w - r.z * q.z) / mag;
	double t3 = (r.w * q.z - r.x * q.y + r.y * q.x - r.z * q.w) / mag;
	Vector4 v = {
		t0, t1, t2, t3
	};
	return v;
}

double yawPitchRoll(const JointOrientation *pJointOrientation)
{
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	Vector4 q = pJointOrientation[JointType_ShoulderRight].Orientation
		/ pJointOrientation[JointType_ElbowRight].Orientation;
	float q0 = q.w;
	float q1 = q.x;
	float q2 = q.y;
	float q3 = q.z;
	double roll = atan2(2 * q0 * q1 + q2 * q3, 1 - 2 * (q1 * q1 + q2 * q2));
	double pitch = asin(2 * (q0 * q2 - q3 * q1));
	double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
	//DPRINT("roll " << to_degrees(roll) << " pitch " << to_degrees(pitch) << " yaw " << to_degrees(yaw) << '\n');
	return roll;

}

MimicArm::Matrix3D skewSymetricCrossProductMatrix(const MimicArm::Vector3D &vec)
{
	std::array<std::array<double, 3>, 3> x = { {
		{ 0.0, -vec.z, vec.y },
		{ vec.z, 0.0, -vec.x },
		{ -vec.y, vec.x, 0.0 },
		} };
	return MimicArm::Matrix3D(x);
}

MimicArm::Matrix3D eye()
{
	std::array<std::array<double, 3>, 3> x = { {
		{ 1.0, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0 },
		{ 0.0, 0.0, 1.0 },
		} };
	return MimicArm::Matrix3D(x);
}

// Find rotation matrix of a onto b
MimicArm::Matrix3D rotationMatrix(const MimicArm::Vector3D &a, const MimicArm::Vector3D &b)
{
	using MimicArm::Matrix3D;
	using MimicArm::Vector3D;
	Vector3D normA = a.Normalize();
	Vector3D normB = b.Normalize();

	Vector3D cross = normA.cross(normB);
	Matrix3D ssc = skewSymetricCrossProductMatrix(cross);
	Matrix3D rotationMatrix = eye() + ssc + ssc * ssc * ((1 - normA.dot(normB)) / pow(cross.Magnitude(), 2));
	return rotationMatrix;
}

// Verify...
MimicArm::Vector3D multiply(const MimicArm::Vector3D &lhs, const MimicArm::Matrix3D &rhs)
{
	using MimicArm::Vector3D;
	std::array<double, 3> result;
	for (int i = 0; i < 3; i++) {
		result[i] = Vector3D(rhs(i, 0), rhs(i, 1), rhs(i, 2)).dot(lhs);
	}
	return Vector3D(result[0], result[1], result[2]);
}

double bicepRotation(const MimicArm::Vector3D &upperArm, const MimicArm::Vector3D &lowerArm)
{
	MimicArm::Matrix3D rotMat = rotationMatrix(upperArm, { 1, 0, 0 });
	//DPRINT(multiply(upperArm.Normalize(), rotMat) << '\n');
	MimicArm::Vector3D rotVec = multiply(lowerArm, rotMat);
	double result = atan2(-rotVec.y , -rotVec.z);
	if (result < 0) {
		result += 2 * M_PI;
	}
	return result;
}

void MimicArm::Arm::sendPositions(const Joint *pJoints, const JointOrientation *pJointOrientation)
{
	//DPRINT(iteration << '\n');
	//DPRINT(
	//"Angle of Spine to Upper Arm      = "
	//	<< AngleBetween(pJoints, JointType_SpineShoulder, JointType_ShoulderRight, JointType_ElbowRight)
	//	<< '\n');

	MimicArm::Vector3D bodyNormal = getBodyNormal(pJoints);
	MimicArm::Vector3D upperArm = vectorBetween(pJoints, JointType_ElbowRight, JointType_ShoulderRight);
	MimicArm::Vector3D lowerArm = vectorBetween(pJoints, JointType_ElbowRight, JointType_WristRight);
	MimicArm::Vector3D spine = vectorBetween(pJoints, JointType_SpineShoulder, JointType_SpineMid);

	MimicArm::Vector3D upperarmNormal = getUpperArmNormal(pJoints);

	double angle;
#if ELBOW_MOTOR
	angle = upperArm.AngleBetween(lowerArm);
	DPRINT("Angle between lowerarm and upperarm "
		<< angle
		<< " "
		<< to_degrees(angle)
		<< " "
		<< angleToMotorPosition(to_degrees(angle))
		<< '\n');
	send(ELBOW_MOTOR, angle);
#endif

#if BICEP_MOTOR
	//angle = -yawPitchRoll(pJointOrientation);
	angle = bicepRotation(upperArm, lowerArm);
	DPRINT("Bicep Rotation "
		<< angle
		<< " "
		<< to_degrees(angle)
		<< " "
		<< angleToMotorPosition(to_degrees(angle))
		<< '\n');
	send(BICEP_MOTOR, angle);
#endif

	//DPRINT("Body Normal " << bodyNormal << '\n');
	//DPRINT("Angle of Body to Upper Arm = "
	//	<< AngleBetween(pJoints, JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight)
	//	<< '\n');
#if UP_DOWN_MOTOR
	DPRINT("Angle of Body to Upperarm in plane of body = "
		<< to_degrees(spine.AngleBetween(upperArm))
		<< '\n');
	send(UP_DOWN_MOTOR, spine.AngleBetween(upperArm));
#endif

#if FORWARD_BACK_MOTOR
	angle = bodyNormal.AngleBetween(upperArm) * SMOOTHING_FACTOR + old_position * (1 - SMOOTHING_FACTOR);
	old_position = angle;
	DPRINT("Angle of Body Normal to Upperarm = "
		<< to_degrees(bodyNormal.AngleBetween(upperArm))
		<< '\n');
	send(FORWARD_BACK_MOTOR, angle);
#endif

}