#pragma once

#define _USE_MATH_DEFINES 
#include <cmath>
#include <iostream>

namespace MimicArm {
	class Vector3D {
	public:
		const double x;
		const double y;
		const double z;
		Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

		double Magnitude() const;

		Vector3D Normalize() const;

		Vector3D operator-(const Vector3D &other) const;
		double dot(const Vector3D &other) const;
		Vector3D cross(const Vector3D &other) const;
		Vector3D projection(const Vector3D &other) const;
		double AngleBetween(const Vector3D &other) const;

		friend std::ostream & operator<< (std::ostream &out, const Vector3D &t);
	};

}

MimicArm::Vector3D operator*(const MimicArm::Vector3D &v, double mag);
MimicArm::Vector3D operator*(double mag, const MimicArm::Vector3D &v);