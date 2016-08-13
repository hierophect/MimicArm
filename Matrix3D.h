#pragma once

#include <array>

#include "Vector3D.h"

namespace MimicArm {
	class Matrix3D;
}

MimicArm::Matrix3D operator*(const MimicArm::Matrix3D &lhs, const MimicArm::Matrix3D &rhs);

namespace MimicArm {
	class Matrix3D
	{
		std::array<std::array<double, 3>, 3> matrix;
	public:
		Matrix3D() : matrix() {}
		Matrix3D(std::array<std::array<double, 3>, 3> mat) : matrix(mat) {}
		double& operator()(int row, int col) {
			return matrix[row][col];
		}

		double operator()(int row, int col) const {
			return matrix[row][col];
		}

		int size() const {
			return matrix.size();
		}

		Matrix3D& operator*=(double d);

		Matrix3D& operator*=(const Matrix3D &other) {
			using std::swap;
			Matrix3D product(*this * other);
			swap(*this, product);
			return *this;
		}

		Matrix3D& operator+=(const Matrix3D &other);
	};
}

inline MimicArm::Matrix3D operator+(MimicArm::Matrix3D lhs, const MimicArm::Matrix3D& rhs)
{
	lhs += rhs;
	return lhs;
}

inline MimicArm::Matrix3D operator*(MimicArm::Matrix3D lhs, double rhs) {
	lhs *= rhs;
	return lhs;
}

inline MimicArm::Matrix3D operator*(double rhs, const MimicArm::Matrix3D &lhs) {
	return lhs * rhs;
}