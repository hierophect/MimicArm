#include "Matrix3D.h"

using MimicArm::Matrix3D;
using MimicArm::Vector3D;

Matrix3D operator*(const Matrix3D &lhs, const Matrix3D &rhs)
{
	Matrix3D product;
	for (int i = 0; i < lhs.size(); i++) {
		for (int j = 0; j < lhs.size(); j++) {
			for (int k = 0; k < lhs.size(); k++) {
				product(i, j) += lhs(i, k) * rhs(k, j);
			}
		}
	}
	return product;
}

Matrix3D& Matrix3D::operator+=(const Matrix3D &other)
{
	for (int i = 0; i < matrix.size(); i++) {
		for (int j = 0; j < matrix[i].size(); j++) {
			matrix[i][j] += other(i, j);
		}
	}
	return *this;
}

Matrix3D& Matrix3D::operator*=(double d)
{
	for (int i = 0; i < matrix.size(); i++) {
		for (int j = 0; j < matrix[i].size(); j++) {
			matrix[i][j] *= d;
		}
	}
	return *this;
}
