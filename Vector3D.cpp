#include "vector3D.h"

using MimicArm::Vector3D;

double Vector3D::Magnitude() const {
	return sqrt(dot(*this));
}

Vector3D Vector3D::Normalize() const {
	double mag = Magnitude();
	return Vector3D(x / mag, y / mag, z / mag);
}

Vector3D Vector3D::operator-(const Vector3D &other) const {
	return Vector3D(x - other.x, y - other.y, z - other.z);
}

double Vector3D::dot(const Vector3D &other) const {
	return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross(const Vector3D &other) const {
	double a = y * other.z - z * other.y;
	double b = z * other.x - x * other.z;
	double c = x * other.y - y * other.x;
	return Vector3D(a, b, c);
}

Vector3D Vector3D::projection(const Vector3D &other) const {
	return other * (dot(other) / other.dot(other));
}

double Vector3D::AngleBetween(const Vector3D &other) const {
	return acos(dot(other) / (Magnitude() * other.Magnitude()));
}

std::ostream & MimicArm::operator<< (std::ostream &out, Vector3D const &v) {
	return out << '<' << v.x << ", " << v.y << ", " << v.z << '>';
}

Vector3D operator*(const Vector3D &v, double mag) {
	return Vector3D(mag * v.x, mag * v.y, mag * v.z);
}

Vector3D operator*(double mag, const Vector3D &v) {
	return v * mag;
}