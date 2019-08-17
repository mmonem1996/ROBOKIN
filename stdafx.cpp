// stdafx.cpp : source file that includes just the standard includes
// ConsoleApplication4.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"

FLOAT toRadians(FLOAT Angle_in_degrees)
{
	return (Angle_in_degrees*PI)/180.f;
}

FLOAT toDegrees(FLOAT Angle_in_Radians)
{
	return (Angle_in_Radians*180.f) / PI;
}

FLOAT Max_Of(std::initializer_list<FLOAT> numbers)
{
	FLOAT result = *numbers.begin();
	for (auto i = numbers.begin() + 1; i < numbers.end(); i++)
	{
		result = Max(result, *i);
	}
	return result;
}

Matrix3x3 Get_Rotation(const Matrix4x4& m) {
	Matrix3x3 out;
	out <<
		m(0, 0), m(0, 1), m(0, 2),
		m(1, 0), m(1, 1), m(1, 2),
		m(2, 0), m(2, 1), m(2, 2);
	return out;
}