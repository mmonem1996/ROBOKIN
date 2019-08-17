// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"
//#include <DirectXMath.h>
#include <vector>
#include <stdio.h>
#include <tchar.h>
#include <Dense>
#include <iostream>
#define PI 3.14159265359f
typedef double FLOAT;
typedef Eigen::Matrix<FLOAT, 3, 1> Vector3;
typedef Eigen::Matrix<FLOAT, 4, 1> Vector4;


typedef Eigen::Matrix<FLOAT, 3, 3> Matrix3x3;
typedef Eigen::Matrix<FLOAT, 4, 4> Matrix4x4;
typedef Eigen::Matrix<FLOAT, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

FLOAT toRadians(FLOAT Angle_in_degrees);
FLOAT toDegrees(FLOAT Angle_in_Radians);
#define Throw_Error(str_Error_Message) {printf(str_Error_Message);system("Pause");}
#define Error_Message(str_Error_Message){printf(str_Error_Message);}
#define Max(x,y) (x > y ? x : y)

FLOAT Max_Of(std::initializer_list<FLOAT> numbers);

Matrix3x3 Get_Rotation(const Matrix4x4& m);
// TODO: reference additional headers your program requires here
