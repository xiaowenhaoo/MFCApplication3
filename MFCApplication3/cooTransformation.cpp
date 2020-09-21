// ×ø±êÏµ±ä»»
#include "pch.h"
#pragma once

#include <math.h>
#include <Eigen/Dense>


using namespace Eigen;

#define PI 3.14159265



// rotation about X axis;
// theta is in degrees
// T is a homogeneous transformation (4*4)
void trotx(double theta, double T[][4])
{
	double theta_rad = theta * PI / 180.f;

	T[0][0] = 1;
	T[1][0] = 0;
	T[2][0] = 0;
	T[3][0] = 0;

	T[0][1] = 0;
	T[1][1] = cos(theta_rad);
	T[2][1] = sin(theta_rad);
	T[3][1] = 0;

	T[0][2] = 0;
	T[1][2] = -sin(theta_rad);
	T[2][2] = cos(theta_rad);
	T[3][2] = 0;

	T[0][3] = 0;
	T[1][3] = 0;
	T[2][3] = 0;
	T[3][3] = 1;
}


// rotation about Y axis;
// theta is in degrees
// T is a homogeneous transformation (4*4)
void troty(double theta, double T[][4])
{
	double theta_rad = theta * PI / 180.f;

	T[0][0] = cos(theta_rad);
	T[1][0] = 0;
	T[2][0] = -sin(theta_rad);
	T[3][0] = 0;

	T[0][1] = 0;
	T[1][1] = 1;
	T[2][1] = 0;
	T[3][1] = 0;

	T[0][2] = sin(theta_rad);
	T[1][2] = 0;
	T[2][2] = cos(theta_rad);
	T[3][2] = 0;

	T[0][3] = 0;
	T[1][3] = 0;
	T[2][3] = 0;
	T[3][3] = 1;
}


// rotation about Z axis;
// theta is in degrees
// T is a homogeneous transformation (4*4)
void trotz(double theta, double T[][4])
{
	double theta_rad = theta * PI / 180.f;

	T[0][0] = cos(theta_rad);
	T[1][0] = sin(theta_rad);
	T[2][0] = 0;
	T[3][0] = 0;

	T[0][1] = -sin(theta_rad);
	T[1][1] = cos(theta_rad);
	T[2][1] = 0;
	T[3][1] = 0;

	T[0][2] = 0;
	T[1][2] = 0;
	T[2][2] = 1;
	T[3][2] = 0;

	T[0][3] = 0;
	T[1][3] = 0;
	T[2][3] = 0;
	T[3][3] = 1;
}


