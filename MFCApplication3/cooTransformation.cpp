#pragma once

#include "pch.h"
#include "cooTransformation.h"

#include <math.h>
#include <Eigen/Dense>


using namespace Eigen;

#define PI 3.14159265


//计算坐标系 B 相对于坐标系 A 的位姿 C; 即 C = inv(A)*B
void Transformate(PoseTydeDef A, PoseTydeDef B, PoseTydeDef* C)
{
	// 计算A坐标系的齐次变换矩阵
	double yaw_A = A.Rz / 180 * PI;
	double pitch_A = A.Ry / 180 * PI;
	double roll_A = A.Rx / 180 * PI;

	Vector3d eulerAngle(yaw_A, pitch_A, roll_A);
	Matrix3d rotation_matrix;

	Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));
	rotation_matrix = yawAngle * pitchAngle * rollAngle;

	Eigen::Translation3d init_translation(A.x, A.y, A.z);

	Eigen::Matrix4d T_A = (init_translation * rotation_matrix).matrix();

	// 计算B坐标系的齐次变换矩阵

	double yaw_B = B.Rz / 180 * PI;
	double pitch_B = B.Ry / 180 * PI;
	double roll_B = B.Rx / 180 * PI;

	Vector3d eulerAngle_B(yaw_B, pitch_B, roll_B);  //初始化欧拉角(Z - Y - X，即RPY)
	Matrix3d rotation_matrix_B;

	Eigen::AngleAxisd rollAngle_B(AngleAxisd(eulerAngle_B(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle_B(AngleAxisd(eulerAngle_B(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle_B(AngleAxisd(eulerAngle_B(0), Vector3d::UnitZ()));
	rotation_matrix_B = yawAngle_B * pitchAngle_B * rollAngle_B;

	Eigen::Translation3d init_translation_B(B.x, B.y, B.z);

	Eigen::Matrix4d T_B = (init_translation_B * rotation_matrix_B).matrix();

	// 计算齐次变换矩阵C

	Matrix4d T_C = T_A.inverse() * T_B;

	Matrix3d rotation_matrix_C = T_C.topLeftCorner(3, 3);

	Vector3d euler_C2 = rotation_matrix_C.eulerAngles(2, 1, 0);

	C->Rx = euler_C2.z();   // 旋转矩阵转欧拉角(Z - Y - X，即RPY)
	C->Ry = euler_C2.y();
	C->Rz = euler_C2.x();

	C->x = T_C(0, 3);
	C->y = T_C(1, 3);
	C->z = T_C(2, 3);
}



// 坐标系 A 继续旋转坐标系 B，得到坐标系 C， 即 C = A*B;
void cooMulti(PoseTydeDef A, PoseTydeDef B, PoseTydeDef* C)
{
	// 计算A坐标系的齐次变换矩阵

	double yaw_A = A.Rz / 180 * PI;
	double pitch_A = A.Ry / 180 * PI;
	double roll_A = A.Rx / 180 * PI;

	Vector3d eulerAngle(yaw_A, pitch_A, roll_A);
	Matrix3d rotation_matrix;

	Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));
	rotation_matrix = yawAngle * pitchAngle * rollAngle;

	Eigen::Translation3d init_translation(A.x, A.y, A.z);

	Eigen::Matrix4d T_A = (init_translation * rotation_matrix).matrix();

	// 计算B坐标系的齐次变换矩阵

	double yaw_B = B.Rz / 180 * PI;
	double pitch_B = B.Ry / 180 * PI;
	double roll_B = B.Rx / 180 * PI;

	Vector3d eulerAngle_B(yaw_B, pitch_B, roll_B);  //初始化欧拉角(Z - Y - X，即RPY)
	Matrix3d rotation_matrix_B;

	Eigen::AngleAxisd rollAngle_B(AngleAxisd(eulerAngle_B(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle_B(AngleAxisd(eulerAngle_B(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle_B(AngleAxisd(eulerAngle_B(0), Vector3d::UnitZ()));
	rotation_matrix_B = yawAngle_B * pitchAngle_B * rollAngle_B;

	Eigen::Translation3d init_translation_B(B.x, B.y, B.z);

	Eigen::Matrix4d T_B = (init_translation_B * rotation_matrix_B).matrix();

	// 计算坐标系 C
	Matrix4d T_C = T_A * T_B;

	Matrix3d rotation_matrix_C = T_C.topLeftCorner(3, 3);

	Vector3d euler_C2 = rotation_matrix_C.eulerAngles(2, 1, 0);

	C->Rx = euler_C2.z();   // 旋转矩阵转欧拉角(Z - Y - X，即RPY)
	C->Ry = euler_C2.y();
	C->Rz = euler_C2.x();

	C->x = T_C(0, 3);
	C->y = T_C(1, 3);
	C->z = T_C(2, 3);
}
