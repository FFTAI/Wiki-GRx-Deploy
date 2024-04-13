#ifndef BASICFUNCTION_H
#define BASICFUNCTION_H
#include<math.h>
#include<Eigen/Dense>
#include<sys/time.h>
namespace Basics{
// rotation matrix to zyx euler
void MatrixToEuler_ZYX(Eigen::Matrix3d R, Eigen::Vector3d & Euler);
void MatrixToEuler_XYZ(Eigen::Matrix3d R, Eigen::Vector3d & Euler);
void Euler_XYZToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a);
void Euler_ZYXToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a);
Eigen::Matrix3d VelProjectionMatrix_EulerXYZ(Eigen::Vector3d Euler);
Eigen::Matrix3d RotX(double x);
Eigen::Matrix3d RotY(double y);
Eigen::Matrix3d RotZ(double z);
void euleraddoffset(Eigen::Vector3d & euler);
void eulersuboffset(Eigen::Vector3d & euler);
// skew
Eigen::Matrix3d skew(Eigen::Vector3d r);
void MatrixToEulerXYZ_(Eigen::Matrix3d R, Eigen::Vector3d &euler);
/**
 * @brief return interval time unit:s
 * 
 * @param ts 
 * @param te 
 * @return double 
 */
void matrixtoeulerxyz_(Eigen::Matrix3d R, Eigen::Vector3d &euler); 
double intervaltime(struct timeval ts, struct timeval te);
Eigen::Matrix3d rotZ(double z);
Eigen::Matrix3d rotY(double y);
Eigen::Matrix3d rotX(double x);
}

#endif // BASICFUNCTION_H
