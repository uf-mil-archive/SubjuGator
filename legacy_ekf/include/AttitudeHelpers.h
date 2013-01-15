#ifndef ATTITUDEHELPERS_H
#define ATTITUDEHELPERS_H

#include <Eigen/Dense>

namespace subjugator
{
	class AttitudeHelpers
	{
	public:
		static Eigen::Vector3d LocalGravity(double lat, double depth);
		static double Markov_wStdDev(double dt, double T, double sigma);
		static Eigen::Vector3d RotationToEuler(const Eigen::Matrix3d& R);
		static Eigen::Matrix3d EulerToRotation(const Eigen::Vector3d& rpy);
		static double DAngleDiff(double a, double b);
		static double DAngleClamp(double a);
		static Eigen::Vector4d RotationToQuaternion(const Eigen::Matrix3d& R);
		static Eigen::Matrix3d VectorSkew3(const Eigen::Vector3d& v);
		static Eigen::MatrixXd DiagMatrixFromVector(const Eigen::VectorXd& v);
		static Eigen::VectorXd Tanh(const Eigen::VectorXd& v);
		static Eigen::VectorXd Sech(const Eigen::VectorXd& v);
	};
}

#endif /* ATTITUDEHELPERS_H */
