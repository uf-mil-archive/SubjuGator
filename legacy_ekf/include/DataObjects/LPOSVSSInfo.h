#ifndef DATAOBJECTS_LPOSVSSINFO_H
#define DATAOBJECTS_LPOSVSSINFO_H

#include <Eigen/Dense>

namespace subjugator {
	struct LPOSVSSInfo {
		Eigen::Vector3d position_NED;
		Eigen::Vector4d quaternion_NED_B;
		Eigen::Vector3d velocity_NED;
		Eigen::Vector3d angularRate_BODY;
		Eigen::Vector3d acceleration_BODY;
	};
}

#endif /* DATAOBJECTS_LPOSVSSINFO_H */

