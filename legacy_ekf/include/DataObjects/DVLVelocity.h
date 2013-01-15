#ifndef DATAOBJECTS_DVLVELOCITY_H
#define DATAOBJECTS_DVLVELOCITY_H

#include <Eigen/Dense>

namespace subjugator {
	struct DVLVelocity {
		Eigen::Vector3d vel;
	};
}

#endif

