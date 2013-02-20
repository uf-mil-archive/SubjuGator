#ifndef DATAOBJECTS_IMUINFO_H
#define DATAOBJECTS_IMUINFO_H

#include <boost/cstdint.hpp>
#include <Eigen/Dense>

namespace subjugator {
    struct IMUInfo {
        boost::uint64_t timestamp;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d ang_rate;
        Eigen::Vector3d mag_field;
    };
}

#endif /* DATAOBJECTS_IMUINFO_H */

