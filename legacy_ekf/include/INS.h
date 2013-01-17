#ifndef INS_H
#define INS_H

#include <Eigen/Dense>

#include "DataObjects/IMUInfo.h"
#include "Kalman.h"


namespace subjugator {
    struct INSData {
    public:
        INSData();

    INSData(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector4d q, Eigen::Vector3d g_body, Eigen::Vector3d a_body, Eigen::Vector3d a_body_raw, Eigen::Vector3d w_body,
            Eigen::Vector3d a_bias, Eigen::Vector3d w_bias, boost::uint64_t time) :
        Position_NED(p), Velocity_NED(v), Quaternion(q), Gravity_BODY(g_body),
            Acceleration_BODY(a_body), Acceleration_BODY_RAW(a_body_raw),
            AngularRate_BODY(w_body), AccelerationBias(a_bias),
            AngularRateBias(w_bias), time(time) { }

        Eigen::Vector3d Position_NED;
        Eigen::Vector3d Velocity_NED;
        Eigen::Vector4d Quaternion;
        Eigen::Vector3d Gravity_BODY;
        Eigen::Vector3d Acceleration_BODY;
        Eigen::Vector3d Acceleration_BODY_RAW;
        Eigen::Vector3d AngularRate_BODY;
        Eigen::Vector3d AccelerationBias;
        Eigen::Vector3d AngularRateBias;
        boost::uint64_t time;
    };

    class INS {
    public:
        INS(double lat, Eigen::Vector3d w_dif_prev, Eigen::Vector3d a_body_prev, Eigen::Vector3d p_prev,
            Eigen::Vector3d v_prev, Eigen::Vector3d g, Eigen::Vector4d q_prev, Eigen::Vector3d w_bias, Eigen::Vector3d a_bias,
            Eigen::Vector4d q_SUB_IMU, boost::uint64_t imuTime);

        void Update(const IMUInfo& info);
        void Reset(const KalmanData& kData);
        boost::shared_ptr<INSData> GetData() {
            return boost::shared_ptr<INSData>(prevData);
        }

    private:
        static const double SECPERNANOSEC = 1e-9;
        static const double r_earth = 6378137;    // Radius of the earth (m)
        static const double w_ie_e = 7.292115e-5;  // Angular rate of the earth (rad/s
        static const double MAX_ACC_MAG = 20.0;    // m/s^2
        static const double MAX_ANG_RATE = 12; // rad/s

           double lat;
        Eigen::Vector3d w_dif_prev;
        Eigen::Vector3d a_body_prev;
        Eigen::Vector3d p_prev;
        Eigen::Vector3d v_prev;
        Eigen::Vector3d g;
        Eigen::Vector4d q_prev;

        Eigen::Vector3d w_bias;
        Eigen::Vector3d a_bias;
        Eigen::Vector4d q_SUB_IMU;
        boost::shared_ptr<INSData> prevData;

        double dt;
        boost::int64_t imuPreviousTime;
        Eigen::Vector3d w_ie_n;
        Eigen::Vector3d w_en_n;

        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector4d q;
    };
}

#endif /* INS_H */
