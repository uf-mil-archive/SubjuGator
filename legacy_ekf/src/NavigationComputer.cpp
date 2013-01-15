#include <iostream>

#include <boost/math/constants/constants.hpp>

#include "AttitudeHelpers.h"
#include "Quaternion.h"
#include "Triad.h"

#include "NavigationComputer.h"

using namespace subjugator;
using namespace std;
using namespace Eigen;

NavigationComputer::NavigationComputer(const Config &conf):
    conf(conf),
    referenceGravityVector(0.0,0.0,1.0),
    initialPosition(0.0,0.0,0.0), initialVelocity(0.0,0.0,0.0),
    white_noise_sigma_f(0.0005,0.0005,0.0005), white_noise_sigma_w(0.05,0.05,0.05),
    q_SUB_DVL(0.0,0.923879532511287,0.382683432365090,0.0), q_SUB_IMU(0.012621022547474,0.002181321593961,-0.004522523520991,0.999907744947984),
    initialized(false)
{
    covariance = .01*Matrix<double, 13, 13>::Identity();
    covariance(0,0) *= .01;
    covariance.block<3,3>(2,2) = 10*covariance.block<3,3>(2,2);

    referenceGravityVector = AttitudeHelpers::LocalGravity(latitudeDeg*boost::math::constants::pi<double>()/180.0, initialPosition(2));

    r_ORIGIN_NAV << 0.43115992,0.0,-0.00165058;

    acceptable_gravity_mag = referenceGravityVector.norm() * 1.04;

    z = Vector7d::Zero();

    depthRefAvailable = false;
    attRefAvailable = false;
    velRefAvailable = false;
}

boost::int64_t NavigationComputer::getTimestamp(void)
{
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    static const uint64_t NSEC_PER_SEC = 1000000000;
    return ((long long int)t.tv_sec * NSEC_PER_SEC) + t.tv_nsec;
}

void NavigationComputer::TryInit(const IMUInfo& imu)
{
    assert(!initialized);

    if(!depthRefAvailable || !attRefAvailable) // || !velRefAvailable)
        return;

    initialPosition += MILQuaternionOps::QuatRotate(attRef, r_ORIGIN_NAV);

    //INS initialization
    ins = std::auto_ptr<INS>(
            new INS(
                    latitudeDeg*boost::math::constants::pi<double>()/180.0,
                    Vector3d::Zero(), // assume the sub is at rest when we start, hence omega is zero
                    MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.acceleration),    // a_body prev MUST be taken from a valid IMU packet!
                    initialPosition,
                    initialVelocity,
                    referenceGravityVector,    // Gravity vector from file or equation
                    attRef,    //
                    Vector3d::Zero(),    // Initial gyro bias, rad/s
                    Vector3d::Zero(),    // Initial accelerometer bias, m/s^2
                    q_SUB_IMU,
                    imu.timestamp
            ));


    // Kalman Initialization
    // Now that we have an initial attitude estimate, initialize the error terms for the kalman
    z = Vector7d::Zero();

    kFilter = std::auto_ptr<KalmanFilter>(
            new KalmanFilter(
                    13,
                    referenceGravityVector.norm(),
                    attRef,
                    covariance,
                    alpha, beta, kappa, bias_var_f, bias_var_w,
                    white_noise_sigma_f, white_noise_sigma_w, T_f,
                    T_w, depth_sigma, conf.dvl_sigma, conf.att_sigma,
                    getTimestamp()
            ));
    kalmanCount = 0;

    // Now build up the kalman timer.
    kTimerMs = 1000 / 50 /*Hz*/;
    kTimer = 0;

    initialized = true;
}

void NavigationComputer::updateKalman()
{
    assert(initialized);

    boost::shared_ptr<INSData> insdata = ins->GetData();

    // Constant error kalman errors
    if(attRefAvailable)
    {
        attRefAvailable = false;
        Vector4d tempQuat = MILQuaternionOps::QuatMultiply(MILQuaternionOps::QuatInverse(attRef), insdata->Quaternion);
        if (tempQuat(0) < 0)
            tempQuat *= -1;
        z.block<3,1>(4,0) = tempQuat.block<3,1>(1,0);
    }
    if(depthRefAvailable)
    {
        depthRefAvailable = false;
        z(0) = insdata->Position_NED(2) - depthRef;
    }
    if(velRefAvailable)
    {
        //velRefAvailable = false;
        z.block<3,1>(1,0) = insdata->Velocity_NED - velRef;
    }

    kFilter->Update(z, -1*insdata->Acceleration_BODY_RAW, insdata->Velocity_NED, insdata->Quaternion, getTimestamp());

    if(++kalmanCount >= 100)    // 2s reset time
    {
        kalmanCount = 0;
        resetErrors();
    }

}

void NavigationComputer::resetErrors()
{
    boost::shared_ptr<KalmanData> kdata = kFilter->GetData();

    ins->Reset(*kdata.get());
    kFilter->Reset();
    z = Vector7d::Zero();
}

void NavigationComputer::GetNavInfo(LPOSVSSInfo& info)
{
    assert(initialized);

    // Subtract errors to build best current estimate
    boost::shared_ptr<KalmanData> kdata = kFilter->GetData();
    boost::shared_ptr<INSData> insdata = ins->GetData();

    // Do angular values first
    info.quaternion_NED_B = MILQuaternionOps::QuatMultiply(insdata->Quaternion, kdata->ErrorQuaternion);
    info.angularRate_BODY = insdata->AngularRate_BODY - kdata->Gyro_bias;

    // Transform position and velocity to the sub origin. Assuming rigid body motion
    Vector3d r_O_N_NED = MILQuaternionOps::QuatRotate(info.quaternion_NED_B, r_ORIGIN_NAV);
    info.position_NED = (insdata->Position_NED - kdata->PositionErrorEst) - r_O_N_NED;
    info.velocity_NED = (insdata->Velocity_NED - kdata->VelocityError) - info.angularRate_BODY.cross(r_O_N_NED);
    info.acceleration_BODY = insdata->Acceleration_BODY - kdata->Acceleration_bias +
            MILQuaternionOps::QuatRotate(MILQuaternionOps::QuatInverse(info.quaternion_NED_B), referenceGravityVector);

    //cout << "INS V\n" << insdata->Velocity_NED << endl;
    //cout<<"RPY:" << endl;
    //cout << MILQuaternionOps::Quat2Euler(info.quaternion_NED_B)*180.0/boost::math::constants::pi<double>() << endl;
}

void NavigationComputer::UpdateIMU(const IMUInfo& imu)
{
    static int count = 0;

    // The INS has the rotation info already, so just push the packet through
    if(initialized)
        ins->Update(imu);

    // We just do a very basic average over the last 10 samples (reduces to 20Hz)
    // the magnetometer and accelerometer
    magSum += MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.mag_field);
    accSum += MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.acceleration);

    count = (count + 1) % 10;
    if(count)    // Don't have enough samples yet
        return;

    Vector3d tempMag = magSum / 10.0;
    // Now we play some games to get a gravitational estimate. We can't feed in the
    // gravity best estimate, because then you get circular dependencies between
    // the filter and the reference sensor, and the filter drifts badly. So we make the assumption
    // that accelerations are short lived. To ensure this, we check to make sure the gravitational
    // average is close to the magnitude of normal gravity. If it isn't we ignore it, and reference
    // attitude updates come in slower.

    Vector3d bodyg = -1.0*accSum / 10.0;    // The INS data gives -ve gravity. This is so we get the proper direction of gravity

    // Reset the sums
    magSum = Vector3d::Zero();
    accSum = Vector3d::Zero();

    if(bodyg.norm() > acceptable_gravity_mag)    // Bad acceleration data would just hurt the filter, eliminate it
        return;

    attRef = triad(referenceGravityVector, conf.referenceNorthVector, bodyg, tempMag);
    attRefAvailable = true;

    if(!initialized)
        TryInit(imu);
}

void NavigationComputer::UpdateDepth(const DepthInfo& depth)
{
    // The depth inside the packet is given in NED
    
    if(std::abs(depth.depth) > MAX_DEPTH)
        return;

    depthRef = depth.depth;
    depthRefAvailable = true;
}

void NavigationComputer::UpdateDVL(const DVLVelocity& dvl)
{
    // DVL data is expected in the NED down frame by the error measurement
    // for the Kalman filter. It comes in the DVL frame, which needs to be rotated
    // to the sub frame, and then transformed by the current best quaternion estimate
    // of SUB to NED
    Vector3d dvl_vel = MILQuaternionOps::QuatRotate(q_SUB_DVL, dvl.vel);

    if(!initialized)
        return;
    boost::shared_ptr<INSData> insdata = ins->GetData();
    boost::shared_ptr<KalmanData> kdata = kFilter->GetData();

    // Rotate dvl data from SUB to NED
    velRef = MILQuaternionOps::QuatRotate(MILQuaternionOps::QuatMultiply(insdata->Quaternion, kdata->ErrorQuaternion), dvl_vel);
    velRefAvailable = true;
}

void NavigationComputer::Update(boost::int64_t dtms) {
    kTimer += dtms;
    if (kTimer >= kTimerMs) {
        updateKalman();
        kTimer = 0;
    }
}
