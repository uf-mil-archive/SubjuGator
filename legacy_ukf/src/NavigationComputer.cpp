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
    white_noise_sigma_f(0.0005,0.0005,0.0005), white_noise_sigma_w(0.05,0.05,0.05),
    q_SUB_DVL(0.0,0.923879532511287,0.382683432365090,0.0), q_SUB_IMU(0.012621022547474,0.002181321593961,-0.004522523520991,0.999907744947984)
{
    referenceGravityVector = AttitudeHelpers::LocalGravity(conf.latitudeDeg*boost::math::constants::pi<double>()/180.0, 0);

    r_ORIGIN_NAV << 0.43115992,0.0,-0.00165058;
    
    reset();
}

void NavigationComputer::reset() {
    depthRefAvailable = false;
    attRefAvailable = false;
    velRefAvailable = false;
    
    attCount = 0;
    
    initialized = false;
}

void NavigationComputer::TryInit(ros::Time currentTime, Vector3d w_body, Vector3d a_body)
{
    assert(!initialized);

    if(!depthRefAvailable || !attRefAvailable) // || !velRefAvailable)
        return;

    //INS initialization
    ins = std::auto_ptr<INS>(
            new INS(
                    conf.latitudeDeg*boost::math::constants::pi<double>()/180.0,
                    w_body,
                    a_body,    // a_body prev MUST be taken from a valid IMU packet!
                    Vector3d(0, 0, depthRef) + MILQuaternionOps::QuatRotate(attRef, r_ORIGIN_NAV), // initialPosition
                    Vector3d(0, 0, 0), // initialVelocity
                    referenceGravityVector,    // Gravity vector from file or equation
                    attRef,    //
                    Vector3d::Zero(),    // Initial gyro bias, rad/s
                    Vector3d::Zero(),    // Initial accelerometer bias, m/s^2
                    currentTime
            ));


    // Kalman Initialization
    // Now that we have an initial attitude estimate, initialize the error terms for the kalman
    z = Vector7d::Zero();

    Eigen::Matrix<double, 13, 13> covariance;
    covariance = .01*Matrix<double, 13, 13>::Identity();
    covariance(0,0) *= .01;
    covariance.block<3,3>(2,2) = 10*covariance.block<3,3>(2,2);

    kFilter = std::auto_ptr<KalmanFilter>(
            new KalmanFilter(
                    13,
                    attRef,
                    covariance,
                    alpha, beta, kappa, bias_var_f, bias_var_w,
                    white_noise_sigma_f, white_noise_sigma_w, T_f,
                    T_w, depth_sigma, conf.dvl_sigma, conf.att_sigma,
                    currentTime
            ));

    nextKalmanTime = ros::Time(0);
    kalmanCount = 0;

    initialized = true;
}

void NavigationComputer::updateKalmanTo(ros::Time time)
{
    assert(initialized);
    
    if(time >= nextKalmanTime) {
        nextKalmanTime = time + ros::Duration(1./50); // 50 Hz
    } else {
        return;
    }

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

    kFilter->Update(z, -1*insdata->Acceleration_BODY_RAW, insdata->Velocity_NED, insdata->Quaternion, time);

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

    info.timestamp = insdata->time;

    // Do angular values first
    info.quaternion_NED_B = MILQuaternionOps::QuatMultiply(insdata->Quaternion, kdata->ErrorQuaternion);
    info.angularRate_BODY = insdata->AngularRate_BODY - kdata->Gyro_bias;

    // Transform position and velocity to the sub origin. Assuming rigid body motion
    Vector3d r_O_N_NED = MILQuaternionOps::QuatRotate(info.quaternion_NED_B, r_ORIGIN_NAV);
    info.position_NED = (insdata->Position_NED - (kdata->PositionErrorEst + (insdata->time - kdata->time).toSec() * kdata->VelocityError)) - r_O_N_NED;
    info.velocity_NED = (insdata->Velocity_NED - kdata->VelocityError) - info.angularRate_BODY.cross(r_O_N_NED);
    info.acceleration_BODY = insdata->Acceleration_BODY - kdata->Acceleration_bias +
            MILQuaternionOps::QuatRotate(MILQuaternionOps::QuatInverse(info.quaternion_NED_B), referenceGravityVector);

    //cout << "INS V\n" << insdata->Velocity_NED << endl;
    //cout<<"RPY:" << endl;
    //cout << MILQuaternionOps::Quat2Euler(info.quaternion_NED_B)*180.0/boost::math::constants::pi<double>() << endl;
}

void NavigationComputer::UpdateIMU(const IMUInfo& imu)
{
    Vector3d w_body = MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.ang_rate);
    Vector3d a_body = MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.acceleration);
    Vector3d m_body = MILQuaternionOps::QuatRotate(q_SUB_IMU, imu.mag_field);
    
    if(initialized && (imu.timestamp < ins->GetData()->time || imu.timestamp > ins->GetData()->time + ros::Duration(0.1))) // reinitialize if time goes backwards or forwards too far
        reset();
    
    // The INS has the rotation info already, so just push the packet through
    if(initialized) {
        ins->Update(imu.timestamp, w_body, a_body);
    }
    
    if(attCount == 0) {
        // Reset the sums
        magSum = Vector3d::Zero();
        accSum = Vector3d::Zero();
    }

    // We just do a very basic average over the last 10 samples (reduces to 20Hz)
    // the magnetometer and accelerometer
    magSum += m_body;
    accSum += a_body;

    attCount = (attCount + 1) % 10;
    if(attCount == 0) {
        // Now we play some games to get a gravitational estimate. We can't feed in the
        // gravity best estimate, because then you get circular dependencies between
        // the filter and the reference sensor, and the filter drifts badly. So we make the assumption
        // that accelerations are short lived. To ensure this, we check to make sure the gravitational
        // average is close to the magnitude of normal gravity. If it isn't we ignore it, and reference
        // attitude updates come in slower.

        Vector3d bodyg = -accSum / 10;    // The INS data gives -ve gravity. This is so we get the proper direction of gravity
        Vector3d bodym = magSum / 10;

        if(bodyg.norm() <= referenceGravityVector.norm() * 1.04) {
            // Bad acceleration data would just hurt the filter, eliminate it

            attRef = triad(referenceGravityVector, conf.referenceNorthVector, bodyg, bodym);
            attRefAvailable = true;

            if(!initialized)
                TryInit(imu.timestamp, w_body, a_body);
        }
    }
    
    if(initialized)
        updateKalmanTo(imu.timestamp);
}

void NavigationComputer::UpdateDepth(double depth)
{
    // The depth inside the packet is given in NED
    
    if(std::abs(depth) > MAX_DEPTH)
        return;

    depthRef = depth;
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
