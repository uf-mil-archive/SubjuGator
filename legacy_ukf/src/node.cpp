#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <depth_driver/Float64Stamped.h>

#include "NavigationComputer.h"
#include "Quaternion.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace depth_driver;

Eigen::Vector3d get_Vector3(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(my_list.size() == 3);

    Eigen::Vector3d res;
    for (uint32_t i = 0; i < 3; i++) {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        res[i] = static_cast<double>(my_list[i]);
    }
    return res;
}


struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    tf::TransformListener tf_listener;
    
    string body_frame;
    string fixed_frame;
    
    message_filters::Subscriber<Imu> imu_sub;
    message_filters::Subscriber<Vector3Stamped> mag_sub;
    TimeSynchronizer<Imu, Vector3Stamped> sync;
    message_filters::Subscriber<Float64Stamped> depth_sub;
    message_filters::Subscriber<Vector3Stamped> dvl_sub;
    ros::Timer timer;
    
    ros::Publisher odometry_pub;
    tf::TransformBroadcaster tf_broadcaster;
    
    boost::scoped_ptr<subjugator::NavigationComputer> navComputer;
    
    Node() :
        private_nh("~"),
        imu_sub(nh, "imu/data_raw", 1),
        mag_sub(nh, "imu/mag", 1),
        sync(imu_sub, mag_sub, 10),
        depth_sub(nh, "depth", 1),
        dvl_sub(nh, "dvl", 1) {
        
        ROS_ASSERT(private_nh.getParam("fixed_frame", fixed_frame));
        ROS_ASSERT(private_nh.getParam("body_frame", body_frame));
        
        subjugator::NavigationComputer::Config navconf;
        navconf.referenceNorthVector = get_Vector3(private_nh, "referenceNorthVector");
        ROS_ASSERT(private_nh.getParam("latitudeDeg", navconf.latitudeDeg));
        navconf.dvl_sigma = get_Vector3(private_nh, "dvl_sigma");
        navconf.att_sigma = get_Vector3(private_nh, "att_sigma");
        
        navComputer.reset(new subjugator::NavigationComputer(navconf));
        
        odometry_pub = nh.advertise<Odometry>("odom", 1);
        
        sync.registerCallback(boost::bind(&Node::imu_callback, this, _1, _2));
        depth_sub.registerCallback(boost::bind(&Node::depth_callback, this, _1));
        dvl_sub.registerCallback(boost::bind(&Node::dvl_callback, this, _1));
    }
    
    void imu_callback(const ImuConstPtr& imu, const Vector3StampedConstPtr& mag) {
        assert(imu->header.stamp == mag->header.stamp);
        
        // XXX use frame_id
        
        subjugator::IMUInfo imuinfo;
        imuinfo.timestamp = imu->header.stamp;
        imuinfo.acceleration[0] = imu->linear_acceleration.x;
        imuinfo.acceleration[1] = imu->linear_acceleration.y;
        imuinfo.acceleration[2] = imu->linear_acceleration.z;
        imuinfo.ang_rate[0] = imu->angular_velocity.x;
        imuinfo.ang_rate[1] = imu->angular_velocity.y;
        imuinfo.ang_rate[2] = imu->angular_velocity.z;
        imuinfo.mag_field[0] = mag->vector.x;
        imuinfo.mag_field[1] = mag->vector.y;
        imuinfo.mag_field[2] = mag->vector.z;
        
        navComputer->UpdateIMU(imuinfo);
        
        publish();
    }
    
    void depth_callback(const Float64StampedConstPtr& depth) {
        subjugator::DepthInfo depthinfo;
        depthinfo.depth = depth->data;
        
        navComputer->UpdateDepth(depthinfo);
    }
    
    void dvl_callback(const Vector3StampedConstPtr& dvl) {
        subjugator::DVLVelocity dvlvelocity;
        dvlvelocity.vel[0] = dvl->vector.x;
        dvlvelocity.vel[1] = dvl->vector.y;
        dvlvelocity.vel[2] = dvl->vector.z;
        
        navComputer->UpdateDVL(dvlvelocity);
    }
    
    void publish() {
        if(!navComputer->getInitialized())
            return;
        
        subjugator::LPOSVSSInfo info;
        navComputer->GetNavInfo(info);
        
        // Emit the LPOSInfo every iteration
        Odometry msg;
        msg.header.stamp = info.timestamp;
        msg.header.frame_id = fixed_frame;
        msg.child_frame_id = body_frame;
        
        Eigen::Vector4d ENU_from_NED = subjugator::MILQuaternionOps::QuatNormalize(Eigen::Vector4d(0, 1, 1, 0));
        Eigen::Vector4d FLU_from_FRD = subjugator::MILQuaternionOps::QuatNormalize(Eigen::Vector4d(0, 1, 0, 0));
        
        Eigen::Vector3d position_ENU = subjugator::MILQuaternionOps::QuatRotate(ENU_from_NED, info.position_NED);
        msg.pose.pose.position.x = position_ENU[0];
        msg.pose.pose.position.y = position_ENU[1];
        msg.pose.pose.position.z = position_ENU[2];
        Eigen::Vector4d orientation_ENU = subjugator::MILQuaternionOps::QuatMultiply(subjugator::MILQuaternionOps::QuatMultiply(ENU_from_NED, info.quaternion_NED_B), subjugator::MILQuaternionOps::QuatConjugate(FLU_from_FRD));
        msg.pose.pose.orientation.w = orientation_ENU[0];
        msg.pose.pose.orientation.x = orientation_ENU[1];
        msg.pose.pose.orientation.y = orientation_ENU[2];
        msg.pose.pose.orientation.z = orientation_ENU[3];
        
        Eigen::Vector3d velocity_BODY = subjugator::MILQuaternionOps::QuatRotate(
            subjugator::MILQuaternionOps::QuatConjugate(info.quaternion_NED_B),
            info.velocity_NED);
        msg.twist.twist.linear.x = velocity_BODY[0];
        msg.twist.twist.linear.y = -velocity_BODY[1];
        msg.twist.twist.linear.z = -velocity_BODY[2];
        msg.twist.twist.angular.x = info.angularRate_BODY[0];
        msg.twist.twist.angular.y = -info.angularRate_BODY[1];
        msg.twist.twist.angular.z = -info.angularRate_BODY[2];
        
        odometry_pub.publish(msg);
        
        
        tf::Transform transform; poseMsgToTF(msg.pose.pose, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, msg.header.stamp, msg.header.frame_id, msg.child_frame_id));
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "legacy_ukf");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
