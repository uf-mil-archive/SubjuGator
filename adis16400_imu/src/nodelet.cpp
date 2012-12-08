#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "adis16400_imu/driver.h"


namespace adis16400_imu {
    
    class Nodelet : public nodelet::Nodelet {
        public:
            Nodelet() {}
            ~Nodelet() {
                running = false;
                polling_thread_inst.join();
            }
            
            virtual void onInit() {
                ros::NodeHandle& private_nh = getPrivateNodeHandle();
                std::string port = "/dev/adis1640x"; private_nh.getParam("port", port);
                frame_id = "/adis16400"; private_nh.getParam("frame_id", frame_id);
                
                ros::NodeHandle& nh = getNodeHandle();
                pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
                mag_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag_raw", 10);
                
                running = true;
                device = boost::make_shared<Device>(port);
                polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
            }
            
        private:
            void polling_thread() {
                while(running) {
                    std::pair<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> res = device->read(frame_id);
                    pub.publish(res.first);
                    mag_pub.publish(res.second);
                }
            }
            
            boost::shared_ptr<Device> device;
            std::string frame_id;
            ros::Publisher pub;
            ros::Publisher mag_pub;
            bool running;
            boost::thread polling_thread_inst;
    };
    
    PLUGINLIB_DECLARE_CLASS(adis16400_imu, nodelet, adis16400_imu::Nodelet, nodelet::Nodelet);
}
