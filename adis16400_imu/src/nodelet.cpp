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
                mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag_raw", 10);
                
                count = 0;
                running = true;
                device = boost::make_shared<Device>(port);
                polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
            }
            
        private:
            void polling_thread() {
                while(running) {
                    sensor_msgs::Imu imu;
                    sensor_msgs::MagneticField mag;
                    if(device->read(frame_id, imu, mag)) {
                        if(count++ % 4 != 0) continue;
                        pub.publish(imu);
                        mag_pub.publish(mag);
                    }
                }
            }
            
            boost::shared_ptr<Device> device;
            std::string frame_id;
            ros::Publisher pub;
            ros::Publisher mag_pub;
            bool running;
            boost::thread polling_thread_inst;
            int count;
    };
    
    PLUGINLIB_DECLARE_CLASS(adis16400_imu, nodelet, adis16400_imu::Nodelet, nodelet::Nodelet);
}
