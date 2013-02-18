#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "depth_driver/Float64Stamped.h"
#include "depth_driver/driver.h"


namespace depth_driver {
    
    class Nodelet : public nodelet::Nodelet {
        public:
            Nodelet() {}
            ~Nodelet() {
                heartbeat_timer.stop();
                running = false;
                device->abort();
                polling_thread_inst.join();
            }
            
            virtual void onInit() {
                std::string port = "/dev/ttyS1"; getPrivateNodeHandle().getParam("port", port);
                int baudrate = 115200; getPrivateNodeHandle().getParam("baudrate", baudrate);
                frame_id = "/map"; getPrivateNodeHandle().getParam("frame_id", frame_id);
                
                pub = getNodeHandle().advertise<Float64Stamped>("depth", 10);
                
                device = boost::make_shared<Device>(port, baudrate);
                heartbeat_timer = getNodeHandle().createTimer(ros::Duration(0.5), boost::bind(&Nodelet::heartbeat_callback, this, _1));
                running = true;
                polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
            }
            
        private:
            void heartbeat_callback(const ros::TimerEvent& event) {
                device->send_heartbeat();
            }
            
            void polling_thread() {
                while(running) {
                    Float64Stamped msg;
                    if(!device->read(msg.data))
                        continue;
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = frame_id;
                    pub.publish(msg);
                }
            }
            
            std::string frame_id;
            ros::Publisher pub;
            boost::shared_ptr<Device> device;
            ros::Timer heartbeat_timer;
            bool running;
            boost::thread polling_thread_inst;
    };
    
    PLUGINLIB_DECLARE_CLASS(depth_driver, nodelet, depth_driver::Nodelet, nodelet::Nodelet);
}