#ifndef DRIVER_H
#define DRIVER_H

#include <cmath>
#include <fstream>

#include <boost/asio/serial_port.hpp>
#include <boost/crc.hpp>

#include "depth_driver/Float64Stamped.h"


namespace depth_driver {
    static uint16_t getu16le(uint8_t* i) { return *i + *(i+1) * 256; }
    
    static const uint8_t flagbyte = 0x7E, escapebyte = 0x7D, maskbyte = 0x20;
    
    class Device {
        private:
            typedef std::vector<boost::uint8_t> ByteVec;
            typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRCCalculator;
            
            const std::string port;
            const int baudrate;
            boost::asio::io_service io;
            boost::asio::serial_port p;
            
            void send_packet(ByteVec unescaped) {
                uint8_t header[] = {21, 40, 0, 0}; unescaped.insert(unescaped.begin(), header, header + sizeof(header)/sizeof(header[0]));
                
                CRCCalculator crc; crc.process_block(&*unescaped.begin(), &*unescaped.end());
                unescaped.push_back(crc.checksum() & 0xFF);
                unescaped.push_back(crc.checksum() >> 8);
                
                ByteVec out; out.reserve(unescaped.size() + 10); // checksum + escapes will create a few extra bytes
                
                out.push_back(flagbyte);
                
                for (ByteVec::const_iterator i = unescaped.begin(); i != unescaped.end(); ++i) { // add escapes
                    if (*i == flagbyte || *i == escapebyte) {
                        out.push_back(escapebyte);
                        out.push_back(*i ^ maskbyte);
                    } else {
                        out.push_back(*i);
                    }
                }
                
                out.push_back(flagbyte);
                
                try {
                    p.write_some(boost::asio::buffer(out.data(), out.size()));
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on write: %s; dropping", exc.what());
                }
            }
            
            uint8_t read_byte() {
                while(true) {
                    try {
                        uint8_t res; p.read_some(boost::asio::buffer(&res, sizeof(res)));
                        return res;
                    } catch(const std::exception &exc) {
                        ROS_ERROR("error on read: %s; reopening", exc.what());
                        open();
                    }
                }
            }
            
            void open() {
                while(true) {
                    try {
                        p.close();
                        p.open(port);
                        p.set_option(boost::asio::serial_port::baud_rate(baudrate));
                        return;
                    } catch(const std::exception &exc) {
                        ROS_ERROR("error on open(%s): %s; reopening after delay", port.c_str(), exc.what());
                        boost::this_thread::sleep(boost::posix_time::seconds(1));
                    }
                }
            }
            
        public:
            Device(const std::string port, int baudrate) : port(port), baudrate(baudrate), p(io) {
                // open is called on first read_byte() in the _polling_ thread
            }
            
            double read() {
                while(true) {
                    while(read_byte() != flagbyte)
                        ;
                    
                    ByteVec unescaped;
                    while(true) {
                        uint8_t b = read_byte();
                        if(b == flagbyte && unescaped.size() != 0)
                            break;
                        else if(b == flagbyte && unescaped.size() == 0)
                            continue; // would be zero length message, so we're out of sync and ignoring it gets us back
                        else if(b == escapebyte)
                            unescaped.push_back(read_byte() ^ maskbyte);
                        else
                            unescaped.push_back(b);
                    }
                    
                    if(unescaped.size() < 2) { // message too short for checksum
                        ROS_INFO("packet too small");
                        continue;
                    }
                    CRCCalculator crc; crc.process_block(&*unescaped.begin(), &*(unescaped.end() - 2));
                    if(*(unescaped.end() - 2) != (crc.checksum() & 0xFF) || *(unescaped.end() - 1) != (crc.checksum() >> 8)) {
                        ROS_INFO("packet with invalid checksum");
                        continue;
                    }
                    unescaped.erase(unescaped.end() - 2, unescaped.end());
                    
                    double temp = getu16le(&*unescaped.end() - 8) / 1024.;
                    return (temp - 10.62)*1.45;
                }
            }
            
            void send_heartbeat() {
                send_packet(ByteVec()); // heartbeat
                uint8_t msg[] = {4, 1, 20}; send_packet(ByteVec(msg, msg + sizeof(msg)/sizeof(msg[0]))); // StartPublishing 20hz
            }
    };
    
}

#endif
