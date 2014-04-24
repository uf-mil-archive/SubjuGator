#include <iostream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <arm_bootloader/arm_bootloader.h>

#include <sub8_motor_driver/protocol.h>

extern unsigned char firmware_bin[];
extern int firmware_bin_len;

using namespace sub8_motor_driver;

double raw_to_voltage(double raw) {
  return raw/4095.*3.3;
}

double hv(double v) {
  return v*12994/499. - 33/2.;
}
double temp(double v) {
  return (v - 400e-3)/19.5e-3;
}
double cur(double voltage) {
  return (voltage - 3.3/2)/45e-3;
}

int main(int argc, char **argv) {
  Dest dest = 0x3ca93415;
  std::string port = "/dev/ttyUSB0";

  boost::asio::io_service io;

  boost::asio::serial_port sp(io);
  sp.open(port);
  sp.set_option(boost::asio::serial_port::baud_rate(115200));

  if(argc <= 1) {
    if(!arm_bootloader::attempt_bootload(port, sp, dest, firmware_bin, firmware_bin_len)) {
      std::cout << "bootloading failed" << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 65535);
  
  arm_bootloader::Reader<Response> reader(sp, 2000);
  
  arm_bootloader::SerialPortSink sps(port, sp);
  uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink>
    packetizer(sps);
  uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >
    checksumadder(packetizer);
  
  
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
  
  double duty = 1;
  while(true) {
    {
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::GetStatus;
      write_object(cmd, checksumadder);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "timeout receiving packet!" << std::endl;
        continue;
      }
      double dt = (boost::posix_time::microsec_clock::universal_time() - start).total_nanoseconds() * 1e-9;
      std::cout << "dt_ns: " << (resp->resp.GetStatus.uptime_ns - dt*1e9) << std::endl;
    }
    
    {
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::SetMotorDutyCycle;
      double dt = (boost::posix_time::microsec_clock::universal_time() - start).total_nanoseconds() * 1e-9;
      //double duty = sin(2*3.14159*dt*10);
      //duty *= -1;
      //duty = fabs(fmod(dt, 4) - 2) - 1;
      duty = 0.1;
      std::cout << duty << std::endl;
      cmd.args.SetMotorDutyCycle.duty_cycle = duty;
      write_object(cmd, checksumadder);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "timeout receiving packet!" << std::endl;
        continue;
      }
      std::cout << "set duty" << std::endl;
    }
    
    {
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::GetTemperatures;
      write_object(cmd, checksumadder);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "timeout receiving packet!" << std::endl;
        continue;
      }
      
      std::cout << "m+ low: " << temp(raw_to_voltage(resp->resp.GetTemperatures.voltage[0])) << " deg C" << std::endl;
      std::cout << "m+ high: " << temp(raw_to_voltage(resp->resp.GetTemperatures.voltage[6])) << " deg C" << std::endl;
      std::cout << "m- low: " << temp(raw_to_voltage(resp->resp.GetTemperatures.voltage[9])) << " deg C" << std::endl;
      std::cout << "m- high: " << temp(raw_to_voltage(resp->resp.GetTemperatures.voltage[5])) << " deg C" << std::endl;
      std::cout << "vin: " << hv(raw_to_voltage(resp->resp.GetTemperatures.voltage[4])) << " V" << std::endl;
      std::cout << "m+: " << hv(raw_to_voltage(resp->resp.GetTemperatures.voltage[2])) << " V" << std::endl;
      std::cout << "m-: " << hv(raw_to_voltage(resp->resp.GetTemperatures.voltage[3])) << " V" << std::endl;
      std::cout << "current (into m+ of motor): " << cur(raw_to_voltage(resp->resp.GetTemperatures.voltage[1])) << " A" << std::endl;
      std::cout << std::endl;
    }
    
    if(1) {
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::GetCurrentLog;
      write_object(cmd, checksumadder);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "timeout receiving packet!" << std::endl;
        continue;
      }
      {
        std::ofstream log("/tmp/voltage.csv");
        for(unsigned int i = 0; i < sizeof(resp->resp.GetCurrentLog.voltage)/
            sizeof(resp->resp.GetCurrentLog.voltage[0]); i++) {
          log << std::setprecision(9) << 1e-9*resp->resp.GetCurrentLog.time[i] << "," <<
            hv(raw_to_voltage(resp->resp.GetCurrentLog.voltage[i])) << std::endl;
        }
      }
      {
        std::ofstream log("/tmp/current.csv");
        for(unsigned int i = 0; i < sizeof(resp->resp.GetCurrentLog.current)/
            sizeof(resp->resp.GetCurrentLog.current[0]); i++) {
          log << std::setprecision(9) << 1e-9*resp->resp.GetCurrentLog.time[i] << "," <<
            cur(raw_to_voltage(resp->resp.GetCurrentLog.current[i])) << std::endl;
        }
      }
    }
  }

  return EXIT_SUCCESS;
}
