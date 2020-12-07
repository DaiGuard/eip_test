#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <console_bridge/console.h>
#include <sensor_msgs/JointState.h>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"
#include "odva_ethernetip/io_scanner.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using boost::system::error_code;

// using eip::socket::TCPSocket;
// using eip::socket::UDPSocket;
using eip::IOScanner;


int main(int argc, char** argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  ros::init(argc, argv, "eip_scanner");
  ros::NodeHandle nh;

  // double frequency;
  // string local_ip, host;
  // int port;

  // ros::param::param<std::string>("~host", host, "192.168.1.1");
  // ros::param::param<int>("~port", port, 2222);
  // ros::param::param<double>("~frequency", frequency, 12.856);
  // ros::param::param<std::string>("~local_ip", local_ip, "0.0.0.0");

  boost::asio::io_service io_service;
  // shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  // shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, port, local_ip));

  try
  {
    error_code ec;

    IOScanner scanner(io_service, argv[1]);
    scanner.run();

    scanner.sendListIdentityRequest();
    scanner.handleListIdentityResponse(ec, 512);    

    cout << ec.message() << endl;
  }
  catch (std::runtime_error ex)
  {
    cout << "Exception caught doing IO scan: " << ex.what() << endl;
  }
  // catch (ros::Exception ex)
  // {

  // }

  return 0;
}