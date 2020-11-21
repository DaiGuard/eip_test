#include <string>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"
#include "odva_ethernetip/serialization/serializable_buffer.h"
#include "odva_ethernetip/io_scanner.h"

#include "eip_test.h"
#include "eip_data.h"
// #include "eip_scanner.h"


using boost::shared_ptr;
using boost::make_shared;
using boost::asio::buffer;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;
using eip::serialization::Serializable;
using eip::serialization::SerializableBuffer;
using eip::IOScanner;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "eip_test");
  ros::NodeHandle nh;

  double frequency;
  string local_ip, host;
  int port;

  ros::param::param<std::string>("~host", host, "192.168.1.1");
  ros::param::param<int>("~port", port, 2222);
  ros::param::param<double>("~frequency", frequency, 12.856);
  ros::param::param<std::string>("~local_ip", local_ip, "0.0.0.0");

  // ros::Rate loop_rate(frequency);
  ros::Rate loop_rate(1);

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, port, local_ip));

  EIPTest eip(socket, io_socket);


  while(ros::ok())
  {
    try{
      eip.open(host);      
    }
    catch(std::runtime_error ex)
    {
      ROS_ERROR("Exception can not open session: %s", ex.what());
      continue;
    }

    shared_ptr<EIPData> send_data = shared_ptr<EIPData>(new EIPData());
    EIPData recv_data;  
    send_data->data.resize(32 / sizeof(EIP_UINT));

    while(ros::ok())
    {
      try
      {
        printf("o: ");
        for(auto it=send_data->data.begin(); it!=send_data->data.end(); ++it)
        {
          *it += 1;
          printf("%d, ", *it);
        }
        printf("\n");

        // Output Assembly 150(0x96)
        eip.setSingleAttributeSerializable(0x04, 150, 3, send_data);
      }
      catch(std::runtime_error ex)
      {
        ROS_ERROR("Exception can not set: %s", ex.what());
        break;
      }

      try
      {
        // Input Assembly 100(0x64)
        eip.getSingleAttributeSerializable(0x04, 100, 3, recv_data);      
        
        printf("i: ");
        for(auto it=recv_data.data.begin(); it!=recv_data.data.end(); ++it)
        {
          printf("%d, ", *it);
        }
        printf("\n");
      }
      catch(std::runtime_error ex)
      {
        ROS_ERROR("Exception can not get: %s", ex.what());
        break;
      }

      loop_rate.sleep();
    }

    eip.close();
  }
}
