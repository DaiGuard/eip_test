#include <string>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <console_bridge/console.h>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"
#include "odva_ethernetip/serialization/serializable_buffer.h"
#include "odva_ethernetip/io_scanner.h"
#include "odva_ethernetip/cpf_packet.h"
#include "odva_ethernetip/cpf_item.h"
#include "odva_ethernetip/sequenced_address_item.h"
#include "odva_ethernetip/sequenced_data_item.h"

#include "eip_test.h"
#include "eip_data.h"
#include "eip_writer.h"
#include "eip_reader.h"
// #include "eip_scanner.h"


using boost::shared_ptr;
using boost::make_shared;
using boost::asio::buffer;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;
using eip::serialization::Serializable;
using eip::serialization::SerializableBuffer;
using eip::serialization::Reader;
using eip::serialization::Writer;
using eip::IOScanner;
using eip::CPFPacket;
using eip::CPFItem;
using eip::SequencedAddressItem;


int main(int argc, char** argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  ros::init(argc, argv, "eip_test");
  ros::NodeHandle nh;

  double frequency;
  string local_ip, host;
  int port;

  ros::param::param<std::string>("~host", host, "192.168.1.1");
  ros::param::param<int>("~port", port, 2222);
  ros::param::param<double>("~frequency", frequency, 12.856);
  ros::param::param<std::string>("~local_ip", local_ip, "0.0.0.0");

  ros::Rate loop_rate(frequency);  

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, port, local_ip));

  EIP_CONNECTION_INFO_T o_to_t;
  EIP_CONNECTION_INFO_T t_to_o;

  EIP_UDINT seq_num = 1;

  shared_ptr<EIPData> send_data = shared_ptr<EIPData>(new EIPData());
  EIPData recv_data;  
  send_data->data.resize(32 / sizeof(EIP_UINT));
  
  CPFPacket send_pkt, recv_pkt;
  int connection_num = 0;

  o_to_t.assembly_id = 150;
  o_to_t.buffer_size = 38;
  // o_to_t.rpi = 10000000;  
  o_to_t.rpi = 1000000;

  t_to_o.assembly_id = 100;
  t_to_o.buffer_size = 34;
  // t_to_o.rpi = 10000000;
  t_to_o.rpi = 1000000;

  EIPTest eip(socket, io_socket);

  while(ros::ok())
  {
    try{
      eip.open(host);      
    }
    catch(std::runtime_error ex)
    {
      ROS_ERROR("Exception can not open session: %s", ex.what());
      break;
    }

    try{
      connection_num = eip.createConnection(o_to_t, t_to_o);
    }
    catch(std::runtime_error ex)
    {
      ROS_ERROR("Exception can not connection: %s", ex.what());
      break;
    }

    while(ros::ok())
    {
      ROS_INFO("LOOP");
            
      try
      {
        // printf("o: ");
        // for(auto it=send_data->data.begin(); it!=send_data->data.end(); ++it)
        // {
        //   *it += 1;
        //   printf("%d, ", *it);
        // }
        // printf("\n");

        //   // Output Assembly 150(0x96)
        //   eip.setSingleAttributeSerializable(0x04, 150, 3, send_data);        

        // shared_ptr<SequencedAddressItem> address = 
        //   make_shared<SequencedAddressItem>(connection_num, seq_num++);
        shared_ptr<SequencedAddressItem> address_o_to_t = 
          make_shared<SequencedAddressItem>(eip.getConnection(connection_num).o_to_t_connection_id, seq_num++);
        shared_ptr<EIPWriter> data = make_shared<EIPWriter>();
        printf("i: ");
        data->data[0] = seq_num & 0x00ff;
        data->data[1] = (seq_num & 0xff00) >> 8;
        data->data[2] = 1;
        data->data[3] = 0;
        data->data[4] = 0;
        data->data[5] = 0;
        for(int i=6; i<sizeof(data->data); i++)
        {
          data->data[i] = seq_num + i;
          printf("%02x, ", data->data[i]);
        }
        printf("\n");

        send_pkt.getItems().clear();
        // send_pkt.getItems().push_back(CPFItem(0x8002, address));
        send_pkt.getItems().push_back(CPFItem(0x8002, address_o_to_t));
        send_pkt.getItems().push_back(CPFItem(0x00B1, data));
        // send_pkt.getItems().push_back(CPFItem(0x8002, address_t_to_o));                

        eip.sendIOPacket(send_pkt);
        recv_pkt = eip.receiveIOPacket();

        SequencedAddressItem address_t_to_o;
        EIPReader reader;

        recv_pkt.getItems()[0].getDataAs(address_t_to_o);
        recv_pkt.getItems()[1].getDataAs(reader);

        printf("o: \n");
        printf("\taddress: %d %d\n\t", address_t_to_o.connection_id, address_t_to_o.sequence_num);
        for(int i=0; i<reader.getLength(); i++)
        {
          printf("%02x, ", reader.data[i]);
        }
        printf("\n");
      }
      catch(std::runtime_error ex)
      {
        ROS_ERROR("Exception can not set: %s", ex.what());
        break;
      }

      try
      {
        // Input Assembly 100(0x64)
        // eip.getSingleAttributeSerializable(0x04, 100, 3, recv_data);

        // shared_ptr<SequencedAddressItem> address = 
        //   make_shared<SequencedAddressItem>(eip.getConnection(0).t_to_o_connection_id, seq_num);

        // shared_ptr<EIPWriter> data = make_shared<EIPWriter>();

        // send_pkt.getItems().clear();
        // send_pkt.getItems().push_back(CPFItem(0x8001, address));
        // send_pkt.getItems().push_back(CPFItem(0x00B1, data));

        // eip.sendIOPacket(send_pkt);
        // recv_pkt = eip.receiveIOPacket();

        // printf("o: ");
        // for(auto it=recv_pkt.getItems().begin(); it!=recv_pkt.getItems().end(); ++it)
        // {
        //   printf("(%d, %d), ", it->getItemType(), it->getLength());
        // }
        // printf("\n");
        
        // printf("i: ");
        // for(auto it=recv_data.data.begin(); it!=recv_data.data.end(); ++it)
        // {
        //   printf("%d, ", *it);
        // }
        // printf("\n");
      }
      catch(std::runtime_error ex)
      {
        ROS_ERROR("Exception can not get: %s", ex.what());
        break;
      }

      loop_rate.sleep();
    }

    eip.closeConnection(0);
    eip.close();
  }
}
