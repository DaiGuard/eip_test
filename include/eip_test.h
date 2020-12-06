#ifndef __EIP_TEST_H__
#define __EIP_TEST_H__


#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"


using eip::Session;
using eip::socket::Socket;


class EIPTest : public Session
{
  public:

    EIPTest(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket,
            EIP_UINT vendor_id = DEFAULT_VENDOR_ID, EIP_UDINT serial_num = DEFAULT_SERIAL_NUM) 
      : Session(socket, io_socket, vendor_id, serial_num)
    {}
};


#endif