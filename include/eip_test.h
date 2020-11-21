#ifndef __EIP_TEST_H__
#define __EIP_TEST_H__


#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"


using eip::Session;
using eip::socket::Socket;


class EIPTest : public Session
{
  public:

    EIPTest(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket) 
      : Session(socket, io_socket)
    {}
};


#endif