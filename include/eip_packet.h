#ifndef __EIP_PACKET_H__
#define __EIP_PACKET_H__


#include <string>
#include <vector>

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/cpf_packet.h"
#include "odva_ethernetip/cpf_item.h"

using std::vector;
using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;
using eip::CPFPacket;
using eip::CPFItem;


class EIPPacket : public CPFPacket
{
  public:
  vector<CPFItem> data;
};


#endif