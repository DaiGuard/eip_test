#ifndef __EIP_DATA_H__
#define __EIP_DATA_H__


#include <string>
#include <vector>

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"


using std::vector;
using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;


class EIPData : public Serializable
{
  public:
  vector<EIP_UINT> data;

  virtual size_t getLength() const
  {
    return data.size() * sizeof(EIP_UINT);
  }

  virtual Writer& serialize(Writer& writer) const
  {
    printf("writer: %d\n", data.size() * sizeof(EIP_UINT));
    writer.writeBytes(&data[0], data.size() * sizeof(EIP_UINT));
    return writer;
  }

  virtual Reader& deserialize(Reader& reader, size_t length)
  {    
    printf("reader: %d\n", length);

    data.resize(length / sizeof(EIP_UINT));

    deserialize(reader);
    return reader;
  }

  virtual Reader& deserialize(Reader& reader)
  {        
    reader.readBytes(&data[0], data.size() * sizeof(EIP_UINT));
    return reader;
  }

};


#endif