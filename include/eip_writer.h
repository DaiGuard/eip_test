#ifndef __EIP_WRITER_H__
#define __EIP_WRITER_H__


#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"

using std::string;
using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;


class EIPWriter : public Serializable
{
  public:

  EIP_BYTE data[38];

  EIPWriter()
  {
    memset(data, 0, sizeof(data));
  }

  virtual size_t getLength() const
  {
    return sizeof(data);
  }

  virtual Writer& serialize(Writer& writer) const
  {
    for(int i=0; i<sizeof(data); i++)
    {
      writer.write(data[i]);
    }

    return writer;
  }

  virtual Reader& deserialize(Reader& reader, size_t length)
  {
    return deserialize(reader);
  }

  virtual Reader& deserialize(Reader& reader)
  {
    for(int i=0; i<sizeof(data); i++)
    {
      reader.read(data[i]);
    }

    return reader;
  }
};


#endif