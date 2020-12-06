#ifndef __EIP_READER_H__
#define __EIP_READER_H__


#include "odva_ethernetip/serialization/reader.h"


using eip::serialization::Reader;


class EIPReader : public Reader
{
  public:
  virtual void readBytes(void* buf, size_t n)
  {

  }

  virtual void readBuffer(mutable_buffer buf)
  {

  }

  virtual size_t getByteCount()
  {

  }

  virtual void skip(size_t n)
  {

  }
}


#endif