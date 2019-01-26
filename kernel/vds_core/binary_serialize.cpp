/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "binary_serialize.h"

vds::expected<void> vds::binary_serializer::put(bool value)
{
  this->data_.add(value ? (uint8_t)0xFF : (uint8_t)0);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(uint8_t value)
{
  this->data_.add(value);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(uint16_t value)
{
  this->data_.add((value >> 8) & 0xFF);
  this->data_.add(value & 0xFF);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(uint32_t value)
{
  this->data_.add((value >> 24) & 0xFF);
  this->data_.add((value >> 16) & 0xFF);
  this->data_.add((value >> 8) & 0xFF);
  this->data_.add(value & 0xFF);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(uint64_t value)
{
  this->data_.add((value >> 56) & 0xFF);
  this->data_.add((value >> 48) & 0xFF);
  this->data_.add((value >> 40) & 0xFF);
  this->data_.add((value >> 32) & 0xFF);
  this->data_.add((value >> 24) & 0xFF);
  this->data_.add((value >> 16) & 0xFF);
  this->data_.add((value >> 8) & 0xFF);
  this->data_.add(value & 0xFF);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::write_number(uint64_t value)
{
  // 0 .... 7 bit
  if (128 > value) {
    return this->put((uint8_t)value);
  }
  
  value -= 128;
  
  uint8_t data[8];
  int index = 0;
  do {
    data[index++] = (value & 0xFF);
    value >>= 8;
  } while (0 != value);
  
  this->data_.add((uint8_t)(0x80 | index));
  while(index > 0){
    this->data_.add(data[--index]);
  }
  
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(const std::string & value)
{
  CHECK_EXPECTED(this->write_number(value.length()));
  this->data_.add(value.c_str(), value.length());
  
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::push_data(const void * data, size_t size, bool serialize_size)
{
  if (serialize_size) {
    CHECK_EXPECTED(this->write_number(size));
  }
  
  this->data_.add(data, size);
  return expected<void>();
}

vds::expected<void> vds::binary_serializer::put(const const_data_buffer& data)
{
  return this->push_data(data.data(), data.size(), true);
}

///////////////////////////////////////////////////////////////////////////
vds::binary_deserializer::binary_deserializer(const const_data_buffer& data)
  : data_(data.data()), len_(data.size()) {
}

vds::binary_deserializer::binary_deserializer(const void* data, size_t len)
: data_(static_cast<const uint8_t *>(data)), len_(len) {
}


vds::expected<void> vds::binary_deserializer::get(bool& value) {
  if (1 > this->len_) {
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }

  value = (0 != *this->data_++);
  --this->len_;

  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::get(uint8_t & value)
{
  if(1 > this->len_){
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }
  
  value = *this->data_++;
  --this->len_;
  
  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::get(uint16_t& value)
{
  if(2 > this->len_){
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }
  
  value = *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  
  this->len_ -= 2;
  
  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::get(uint32_t& value)
{
  if(4 > this->len_){
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }
  
  value = *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  
  this->len_ -= 4;
  
  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::get(uint64_t& value)
{
  if(8 > this->len_){
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }
  
  value = *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  value <<= 8;
  value |= *this->data_++;
  
  this->len_ -= 8;
  
  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::get(std::string& value)
{
  GET_EXPECTED(len, this->read_number());

  value.resize(len);
  for(uint64_t i = 0; i < len; ++i){
    uint8_t ch;
    CHECK_EXPECTED(this->get(ch));
    value[i] = ch;
  }
  
  return expected<void>();
}


vds::expected<void> vds::binary_deserializer::get(vds::const_data_buffer & data)
{
  GET_EXPECTED(len, this->read_number());
  if(len > 1024 * 1024 * 1024){
    return vds::make_unexpected<std::runtime_error>("very big object");
  }


  if (len> this->len_) {
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }

  data.resize(len);
  memcpy(&data[0], this->data_, len);
  this->data_ += len;
  this->len_ -= len;

  return expected<void>();
}

vds::expected<void> vds::binary_deserializer::pop_data(void* data, size_t& size, bool serialize_size)
{
  if(serialize_size){
    GET_EXPECTED(len, this->read_number());
    if(size < len){
      return vds::make_unexpected<std::runtime_error>("Buffer too small");
    }
    size = len;
  }

  if (size > this->len_) {
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }

  memcpy(data, this->data_, size);
  this->data_ += size;
  this->len_ -= size;

  return expected<void>();
}

vds::expected<size_t> vds::binary_deserializer::pop_data(void* data, size_t size)
{
  CHECK_EXPECTED(this->pop_data(data, size, true));
  return size;
}


vds::expected<uint64_t> vds::binary_deserializer::read_number()
{
  uint8_t value;
  CHECK_EXPECTED(this->get(value));
  
  if(0x80 > value){
    return value;
  }
  
  uint64_t result = 0;
  for(uint8_t i = (value & 0x7F); i > 0; --i){
    CHECK_EXPECTED(this->get(value));
    result <<= 8;
    result |= value;
  }
  
  return result + 0x80;
}
