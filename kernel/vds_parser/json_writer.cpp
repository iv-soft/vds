/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "json_writer.h"

vds::json_writer::json_writer()
  : state_(BOF)
{
}

std::string vds::json_writer::str() const
{
  return this->stream_.str();
}


vds::expected<void> vds::json_writer::write_string_value(const std::string & value)
{
  switch (this->state_)
  {
  case PROPERTY:
    this->state_ = PROPERTY_END;
    break;
    
  case START_ARRAY:
    this->state_ = ARRAY_BODY;
    break;

  case ARRAY_BODY:
    this->stream_ << ',';
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  return this->write_string(value);
}

vds::expected<void> vds::json_writer::write_null_value()
{
  switch (this->state_)
  {
  case PROPERTY:
    this->state_ = PROPERTY_END;
    break;

  case ARRAY_BODY:
    this->stream_ << ',';
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->stream_ << "null";
  return expected<void>();
}

vds::expected<void> vds::json_writer::start_property(const std::string & name)
{
  switch (this->state_)
  {
  case START_OBJECT:
    this->state_ = OBJECT_BODY;
    break;
  case OBJECT_BODY:
    this->stream_ << ',';
    break;
  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_path_.push(this->state_);
  this->state_ = PROPERTY;

  CHECK_EXPECTED(this->write_string(name));
  this->stream_ << ':';
  return expected<void>();
}

vds::expected<void> vds::json_writer::end_property()
{
  switch (this->state_)
  {
  case PROPERTY_END:
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_ = this->state_path_.top();
  this->state_path_.pop();
  return expected<void>();
}

vds::expected<void> vds::json_writer::write_property(const std::string & name, const std::string & value)
{
  CHECK_EXPECTED(this->start_property(name));
  CHECK_EXPECTED(this->write_string_value(value));
  return this->end_property();
}

vds::expected<void> vds::json_writer::start_object()
{
  switch (this->state_)
  {
  case BOF:
  //case START_OBJECT:
  //case OBJECT_BODY:
  case START_ARRAY:
    this->state_path_.push(ARRAY_BODY);
    break;
    
  case ARRAY_BODY:
    this->stream_ << ',';
    this->state_path_.push(ARRAY_BODY);
    break;
      
  case PROPERTY:
    this->state_path_.push(PROPERTY_END);
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_ = START_OBJECT;

  this->stream_ << '{';
  return expected<void>();
}

vds::expected<void> vds::json_writer::end_object()
{
  switch (this->state_)
  {
  case START_OBJECT:
  case OBJECT_BODY:
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_ = this->state_path_.top();
  this->state_path_.pop();

  this->stream_ << '}';
  return expected<void>();
}

vds::expected<void> vds::json_writer::start_array()
{
  switch (this->state_)
  {
  case BOF:
  //case START_OBJECT:
  //case OBJECT_BODY:
  case PROPERTY:
    this->state_path_.push(PROPERTY_END);
    break;
  case START_ARRAY:
  case ARRAY_BODY:
    this->state_path_.push(this->state_);
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_ = START_ARRAY;

  this->stream_ << '[';
  return expected<void>();
}

vds::expected<void> vds::json_writer::end_array()
{
  switch (this->state_)
  {
  case START_ARRAY:
  case ARRAY_BODY:
    break;

  default:
    return vds::make_unexpected<std::runtime_error>("Invalid json_writer state");
  }

  this->state_ = this->state_path_.top();
  this->state_path_.pop();

  this->stream_ << ']';
  return expected<void>();
}

vds::expected<void> vds::json_writer::write_string(const std::string & value)
{
  this->stream_ << '\"';
  const char * utf8string = value.c_str();
  size_t len = value.length();

  while(0 < len) {
    GET_EXPECTED(ch, utf8::next_char(utf8string, len));
    switch (ch) {
    case '\\':
      this->stream_ << "\\\\";
      break;
    case '\"':
      this->stream_ << "\\\"";
      break;
    case '\n':
      this->stream_ << "\\n";
      break;
    case '\r':
      this->stream_ << "\\r";
      break;
    case '\b':
      this->stream_ << "\\b";
      break;
    case '\t':
      this->stream_ << "\\t";
      break;
    case '\f':
      this->stream_ << "\\f";
      break;
    default:
      if (ch < 0x80 && isprint((int)ch)) {
        this->stream_ << (char)ch;
      }
      else {
        this->stream_ << "\\u" << std::setw(4) << std::setfill('0') << std::hex << (uint16_t)ch;
      }
    }
  }

  this->stream_ << '\"';
  return expected<void>();
}
