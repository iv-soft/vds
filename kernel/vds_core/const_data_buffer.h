#ifndef __VDS_CORE_DATA_BUFFER_H_
#define __VDS_CORE_DATA_BUFFER_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "dataflow.h"

namespace vds{
  class const_data_buffer
  {
  public:
    const_data_buffer()
    : data_(nullptr), len_(0)
    {
    }
    
    const_data_buffer(const void * data, size_t len)
    : data_(new uint8_t[len]), len_(len)
    {
      memcpy(this->data_, data, len);
    }
    
    const_data_buffer(const const_data_buffer & other)
    : data_(new uint8_t[other.len_]), len_(other.len_)
    {
      memcpy(this->data_, other.data_, other.len_);
    }
    
    const_data_buffer(const_data_buffer&& other)
    : data_(other.data_), len_(other.len_)
    {
      other.data_ = nullptr;
      other.len_ = 0;
    }
    
    const_data_buffer(const std::vector<uint8_t> & data)
    : data_(new uint8_t[data.size()]), len_(data.size())
    {
      memcpy(this->data_, data.data(), data.size());
    }
    
    ~const_data_buffer()
    {
      delete this->data_;
    }
    
    const uint8_t * data() const { return this->data_; }
    size_t size() const { return this->len_; }
    
    void reset(const void * data, size_t len)
    {
      delete this->data_;
      this->data_ = new uint8_t[len];
      this->len_ = len;
      memcpy(this->data_, data, len);
    }

    const_data_buffer & operator = (const_data_buffer && other)
    {
      delete this->data_;
      this->data_ = other.data_;
      this->len_ = other.len_;
      other.data_ = nullptr;
      other.len_ = 0;
      
      return *this;
    }
    
    const_data_buffer & operator = (const const_data_buffer & other)
    {
      this->reset(other.data(), other.size());
      
      return *this;
    }    
    
    bool operator == (const const_data_buffer & other) const
    {
      return this->len_ == other.len_
      && 0 == memcmp(this->data_, other.data_, this->len_);
    }
    
    bool operator != (const const_data_buffer & other) const
    {
      return this->len_ != other.len_
      || 0 != memcmp(this->data_, other.data_, this->len_);
    }
    
    uint8_t operator[](size_t index) const
    {
      return this->data_[index];
    }
    
  private:
    uint8_t * data_;
    size_t len_;
  };

  class collect_data
  {
  public:
    collect_data()
    {
    }

    template <typename context_type>
    class handler : public dataflow_step<context_type, void(const void *, size_t)>
    {
      using base_class = dataflow_step<context_type, void(const void *, size_t)>;

    public:
      handler(
        const context_type & context,
        const collect_data & args)
        : base_class(context)
      {
      }

      void operator ()(const void * data, size_t len)
      {
        if (0 == len) {
          this->next(this->buffer_.data(), this->buffer_.size());
        }
        else {
          this->buffer_.insert(this->buffer_.end(),
            reinterpret_cast<const uint8_t *>(data),
            reinterpret_cast<const uint8_t *>(data) + len);
          this->prev();
        }
      }

    private:
      std::vector<uint8_t> buffer_;
    };
  };
}

#endif // __VDS_CORE_DATA_BUFFER_H_