/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#ifndef __TEST_VDS_LIBS__COMPARE_DATA_H_
#define __TEST_VDS_LIBS__COMPARE_DATA_H_

#include "targetver.h"
#include <memory>
#include "stream.h"
#include "expected.h"

template <typename item_type>
class compare_data_async : public vds::stream_output_async<item_type>
{
public:
  compare_data_async(
    const item_type *data,
    size_t len)
    : data_(data),
    len_(len)
  {
  }

  vds::async_task<vds::expected<void>> write_async(
    const item_type * data,
    size_t len) override
  {
    if (0 == len) {
      if (0 != this->len_) {
        co_return vds::make_unexpected<std::runtime_error>("Unexpected end of stream while comparing data");
      }

      co_return vds::expected<void>();
    }

    if (this->len_ < len) {
      co_return vds::make_unexpected<std::runtime_error>("Unexpected data while comparing data");
    }

    if (0 != memcmp(this->data_, data, len)) {
      co_return vds::make_unexpected<std::runtime_error>("Compare data error");
    }

    this->data_ += len;
    this->len_ -= len;

    co_return vds::expected<void>();
  }

private:
  const item_type * data_;
  size_t len_;
};



#endif // __TEST_VDS_LIBS__COMPARE_DATA_H_
