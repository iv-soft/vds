/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "test_vds.h"
#include "vds_mock.h"

TEST(test_vds, test_initial)
{
  try{
    vds_mock mock;

    mock.start(5);


    size_t len;
    do
    {
      vds::crypto_service::rand_bytes(&len, sizeof(len));
      len %= 32 * 1024 * 1024;
    } while (len < 1024 || len > 32 * 1024 * 1024);

    std::unique_ptr<unsigned char> buffer(new unsigned char[len]);
    vds::crypto_service::rand_bytes(buffer.get(), (int)len);

    mock.upload_file(3, "test data", buffer.get(), len);

    auto result = mock.download_data(4, "test data");

    mock.stop();

    ASSERT_EQ(len, result.size());
    ASSERT_EQ(memcmp(buffer.get(), result.data(), len), 0);
  }
  catch(std::exception * ex){
    FAIL() << std::unique_ptr<std::exception>(ex)->what();
  }
}