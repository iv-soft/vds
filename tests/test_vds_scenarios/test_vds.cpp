/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "test_vds.h"
#include "vds_mock.h"
#include "test_config.h"
#include "compare_data.h"

TEST(test_vds, integration_test)
{
    vds_mock mock;

    CHECK_EXPECTED_GTEST(mock.start(16, true));

    std::cout << "Initiating root\n";
    CHECK_EXPECTED_GTEST(mock.init_root(5));
    CHECK_EXPECTED_GTEST(mock.allocate_storage());

    //Waiting to sync logs
    CHECK_EXPECTED_GTEST(mock.sync_wait());

    std::cout << "Create channel...\n";
    GET_EXPECTED_GTEST(channel, mock.create_channel(0, "test", "Test file"));

    size_t len;
    do
    {
      vds::crypto_service::rand_bytes(&len, sizeof(len));
      len %= 32 * 1024 * 1024;
    } while (len < 1024 || len > 32 * 1024 * 1024);

    std::unique_ptr<uint8_t> buffer(new uint8_t[len]);
    vds::crypto_service::rand_bytes(buffer.get(), (int)len);

    //Waiting to sync logs
    CHECK_EXPECTED_GTEST(mock.sync_wait());

    mock.allow_write_channel(3, channel.id());

    mock.get_sp(3);

    std::cout << "Upload local file...\n";
    GET_EXPECTED_GTEST(file_hash, mock.upload_file(
      3,
      channel.id(),
      "test data",
      "application/octet-stream",
      std::make_shared<vds::buffer_stream_input_async>(vds::const_data_buffer(buffer.get(), len))));

    std::cout << "Download local file...\n";

    auto result_data = std::make_shared<compare_data_async<uint8_t>>(buffer.get(), len);

    GET_EXPECTED_GTEST(result, mock.download_data(3, channel.id(), "test data", file_hash).get());

    ASSERT_EQ(len, result.size);
    CHECK_EXPECTED_GTEST(result.body->copy_to(result_data).get());

    mock.allow_read_channel(4, channel.id());
    //Waiting to sync logs
    CHECK_EXPECTED_GTEST(mock.sync_wait());
    std::cout << "Download file...\n";

    result_data = std::make_shared<compare_data_async<uint8_t>>(buffer.get(), len);

    bool is_final = false;
    std::string error;
    mock.download_data(4, channel.id(), "test data", file_hash).then([&is_final, &error, result_data](vds::expected<vds::file_manager::file_operations::download_result_t> result) {
      if (result.has_error()) {
        error = result.error()->what();
        is_final = true;
      }
      else {
        auto value = std::move(result.value());
        auto body = value.body;
        body->copy_to(result_data).then([&is_final, &error, body](vds::expected<void> r) {
          if (r.has_error()) {
            error = r.error()->what();
          }
          is_final = true;
        });
      }
    });

    while (!is_final) {
      CHECK_EXPECTED_GTEST(mock.dump_log());
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    if (error.empty()) {
      std::cout << "Done...\n";
    }
    else {
      std::cout << error << std::endl;
    }

    CHECK_EXPECTED_GTEST(mock.stop());

    ASSERT_EQ(len, result.size);
}