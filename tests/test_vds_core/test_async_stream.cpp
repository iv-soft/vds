//#include "stdafx.h"
//#include "test_async_stream.h"
//#include "mt_service.h"
//#include "random_reader.h"
//#include "compare_data.h"
//
//#include "async_buffer.h"
//#include "random_buffer.h"
//#include "test_config.h"
//
//TEST(mt_tests, test_async_stream) {
//  vds::service_registrator registrator;
//
//  vds::console_logger logger(
//    test_config::instance().log_level(),
//    test_config::instance().modules());
//  vds::mt_service mt_service;
//
//  registrator.add(logger);
//  registrator.add(mt_service);
//
//  auto sp = registrator.build("test_async_stream");
//  registrator.start(sp);
//  
//  vds::mt_service::enable_async(sp);
//
//  random_buffer data;
//  auto stream = std::make_shared<vds::continuous_buffer<uint8_t>>();
//
//  vds::barrier b;
//  std::shared_ptr<std::exception> err;
//
//  vds::async_series(
//    vds::create_std::future(
//      [&data, stream](
//        const std::function<void()> & done,
//        const vds::error_handler & error,
//        ) {
//        vds::mt_service::async(sp, [&data, stream, done, error, sp] {
//          vds::dataflow(
//            random_reader<uint8_t>(data.data(), data.size()),
//            vds::stream_write<vds::continuous_buffer<uint8_t>>(stream)
//          ).wait(done, error, sp);
//        });
//      }
//    ),
//    vds::create_std::future(
//      [&data, stream](
//        const std::function<void()> & done,
//        const vds::error_handler & error,
//        ) {
//        vds::mt_service::async(sp, [&data, stream, done, error, sp] {
//          vds::dataflow(
//            vds::stream_read<vds::continuous_buffer<uint8_t >> (stream),
//            compare_data<uint8_t>(data.data(), data.size())
//          ).wait(done, error, sp);
//        });
//      }
//    )
//  )
//    .wait(
//      [&b]() {
//        b.set();
//      },
//      [&b, &err]( const std::shared_ptr<std::exception> & ex) {
//        err = ex;
//        b.set();
//      },
//        sp
//        );
//
//      b.wait();
//      registrator.shutdown(sp);
//      if (err) {
//        GTEST_FAIL() << err->what();
//      }
//}
