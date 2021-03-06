#ifndef __VDS_HTTP__WEBSOCKET_H_
#define __VDS_HTTP__WEBSOCKET_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "http_request.h"
#include "http_response.h"
#include "async_mutex.h"

namespace vds {

	class websocket_output : public std::enable_shared_from_this<websocket_output>
	{
	public:
    websocket_output(std::shared_ptr<stream_output_async<uint8_t>> target)
      : target_(target){
    }

		async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>> start(uint64_t message_size, bool is_binary);

    async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>> start_pong(uint64_t message_size);

  private:
    std::shared_ptr<stream_output_async<uint8_t>> target_;
    async_mutex async_mutex_;

    class output_stream : public stream_output_async<uint8_t>
    {
    public:
      output_stream(
        std::shared_ptr<websocket_output> target,
        uint64_t message_size)
        : target_(target), message_size_(message_size) {
      }

      async_task<expected<void>> write_async(
        const uint8_t *data,
        size_t len) override;

    private:
      std::shared_ptr<websocket_output> target_;
      uint64_t message_size_;
    };
	};

	class websocket : public std::enable_shared_from_this<websocket> {
	public:
		static vds::async_task<vds::expected<std::shared_ptr<stream_output_async<uint8_t>>>> open_connection(
			const vds::service_provider * sp,
      const std::shared_ptr<http_async_serializer> & output_stream,
			const http_message & msg,
      lambda_holder_t<async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>>, bool /*is_binary*/, std::shared_ptr<websocket_output>> handler);

	private:

    class websocket_handler : public stream_output_async<uint8_t>
    {
    public:
      websocket_handler(
        const vds::service_provider * sp,
        std::shared_ptr<stream_output_async<uint8_t>> target,
        lambda_holder_t<async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>>, bool /*is_binary*/, std::shared_ptr<websocket_output>> handler);

      async_task<expected<void>> write_async(
        const uint8_t *data,
        size_t len) override;

    private:
      const service_provider * sp_;
      std::shared_ptr<websocket_output> target_;
      lambda_holder_t<async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>>, bool /*is_binary*/, std::shared_ptr<websocket_output>> handler_;

      //Read state
      enum class read_state_t {
        HEADER,
        TEXT,
        BINARY,
        PING,
        CLOSED
      };

      read_state_t read_state_ = read_state_t::HEADER;

      uint8_t buffer_[1024 * 1024];
      uint8_t readed_ = 0;

      bool fin_;
      bool RSV1_;
      bool RSV2_;
      bool RSV3_;
      unsigned int Opcode_;

      bool has_mask_;
      uint64_t payloadLength_;

      uint8_t mask_[4];
      int mask_index_;

      std::shared_ptr<stream_output_async<uint8_t>> current_stream_;

      bool read_minimal(uint8_t min_size, const uint8_t * & data, size_t & len);
    };
	};
}//vds

#endif //__VDS_WEB_SERVER_LIB_WEBSOCKET_API_H_
