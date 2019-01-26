
#include "stdafx.h"
#include "private/upload_stream_task_p.h"
#include "db_model.h"
#include "dht_network_client.h"

vds::_upload_stream_task::_upload_stream_task()
: total_size_(0), readed_(0) {
}

vds::async_task<vds::expected<std::list<vds::transactions::user_message_transaction::file_block_t>>> vds::_upload_stream_task::start(
  const service_provider * sp,
    const std::shared_ptr<stream_input_async<uint8_t>> & input_stream) {

  GET_EXPECTED_VALUE_ASYNC(this->total_hash_, hash::create(hash::sha256()));
  auto result = std::list<transactions::user_message_transaction::file_block_t>();

  auto network_client = sp->get<dht::network::client>();
  for (bool is_eof = false; !is_eof;) {
    uint64_t readed;
    if (this->readed_ == 0) {
      GET_EXPECTED_VALUE_ASYNC(this->readed_, co_await input_stream->read_async(this->buffer_, sizeof(this->buffer_)));

      if (this->readed_ == 0) {
        break;
      }

      this->total_size_ += this->readed_;
      CHECK_EXPECTED_ASYNC(this->total_hash_.update(this->buffer_, this->readed_));
    }

    auto block_info = co_await network_client->start_save(sp, [pthis = this->shared_from_this(), input_stream, &readed, &is_eof](
      const std::shared_ptr<stream_output_async<uint8_t>>& stream) -> async_task<expected<void>> {
      readed = 0;
      for(;;) {
        if(pthis->readed_ == 0) {
          GET_EXPECTED_VALUE_ASYNC(pthis->readed_, co_await input_stream->read_async(pthis->buffer_, sizeof(pthis->buffer_)));

          if(pthis->readed_ == 0) {
            is_eof = true;
            break;
          }

          pthis->total_size_ += pthis->readed_;
          CHECK_EXPECTED_ASYNC(pthis->total_hash_.update(pthis->buffer_, pthis->readed_));
        }

        auto len = pthis->readed_;
        if(len + readed > dht::network::service::BLOCK_SIZE) {
          len = dht::network::service::BLOCK_SIZE - readed;
        }
        CHECK_EXPECTED_ASYNC(co_await stream->write_async(pthis->buffer_, len));
        
        readed += len;
        pthis->readed_ -= len;
        if(0 != pthis->readed_) {
          memmove(pthis->buffer_, pthis->buffer_ + len, pthis->readed_);
        }
        if (readed >= dht::network::service::BLOCK_SIZE) {
          break;
        }
      }
      CHECK_EXPECTED_ASYNC(co_await stream->write_async(nullptr, 0));
      co_return expected<void>();
    });
    CHECK_EXPECTED_ERROR_ASYNC(block_info);

    result.push_back(transactions::user_message_transaction::file_block_t{
      /*block_id =*/ block_info.value().id,
      /*block_key =*/ block_info.value().key,
      /*object_ids*/block_info.value().object_ids,
      /*block_size =*/ readed
    });
  }
  CHECK_EXPECTED_ASYNC(this->total_hash_.final());

  co_return result;
}
