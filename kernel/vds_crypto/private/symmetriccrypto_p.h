#ifndef __VDS_CRYPTO_SYMMETRICCRYPTO_P_H_
#define __VDS_CRYPTO_SYMMETRICCRYPTO_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "symmetriccrypto.h"
#include "crypto_exception.h"

namespace vds {
  class symmetric_encrypt;
  class symmetric_decrypt;
    
  class _symmetric_crypto_info
  {
  public:
    _symmetric_crypto_info(const EVP_CIPHER * cipher);
    
    const EVP_CIPHER * cipher() const {
      return this->cipher_;
    }
    
    size_t key_size() const;
    size_t iv_size() const;

    size_t block_size() const
    {
      return EVP_CIPHER_block_size(this->cipher_);
    }

  private:
    const EVP_CIPHER * cipher_;
  };
  
  class _symmetric_crypto
  {
  public:
    static const symmetric_crypto_info & aes_256_cbc();
  };
  
  class _symmetric_key
  {
  public:
    _symmetric_key(
        const symmetric_crypto_info & crypto_info,
        uint8_t * key,
        uint8_t * iv);
    ~_symmetric_key();
    
    const uint8_t * key() const {
      return this->key_;
    }
    
    const uint8_t * iv() const {
      return this->iv_;
    }    
    
  private:
    friend class symmetric_encrypt;
    friend class symmetric_decrypt;
    friend class _symmetric_encrypt;
    friend class _symmetric_decrypt;
    friend class symmetric_key;

    const symmetric_crypto_info & crypto_info_;
    uint8_t * key_;
    uint8_t * iv_;
  };
  
  class _symmetric_encrypt
  {
  public:
    _symmetric_encrypt()
    : ctx_(nullptr),
      block_size_(0),
      input_buffer_(nullptr),
      input_buffer_offset_(0),
      output_buffer_(nullptr) {
    }

    ~_symmetric_encrypt()
    {
      delete[] this->input_buffer_;
      delete[] this->output_buffer_;

      if (nullptr != this->ctx_) {
        EVP_CIPHER_CTX_free(this->ctx_);
      }
    }

    expected<void> create(
      const symmetric_key & key,
      const std::shared_ptr<stream_output_async<uint8_t>> & target) {
      this->target_ = target;
      this->ctx_ = EVP_CIPHER_CTX_new();
      this->block_size_ = key.block_size();
      this->input_buffer_ = new uint8_t[key.block_size()];
      this->input_buffer_offset_ = 0;
      this->output_buffer_ = new uint8_t[2 * key.block_size()];

      if (nullptr == this->ctx_) {
        return vds::make_unexpected<std::runtime_error>("Create crypto context failed");
      }

      if (1 != EVP_EncryptInit_ex(
        this->ctx_,
        key.impl_->crypto_info_.impl_->cipher(),
        nullptr,
        key.impl_->key(),
        key.impl_->iv())) {
        return vds::make_unexpected<std::runtime_error>("Create crypto context failed");
      }

      return expected<void>();
    }

    vds::async_task<vds::expected<void>> write_async(
        const uint8_t * input_buffer,
        size_t input_buffer_size) {
      if (0 < input_buffer_size) {
        while (0 < input_buffer_size) {
          auto s = this->block_size_ - this->input_buffer_offset_;
          if (s > input_buffer_size) {
            s = input_buffer_size;
          }

          memcpy(this->input_buffer_ + this->input_buffer_offset_, input_buffer, s);

          this->input_buffer_offset_ += s;
          input_buffer += s;
          input_buffer_size -= s;

          if (this->input_buffer_offset_ == this->block_size_) {
            int len = 2 * this->block_size_;

            if (0 == EVP_CipherUpdate(this->ctx_,
                                      reinterpret_cast<unsigned char *>(this->output_buffer_), &len,
                                      reinterpret_cast<const unsigned char *>(this->input_buffer_),
                                      (int) this->block_size_)) {
              auto error = ERR_get_error();
              co_return vds::make_unexpected<crypto_exception>("EVP_CipherUpdate failed", error);
            }

            if(0 < len) {
              CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
            }

            this->input_buffer_offset_ = 0;
          }
        }
      } else {
        if(0 < this->input_buffer_offset_){
//          while(this->input_buffer_offset_ != this->block_size_) {
//            this->input_buffer_[this->input_buffer_offset_++] = 0x8F;//Padding
//          }

          int len = 2 * this->block_size_;
          if (0 == EVP_CipherUpdate(this->ctx_,
                                    reinterpret_cast<unsigned char *>(this->output_buffer_), &len,
                                    reinterpret_cast<const unsigned char *>(this->input_buffer_), (int)this->input_buffer_offset_)) {
            auto error = ERR_get_error();
            co_return vds::make_unexpected<crypto_exception>("EVP_CipherUpdate failed", error);
          }

          if(0 < len) {
            CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
          }
        }

        int len = 2 * this->block_size_;
        if (0 == EVP_CipherFinal_ex(
            this->ctx_,
            reinterpret_cast<unsigned char *>(this->output_buffer_), &len)) {
          auto error = ERR_get_error();
          co_return vds::make_unexpected<crypto_exception>("EVP_CipherFinal_ex failed", error);
        }

        if(0 < len) {
          CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
        }

        CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(nullptr, 0));
      }

      co_return vds::expected<void>();
    }

  private:
    std::shared_ptr<stream_output_async<uint8_t>> target_;
    EVP_CIPHER_CTX * ctx_;
    size_t block_size_;

    uint8_t  * input_buffer_;
    size_t    input_buffer_offset_;
    uint8_t  * output_buffer_;
  };

  class _symmetric_decrypt
  {
  public:
    _symmetric_decrypt()
      : ctx_(nullptr),
        block_size_(0),
        input_buffer_(nullptr),
        input_buffer_offset_(0),
        output_buffer_(nullptr) {
    }

    ~_symmetric_decrypt()
    {
      delete[] this->input_buffer_;
      delete[] this->output_buffer_;

      if (nullptr != this->ctx_) {
        EVP_CIPHER_CTX_free(this->ctx_);
      }
    }

    expected<void> create(
      const symmetric_key & key,
      const std::shared_ptr<stream_output_async<uint8_t>> & target) {
      this->target_ = target;
      this->ctx_ = EVP_CIPHER_CTX_new();
      this->block_size_ = key.block_size();
      this->input_buffer_ = new uint8_t[key.block_size()];
      this->input_buffer_offset_ = 0;
      this->output_buffer_ = new uint8_t[2 * key.block_size()];

      if (nullptr == this->ctx_) {
        return vds::make_unexpected<std::runtime_error>("Create crypto context failed");
      }

      if (1 != EVP_DecryptInit_ex(
        this->ctx_,
        key.impl_->crypto_info_.impl_->cipher(),
        nullptr,
        key.impl_->key(),
        key.impl_->iv())) {
        return vds::make_unexpected<std::runtime_error>("Create decrypt context failed");
      }

      return expected<void>();
    }

    vds::async_task<vds::expected<void>> write_async(
        const uint8_t * input_buffer,
        size_t input_buffer_size)
    {
      if (0 < input_buffer_size) {
        while (0 < input_buffer_size) {
          auto s = this->block_size_ - this->input_buffer_offset_;
          if (s > input_buffer_size) {
            s = input_buffer_size;
          }

          memcpy(this->input_buffer_, input_buffer, s);

          this->input_buffer_offset_ += s;
          input_buffer += s;
          input_buffer_size -= s;

          if (this->input_buffer_offset_ == this->block_size_) {
            int len = 2 * this->block_size_;

            if (0 == EVP_CipherUpdate(this->ctx_,
                                      reinterpret_cast<unsigned char *>(this->output_buffer_), &len,
                                      reinterpret_cast<const unsigned char *>(this->input_buffer_),
                                      (int) this->block_size_)) {
              auto error = ERR_get_error();
              co_return vds::make_unexpected<crypto_exception>("EVP_CipherUpdate failed", error);
            }

            if(0 < len) {
              CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
            }

            this->input_buffer_offset_ = 0;
          }
        }
      } else {
        if(0 < this->input_buffer_offset_){
//          while(this->input_buffer_offset_ != this->block_size_) {
//            this->input_buffer_[this->input_buffer_offset_++] = 0x8F;//Padding
//          }

          int len = 2 * this->block_size_;
          if (0 == EVP_CipherUpdate(this->ctx_,
                                    reinterpret_cast<unsigned char *>(this->output_buffer_), &len,
                                    reinterpret_cast<const unsigned char *>(this->input_buffer_), (int)this->input_buffer_offset_)) {
            auto error = ERR_get_error();
            co_return vds::make_unexpected<crypto_exception>("EVP_CipherUpdate failed", error);
          }

          if(0 < len) {
            CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
          }
        }

        int len = 2 * this->block_size_;
        if (0 == EVP_CipherFinal_ex(
            this->ctx_,
            reinterpret_cast<unsigned char *>(this->output_buffer_), &len)) {
          auto error = ERR_get_error();
          co_return vds::make_unexpected<crypto_exception>("EVP_CipherFinal_ex failed", error);
        }

        if(0 < len) {
          CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(this->output_buffer_, len));
        }

        CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(nullptr, 0));
      }

      co_return expected<void>();
    }

  private:
    std::shared_ptr<stream_output_async<uint8_t>> target_;
    EVP_CIPHER_CTX * ctx_;
    size_t block_size_;
    uint8_t  * input_buffer_;
    size_t    input_buffer_offset_;
    uint8_t  * output_buffer_;
  };

}

#endif // __VDS_CRYPTO_SYMMETRICCRYPTO_P_H_
