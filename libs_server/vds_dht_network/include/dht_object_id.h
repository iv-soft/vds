#ifndef __VDS_P2P_NETWORK_NODE_ID_T_H_
#define __VDS_P2P_NETWORK_NODE_ID_T_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "hash.h"
#include "vds_debug.h"
#include "binary_serialize.h"
#include "resizable_data_buffer.h"
#include "encoding.h"
#include "crypto_service.h"

namespace vds {
  namespace dht {
    class dht_object_id {
    public:
      static const_data_buffer distance(const const_data_buffer &left, const const_data_buffer &right) {
        size_t min_size;
        size_t max_size;
        const uint8_t * pleft;
        const uint8_t * pright;

        if(left.size() < right.size()) {
          min_size = left.size();
          max_size = right.size();
          pleft = left.data();
          pright = right.data();
        }
        else {
          min_size = right.size();
          max_size = left.size();
          pleft = right.data();
          pright = left.data();

        }

        auto p = alloca(max_size);
        auto result = static_cast<uint8_t *>(p);

        auto offset = min_size;
        while (0 != offset--) {
          *result++ = *pleft++ ^ *pright++;
        }

        if (max_size > min_size) {
          memcpy(result, pright, max_size - min_size);
        }

        return const_data_buffer(p, max_size);
      }

      static size_t distance_exp(const const_data_buffer &left, const const_data_buffer &right) {
        const auto min_size = (left.size() < right.size()) ? left.size() : right.size();

        for (size_t i = 0; i < min_size; ++i) {
          const auto b = static_cast<uint8_t>(left[i] ^ right[i]);
          if (0 == b) {
            continue;
          }

          auto result = (i << 3);

          uint8_t mask = 0x80;
          while (0 != mask) {
            if (b >= mask) {
              break;
            }

            ++result;
            mask >>= 1;
          }

          return result;
        }

        if (left.size() == right.size()) {
          return 0;
        } else {
          return min_size << 3;
        }
      }

      static const_data_buffer generate_random_id() {
        uint8_t buffer[32];//MD5 size
        crypto_service::rand_bytes(buffer, sizeof(buffer));

        return const_data_buffer(buffer, sizeof(buffer));
      }

      static const_data_buffer generate_random_id(const const_data_buffer &original, size_t exp_index) {
        std::vector<uint8_t> result(original.size());
        memcpy(result.data(), original.data(), original.size());

        result[exp_index / 8] ^= (0x80 >> (exp_index % 8));
        result[exp_index / 8] ^= ((0x80 >> (exp_index % 8)) - 1) & static_cast<uint8_t>(std::rand());

        for (size_t i = exp_index / 8 + 1; i < original.size(); ++i) {
          result[i] = static_cast<uint8_t>(std::rand());
        }

        vds_assert(exp_index == distance_exp(original, const_data_buffer(result.data(), result.size())));

        return const_data_buffer(result.data(), result.size());
      }

      static expected<const_data_buffer> from_user_email(const std::string & user_email){
        return from_string("email:" + user_email);
      }

      static expected<const_data_buffer> my_record_channel(const std::string & user_email) {
        return from_string("my.records.channel:" + user_email);
      }

      static expected<std::string> user_credentials_to_key(const std::string & user_password) {
        GET_EXPECTED(sig, hash::signature(hash::sha256(), user_password.c_str(), user_password.length()));
        return user_credentials_to_key(sig);
      }

      static std::string user_credentials_to_key(const const_data_buffer & password_hash) {
        auto ph = base64::from_bytes(password_hash);
        return std::to_string(ph.length()) + "." + ph;
      }

    private:
      static expected<const_data_buffer> from_string(const std::string & value){
        return hash::signature(hash::sha256(), value.c_str(), value.length());
      }
    };
  }
}

#endif //__VDS_P2P_NETWORK_NODE_ID_T_H_
