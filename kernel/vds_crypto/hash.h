#ifndef __VDS_CRYPTO_HASH_H_
#define __VDS_CRYPTO_HASH_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

namespace vds {
  struct hash_info;

  class _hash;
  class hash
  {
  public:
    static const hash_info & md5();
    static const hash_info & sha256();

    hash(const hash_info & info);
    ~hash();

    void update(
      const void * data,
      size_t len);

    void final();

    const const_data_buffer & signature() const;
    
    static const_data_buffer signature(
      const hash_info & info,
      const const_data_buffer & data);

    static const_data_buffer signature(
      const hash_info & info,
      const void * data,
      size_t data_size);
    
  private:
    _hash * impl_;
  };

  class _hmac;
  class hmac
  {
  public:
    hmac(const std::string & key, const hash_info & info = hash::sha256());
    ~hmac();

    void update(
      const void * data,
      size_t len);

    void final();

    const const_data_buffer signature() const;

  private:
    _hmac * impl_;
  };
}

#endif // __HASH_H_
