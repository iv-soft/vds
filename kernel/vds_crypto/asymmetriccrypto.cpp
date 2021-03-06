/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "asymmetriccrypto.h"
#include "private/asymmetriccrypto_p.h"
#include "crypto_exception.h"
#include "private/hash_p.h"
#include "symmetriccrypto.h"
#include "vds_debug.h"

///////////////////////////////////////////////////////////////////////
vds::asymmetric_private_key::asymmetric_private_key()
  : impl_(nullptr)
{
}

vds::asymmetric_private_key::asymmetric_private_key(asymmetric_private_key && original) noexcept
  : impl_(original.impl_)
{
  original.impl_ = nullptr;
}

vds::expected<void> vds::asymmetric_private_key::create(
  const vds::asymmetric_crypto_info & info)
{
  this->impl_ = new _asymmetric_private_key();
  return this->impl_->create(info);
}

vds::asymmetric_private_key::asymmetric_private_key(_asymmetric_private_key * impl)
: impl_(impl)
{
}

vds::asymmetric_private_key::~asymmetric_private_key()
{
  delete this->impl_;
}

vds::expected<vds::asymmetric_private_key> vds::asymmetric_private_key::generate(const asymmetric_crypto_info & info)
{
  auto impl = std::make_unique<_asymmetric_private_key>();
  CHECK_EXPECTED(impl->create(info));
  CHECK_EXPECTED(impl->generate());
  return asymmetric_private_key(impl.release());
}

vds::expected<vds::asymmetric_private_key> vds::asymmetric_private_key::parse(const std::string & value, const std::string & password)
{
  auto io = BIO_new_mem_buf((void*)value.c_str(), (int)value.length());
  auto key = PEM_read_bio_PrivateKey(io, 0, 0, password.empty() ? nullptr : (void *)password.c_str());

  auto result = std::make_unique<_asymmetric_private_key>();
  CHECK_EXPECTED(result->create(key));
  return asymmetric_private_key(result.release());
}

vds::expected<std::string> vds::asymmetric_private_key::str(const std::string & password/* = std::string()*/) const
{
  return this->impl_->str(password);
}

vds::expected<vds::const_data_buffer> vds::asymmetric_private_key::der(const std::string &password) const
{
  return this->impl_->der(password);
}

vds::expected<vds::asymmetric_private_key> vds::asymmetric_private_key::parse_der(
  const const_data_buffer & value,
  const std::string & password /*= std::string()*/)
{
  return _asymmetric_private_key::parse_der(value, password);
}


vds::expected<void> vds::asymmetric_private_key::load(const filename & filename, const std::string & password/* = std::string()*/)
{
  return this->impl_->load(filename, password);
}

vds::expected<void> vds::asymmetric_private_key::save(const filename & filename, const std::string & password/* = std::string()*/) const
{
  return this->impl_->save(filename, password);
}

vds::expected<vds::const_data_buffer> vds::asymmetric_private_key::decrypt(const void * data, size_t size) const
{
  return this->impl_->decrypt(data, size);
}

vds::asymmetric_private_key& vds::asymmetric_private_key::operator=(asymmetric_private_key&& original) {
  this->impl_ = original.impl_;
  original.impl_ = nullptr;
  return *this;
}

vds::expected<vds::const_data_buffer> vds::asymmetric_private_key::decrypt(const const_data_buffer & data) const
{
  return this->impl_->decrypt(data.data(), data.size());
}

///////////////////////////////////////////////////////////////////////
vds::_asymmetric_private_key::_asymmetric_private_key() 
: info_(nullptr), ctx_(nullptr), key_(nullptr) {
}

vds::expected<void> vds::_asymmetric_private_key::create(EVP_PKEY * key)
{
  vds_assert(nullptr != key);
  this->key_ = key;
  return expected<void>();
}

vds::expected<void> vds::_asymmetric_private_key::create(
  const vds::asymmetric_crypto_info & info)
{
  this->info_ = &info;
  this->key_ = nullptr;
    this->ctx_ = EVP_PKEY_CTX_new_id(info.id, NULL);
  if(nullptr == this->ctx_) {
    return vds::make_unexpected<std::runtime_error>("Unable to create RSA context");
  }

  return expected<void>();
}

vds::_asymmetric_private_key::~_asymmetric_private_key()
{
  if (nullptr != this->ctx_) {
    EVP_PKEY_CTX_free(this->ctx_);
  }
  else if (nullptr != this->key_) {
    EVP_PKEY_free(this->key_);
  }
}


vds::expected<void>  vds::_asymmetric_private_key::generate()
{
  vds_assert(nullptr != this->info_);

  if (0 >= EVP_PKEY_keygen_init(this->ctx_)) {
    return vds::make_unexpected<std::runtime_error>("Unable to init RSA context");
  }
  
  if (0 >= EVP_PKEY_CTX_set_rsa_keygen_bits(this->ctx_, this->info_->key_bits)) {
    return vds::make_unexpected<std::runtime_error>("Unable to set RSA bits");
  }
  
  if (0 >= EVP_PKEY_keygen(this->ctx_, &this->key_)) {
    return vds::make_unexpected<std::runtime_error>("Unable to generate RSA key");
  }

  return expected<void>();
}

vds::expected<std::string> vds::_asymmetric_private_key::str(const std::string & password/* = std::string()*/) const
{
  BIO * bio = BIO_new(BIO_s_mem());
  PEM_write_bio_PrivateKey(bio, this->key_, NULL, NULL, 0, NULL, password.empty() ? nullptr : (void *)password.c_str());

  auto len = BIO_pending(bio);
  std::string result;
  result.resize(len);
  if (len != BIO_read(bio, const_cast<char *>(result.data()), len)) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to BIO_read", error);
  }
  BIO_free_all(bio);

  return result;
}

vds::expected<vds::const_data_buffer> vds::_asymmetric_private_key::der(const std::string &password) const
{
  auto len = i2d_PrivateKey(this->key_, NULL);

  auto buf = (unsigned char *)OPENSSL_malloc(len);
  if (NULL == buf) {
    return vds::make_unexpected<std::runtime_error>("Out of memory at get DER format of certificate");
  }

  auto p = buf;
  i2d_PrivateKey(this->key_, &p);

  if(!password.empty()){
    std::vector<uint8_t> buffer;

    GET_EXPECTED(key, symmetric_key::from_password(password));
    GET_EXPECTED(result, symmetric_encrypt::encrypt(key, buf, len));
    
    OPENSSL_free(buf);
    return result;
  }
  else {
    const_data_buffer result(buf, len);
    OPENSSL_free(buf);

    return result;
  }
}

vds::expected<vds::asymmetric_private_key> vds::_asymmetric_private_key::parse_der(
  const const_data_buffer & value,
  const std::string & password /*= std::string()*/)
{
  if(!password.empty()){
    GET_EXPECTED(skey, symmetric_key::from_password(password));
    GET_EXPECTED(buffer, symmetric_decrypt::decrypt(
      skey,
      value.data(),
      value.size()));
    
    const unsigned char * p = buffer.data();
    auto key = d2i_AutoPrivateKey(NULL, &p, safe_cast<long>(buffer.size()));
    if (nullptr == key) {
      auto error = ERR_get_error();
      return vds::make_unexpected<crypto_exception>("d2i_AutoPrivateKey", error);
    }

    auto result = std::make_unique<_asymmetric_private_key>();
    CHECK_EXPECTED(result->create(key));
    return asymmetric_private_key(result.release());
  }
  else{
      const unsigned char * p = value.data();
      auto key = d2i_AutoPrivateKey(NULL, &p, safe_cast<long>(value.size()));
      if(nullptr == key){
        auto error = ERR_get_error();
        return vds::make_unexpected<crypto_exception>("d2i_AutoPrivateKey", error);
      }

      auto result = std::make_unique<_asymmetric_private_key>();
      CHECK_EXPECTED(result->create(key));
      return asymmetric_private_key(result.release());
  }
}

vds::expected<void> vds::_asymmetric_private_key::load(const filename & filename, const std::string & password/* = std::string()*/)
{
  auto in = BIO_new_file(filename.local_name().c_str(), "r");
  if (nullptr == in) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to private key " + filename.str(), error);
  }

  RSA * r = PEM_read_bio_RSAPrivateKey(in, NULL, NULL, password.empty() ? nullptr : (void *)password.c_str());
  if (nullptr == r) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to read file key " + filename.str(), error);
  }

  this->key_ = EVP_PKEY_new();
  EVP_PKEY_assign_RSA(this->key_, r);

  BIO_free(in);

  return expected<void>();
}

vds::expected<void> vds::_asymmetric_private_key::save(const filename & filename, const std::string & password/* = std::string()*/) const
{
  auto outf = BIO_new_file(filename.local_name().c_str(), "w");
  if (nullptr == outf) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed create file key " + filename.str(), error);
  }

  int r = PEM_write_bio_PrivateKey(outf, this->key_, NULL, NULL, 0, NULL, password.empty() ? nullptr : (void *)password.c_str());
  if (0 == r) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed save private key " + filename.str(), error);
  }

  BIO_free(outf);

  return expected<void>();
}
////////////////////////////////////////////////////////////////////////////////
const vds::asymmetric_crypto_info & vds::asymmetric_crypto::rsa2048()
{
  static asymmetric_crypto_info result = {
    EVP_PKEY_RSA,
    2048
  };

  return result;
}

const vds::asymmetric_crypto_info & vds::asymmetric_crypto::rsa4096()
{
  static asymmetric_crypto_info result = {
    EVP_PKEY_RSA,
    4096
  };

  return result;
}

vds::asymmetric_sign::asymmetric_sign()
  : impl_(nullptr)
{
}

vds::asymmetric_sign::~asymmetric_sign() {
  delete this->impl_;
}

vds::expected<void> vds::asymmetric_sign::create(
  const hash_info & hash_info,
  const asymmetric_private_key & key)
{
  this->impl_ = new _asymmetric_sign();
  return this->impl_->create(hash_info, key);
}

vds::expected<vds::const_data_buffer> vds::asymmetric_sign::signature()
{
  return this->impl_->signature();
}

vds::expected<vds::const_data_buffer> vds::asymmetric_sign::signature(
  const hash_info & hash_info,
  const asymmetric_private_key & key,
  const const_data_buffer & data)
{
  return signature(
    hash_info,
    key,
    data.data(),
    data.size());
}

vds::expected<vds::const_data_buffer> vds::asymmetric_sign::signature(
  const vds::hash_info& hash_info,
  const vds::asymmetric_private_key & key,
  const void* data,
  size_t data_size)
{
  thread_unprotect unprotect;
  _asymmetric_sign s;
  CHECK_EXPECTED(s.create(hash_info, key));
  CHECK_EXPECTED(s.write_async(reinterpret_cast<const uint8_t *>(data), data_size).get());
  CHECK_EXPECTED(s.write_async(nullptr, 0).get());
  return s.signature();
}

vds::async_task<vds::expected<void>> vds::asymmetric_sign::write_async( const uint8_t* data, size_t len) {
  return this->impl_->write_async(data, len);
}


////////////////////////////////////////////////////////////////////////////////
vds::_asymmetric_sign::_asymmetric_sign()
  : ctx_(nullptr), md_(nullptr) {  
}

vds::expected<void> vds::_asymmetric_sign::create(
    const hash_info & hash_info,
    const asymmetric_private_key & key)
{
  this->ctx_ = EVP_MD_CTX_create();
  if (nullptr == this->ctx_) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_MD_CTX_create", error);
  }

  this->md_ = EVP_get_digestbynid(hash_info.id);
  if (nullptr == this->md_) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_get_digestbynid", error);
  }

  if (1 != EVP_DigestInit_ex(this->ctx_, this->md_, NULL)) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  if (1 != EVP_DigestSignInit(this->ctx_, NULL, this->md_, NULL, key.impl_->key_)){
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  return expected<void>();
}

vds::_asymmetric_sign::~_asymmetric_sign()
{
  if (this->ctx_) {
    EVP_MD_CTX_destroy(this->ctx_);
  }
}

vds::async_task<vds::expected<void>> vds::_asymmetric_sign::write_async(
  const uint8_t * data,
  size_t data_size) {
	if (0 == data_size) {
		size_t req = 0;
		if (1 != EVP_DigestSignFinal(this->ctx_, NULL, &req) || req <= 0) {
			auto error = ERR_get_error();
			co_return vds::make_unexpected<crypto_exception>("EVP_DigestSignFinal", error);
		}

    this->sig_.resize(req);

		auto len = req;
		if (1 != EVP_DigestSignFinal(this->ctx_, this->sig_.data(), &len)) {
			const auto error = ERR_get_error();
      co_return vds::make_unexpected<crypto_exception>("EVP_DigestSignFinal", error);
		}

		if (len != req) {
			const auto error = ERR_get_error();
      co_return vds::make_unexpected<crypto_exception>("EVP_DigestSignFinal", error);
		}
	}
	else if (1 != EVP_DigestSignUpdate(this->ctx_, data, data_size)) {
    const auto error = ERR_get_error();
    co_return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  co_return expected<void>();
}

///////////////////////////////////////////////////////////////
vds::asymmetric_sign_verify::asymmetric_sign_verify()
: impl_(nullptr) {
}

vds::asymmetric_sign_verify::~asymmetric_sign_verify() {
  delete this->impl_;  
}

vds::expected<void> vds::asymmetric_sign_verify::create(
  const hash_info & hash_info,
  const asymmetric_public_key & key,
  const const_data_buffer & sig) {
  vds_assert(nullptr == this->impl_);
  this->impl_ = new _asymmetric_sign_verify();
  return this->impl_->create(hash_info, key, sig);
}

vds::expected<bool> vds::asymmetric_sign_verify::result() const {
  return this->impl_->result();
}

vds::expected<bool> vds::asymmetric_sign_verify::verify(
    const vds::hash_info &hash_info,
    const vds::asymmetric_public_key &key,
    const const_data_buffer &signature,
    const void *data,
    size_t data_size)
{
  thread_unprotect unprotect;

  _asymmetric_sign_verify s;
  CHECK_EXPECTED(s.create(hash_info, key, signature));
  CHECK_EXPECTED(s.write_async(reinterpret_cast<const uint8_t *>(data), data_size).get());
  CHECK_EXPECTED(s.write_async(nullptr, 0).get());
  return s.result();
}

vds::expected<bool> vds::asymmetric_sign_verify::verify(
    const hash_info &hash_info,
    const asymmetric_public_key &key,
    const const_data_buffer &signature,
    const const_data_buffer &data)
{
  return verify(hash_info, key, signature, data.data(), data.size());
}

vds::async_task<vds::expected<void>> vds::asymmetric_sign_verify::write_async(const uint8_t* data, size_t len) {
  return this->impl_->write_async(data, len);
}

///////////////////////////////////////////////////////////////
vds::_asymmetric_sign_verify::_asymmetric_sign_verify()
  : md_(nullptr), result_(false) {
  
}
vds::expected<void> vds::_asymmetric_sign_verify::create(
  const hash_info & hash_info,
  const asymmetric_public_key & key,
  const const_data_buffer & sig)
{
  this->sig_ = sig;
    this->ctx_ = EVP_MD_CTX_create();
  if (nullptr == this->ctx_) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_MD_CTX_create", error);
  }

  this->md_ = EVP_get_digestbynid(hash_info.id);
  if (nullptr == this->md_) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_get_digestbynid", error);
  }

  if (1 != EVP_DigestInit_ex(this->ctx_, this->md_, NULL)) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  if (1 != EVP_DigestVerifyInit(this->ctx_, NULL, this->md_, NULL, key.impl_->key_)) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  return expected<void>();
}

vds::_asymmetric_sign_verify::~_asymmetric_sign_verify()
{
  if (this->ctx_) {
    EVP_MD_CTX_destroy(this->ctx_);
  }
}

vds::async_task<vds::expected<void>> vds::_asymmetric_sign_verify::write_async(
  const uint8_t *data,
  size_t len) {
	if (0 == len) {
		this->result_ = (1 == EVP_DigestVerifyFinal(
        this->ctx_,
        const_cast<unsigned char *>(this->sig_.data()),
        this->sig_.size()));
	}
	else if (1 != EVP_DigestVerifyUpdate(this->ctx_, data, len)) {
    auto error = ERR_get_error();
    co_return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  co_return expected<void>();
}

vds::asymmetric_public_key& vds::asymmetric_public_key::operator=(asymmetric_public_key&& original) noexcept {
  delete this->impl_;
  this->impl_ = original.impl_;
  original.impl_ = nullptr;

  return *this;
}

//////////////////////////////////////////////////////////////////////
vds::asymmetric_public_key::asymmetric_public_key(_asymmetric_public_key * impl)
: impl_(impl)
{
}

vds::asymmetric_public_key::asymmetric_public_key(asymmetric_public_key && original)
  : impl_(original.impl_)
{
  original.impl_ = nullptr;
}

vds::asymmetric_public_key::asymmetric_public_key()
  : impl_(nullptr)
{
}

vds::asymmetric_public_key::~asymmetric_public_key()
{
  delete this->impl_;
}

vds::expected<vds::asymmetric_public_key> vds::asymmetric_public_key::create(const asymmetric_private_key & key)
{
  auto impl = std::make_unique<_asymmetric_public_key>();
  CHECK_EXPECTED(impl->create(key));

  return asymmetric_public_key(impl.release());
}

vds::expected<vds::asymmetric_public_key> vds::asymmetric_public_key::parse(const std::string & value)
{
  auto io = BIO_new_mem_buf((void*)value.c_str(), (int)value.length());
  auto key = PEM_read_bio_PUBKEY(io, 0, 0, 0);
  return asymmetric_public_key(new _asymmetric_public_key(key));
}

vds::expected<std::string> vds::asymmetric_public_key::str() const
{
  return this->impl_->str();
}

vds::expected<vds::asymmetric_public_key> vds::asymmetric_public_key::parse_der(const const_data_buffer& value) {
  return _asymmetric_public_key::parse_der(value);
}

vds::expected<vds::const_data_buffer> vds::asymmetric_public_key::der() const {
  return this->impl_->der();
}

vds::expected<vds::const_data_buffer> vds::asymmetric_public_key::fingerprint() const {
  return this->impl_->fingerprint();
}

vds::expected<void> vds::asymmetric_public_key::load(const filename & filename)
{
  return this->impl_->load(filename);
}

vds::expected<void> vds::asymmetric_public_key::save(const filename & filename)
{
  return this->impl_->save(filename);
}

vds::expected<vds::const_data_buffer> vds::asymmetric_public_key::encrypt(const const_data_buffer & data)
{
  return this->impl_->encrypt(data.data(), data.size());
}

vds::expected<vds::const_data_buffer> vds::asymmetric_public_key::encrypt(const void * data, size_t data_size)
{
  return this->impl_->encrypt(data, data_size);
}

//////////////////////////////////////////////////////////////////////
vds::_asymmetric_public_key::_asymmetric_public_key(EVP_PKEY * key)
  : info_(nullptr), key_(key)
{
}

vds::_asymmetric_public_key::~_asymmetric_public_key()
{
  if(nullptr != this->key_){
    EVP_PKEY_free(this->key_);
  }
}

vds::expected<void> vds::_asymmetric_public_key::create(const asymmetric_private_key & key)
{
  this->info_ = key.impl_->info_;

  auto len = i2d_PUBKEY(key.impl_->key_, NULL);
  unsigned char * buf = (unsigned char *)OPENSSL_malloc(len);

  unsigned char * p = buf;
  len = i2d_PUBKEY(key.impl_->key_, &p);

  const unsigned char * p1 = buf;
  this->key_ = d2i_PUBKEY(NULL, &p1, len);

  if (nullptr == this->key_) {
    auto error = ERR_get_error();
    OPENSSL_free(buf);

    return vds::make_unexpected<crypto_exception>("d2i_PUBKEY", error);
  }

  return expected<void>();
}

vds::expected<vds::const_data_buffer> vds::_asymmetric_public_key::fingerprint() const
{
  resizable_data_buffer data;
  CHECK_EXPECTED(data.add_uint32(7));
  CHECK_EXPECTED(data.add("ssh-rsa", 7));

  const BIGNUM *n,*e;
  RSA_get0_key(EVP_PKEY_get0_RSA(this->key_), &n, &e, NULL);

  auto fe = BN_bn2hex(e);
  GET_EXPECTED(buffer, (fe[0] > '8') ? hex::to_bytes(std::string("00") + fe) : hex::to_bytes(fe));
  OPENSSL_free(fe);

  CHECK_EXPECTED(data.add_uint32(buffer.size()));
  CHECK_EXPECTED(data.add(buffer));

  auto fn = BN_bn2hex(n);
  GET_EXPECTED_VALUE(buffer, (fn[0] > '8') ? hex::to_bytes(std::string("00") + fn) : hex::to_bytes(fn));
  OPENSSL_free(fn);

  CHECK_EXPECTED(data.add_uint32(buffer.size()));
  CHECK_EXPECTED(data.add(buffer));

  return hash::signature(hash::sha256(), data.move_data());
}

vds::expected<vds::const_data_buffer> vds::_asymmetric_public_key::der() const {
  const auto len = i2d_PublicKey(this->key_, NULL);

  const auto buf = (unsigned char *)OPENSSL_malloc(len);
  if (NULL == buf) {
    return vds::make_unexpected<std::runtime_error>("Out of memory at get DER format of public key");
  }

  auto p = buf;
  i2d_PublicKey(this->key_, &p);

  const_data_buffer result(buf, len);
  OPENSSL_free(buf);

  return result;
}

vds::expected<vds::asymmetric_public_key> vds::_asymmetric_public_key::parse_der(const const_data_buffer& value) {
  const unsigned char * p = value.data();
  auto key_data = d2i_RSAPublicKey(NULL, &p, safe_cast<long>(value.size()));
  if (nullptr == key_data) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("d2i_PublicKey", error);
  }

  auto key = EVP_PKEY_new();
  EVP_PKEY_assign_RSA(key, key_data);
  return asymmetric_public_key(new _asymmetric_public_key(key));
}


vds::expected<std::string> vds::_asymmetric_public_key::str() const
{
  BIO * bio = BIO_new(BIO_s_mem());
  PEM_write_bio_PUBKEY(bio, this->key_);

  auto len = BIO_pending(bio);
  std::string result;
  result.resize(len);
  if (len != BIO_read(bio, const_cast<char *>(result.data()), len)) {
    auto error = ERR_get_error();
    BIO_free_all(bio);
    return vds::make_unexpected<crypto_exception>("EVP_DigestInit_ex", error);
  }

  BIO_free_all(bio);

  return result;
}

vds::expected<void> vds::_asymmetric_public_key::load(const filename & filename)
{
  auto in = BIO_new_file(filename.local_name().c_str(), "r");
  if (nullptr == in) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to public key " + filename.str(), error);
  }

  RSA * r = PEM_read_bio_RSAPublicKey(in, NULL, NULL, NULL);
  if (nullptr == r) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to read public file key " + filename.str(), error);
  }

  this->key_ = EVP_PKEY_new();
  EVP_PKEY_assign_RSA(this->key_, r);

  BIO_free(in);

  return expected<void>();
}

vds::expected<void> vds::_asymmetric_public_key::save(const filename & filename) const
{
  auto outf = BIO_new_file(filename.local_name().c_str(), "w");
  if (nullptr == outf) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed create file key " + filename.str(), error);
  }

  int r = PEM_write_bio_PUBKEY(outf, this->key_);
  if (0 == r) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed save private key " + filename.str(), error);
  }

  BIO_free(outf);

  return expected<void>();
}

/////////////////////////////////////////////////////////////////////////////
vds::certificate::certificate()
: impl_(nullptr)
{
}

vds::certificate::certificate(_certificate * impl)
: impl_(impl)
{
}

vds::certificate::certificate(certificate && original)
  : impl_(original.impl_)
{
  original.impl_ = nullptr;
}

vds::certificate::~certificate()
{
  delete this->impl_;
}

vds::expected<vds::certificate> vds::certificate::parse(const std::string & value)
{
  auto io = BIO_new_mem_buf((void*)value.c_str(), (int)value.length());
  auto cert = PEM_read_bio_X509(io, 0, 0, 0);
  if(nullptr == cert){
    return vds::make_unexpected<std::runtime_error>("Invalid certificate format");
  }
  return certificate(new _certificate(cert));
}

vds::expected<std::string> vds::certificate::str() const
{
  return this->impl_->str();
}

vds::expected<vds::certificate> vds::certificate::parse_der(const const_data_buffer & value)
{
  auto p = value.data();
  auto cert = d2i_X509(NULL, &p, safe_cast<long>(value.size()));

  if (NULL == cert) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to parse certificate from DER", error);
  }

  return certificate(new _certificate(cert));
}

vds::expected<vds::const_data_buffer> vds::certificate::der() const
{
  return this->impl_->der();
}

vds::expected<void> vds::certificate::load(const filename & filename)
{
  return this->impl_->load(filename);
}

vds::expected<void> vds::certificate::save(const filename & filename) const
{
  return this->impl_->save(filename);
}

std::string vds::certificate::subject() const
{
  return this->impl_->subject();
}

std::string vds::certificate::issuer() const
{
  return this->impl_->issuer();
}

vds::expected<vds::const_data_buffer> vds::certificate::fingerprint(const vds::hash_info & hash_algo) const
{
  return this->impl_->fingerprint(hash_algo);
}

vds::expected<vds::certificate> vds::certificate::create_new(
  const asymmetric_public_key & new_certificate_public_key,
  const asymmetric_private_key & new_certificate_private_key,
  const create_options & options
)
{
  return _certificate::create_new(
    new_certificate_public_key,
    new_certificate_private_key,
    options);
}

vds::expected<vds::asymmetric_public_key> vds::certificate::public_key() const
{
  return this->impl_->public_key();
}

bool vds::certificate::is_ca_cert() const
{
  return this->impl_->is_ca_cert();
}

bool vds::certificate::is_issued(const vds::certificate& issuer) const
{
  return this->impl_->is_issued(issuer);
}

int vds::certificate::extension_count() const
{
  return this->impl_->extension_count();
}

int vds::certificate::extension_by_NID(int nid) const
{
  return this->impl_->extension_by_NID(nid);
}

vds::expected<vds::certificate_extension> vds::certificate::get_extension(int index) const
{
  return this->impl_->get_extension(index);
}

vds::certificate & vds::certificate::operator = (certificate && original)
{
  this->impl_ = original.impl_;
  original.impl_ = nullptr;
  return *this;
}

/////////////////////////////////////////////////////////////////////////////
vds::_certificate::_certificate(X509 * cert)
  : cert_(cert)
{
  vds_assert(nullptr != cert);
}

vds::_certificate::~_certificate()
{
  if(nullptr != this->cert_){
    X509_free(this->cert_);
  }
}

vds::expected<std::string> vds::_certificate::str() const
{
  BIO * bio = BIO_new(BIO_s_mem());
  PEM_write_bio_X509(bio, this->cert_);

  auto len = BIO_pending(bio);
  std::string result;
  result.resize(len);
  BIO_read(bio, const_cast<char *>(result.data()), len);
  BIO_free_all(bio);

  return result;
}

vds::expected<vds::const_data_buffer> vds::_certificate::der() const
{
  auto len = i2d_X509(this->cert_, NULL);

  auto buf = (unsigned char *)OPENSSL_malloc(len);
  if (NULL == buf) {
    return vds::make_unexpected<std::runtime_error>("Out of memory at get DER format of certificate");
  }

  auto p = buf;
  i2d_X509(this->cert_, &p);

  const_data_buffer result(buf, len);
  OPENSSL_free(buf);

  return result;
}

vds::expected<void> vds::_certificate::load(const filename & filename)
{
  auto in = BIO_new_file(filename.local_name().c_str(), "r");
  if(nullptr == in){
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("Failed to load certificate " + filename.str(), error);
  }

  this->cert_ = PEM_read_bio_X509(in, NULL, NULL, NULL);
  if(nullptr == this->cert_){
    auto error = ERR_get_error();
    BIO_free(in);
    return vds::make_unexpected<crypto_exception>("Failed to load certificate " + filename.str(), error);
  }
  
  BIO_free(in);

  return expected<void>();
}

vds::expected<void> vds::_certificate::save(const filename & filename) const
{
  auto out = BIO_new_file(filename.local_name().c_str(), "w");
  auto ret = PEM_write_bio_X509(out, this->cert_);
  BIO_free_all(out);

  if (1 != ret) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("PEM_write_bio_X509", error);
  }

  return expected<void>();
}

std::string vds::_certificate::subject() const
{
  char result[1024];
  X509_NAME_oneline(X509_get_subject_name(this->cert_), result, sizeof(result));

  return result;
}

std::string vds::_certificate::issuer() const
{
  char result[1024];
  X509_NAME_oneline(X509_get_issuer_name(this->cert_), result, sizeof(result));

  return result;
}

vds::expected<vds::const_data_buffer> vds::_certificate::fingerprint(const vds::hash_info & hash_algo) const
{
  unsigned char md[EVP_MAX_MD_SIZE];
  unsigned int n;
  if(!X509_digest(this->cert_, hash_algo.type, md, &n)){
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("X509_digest", error);
  }
  
  return const_data_buffer(md, n);
}


vds::expected<vds::certificate> vds::_certificate::create_new(
  const asymmetric_public_key & new_certificate_public_key,
  const asymmetric_private_key & new_certificate_private_key,
  const certificate::create_options & options
)
{
  X509 * x509 = X509_new();
  if (nullptr == x509) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("X509_new", error);
  }

    //if (!EVP_PKEY_assign_RSA(new_certificate_private_key.key(), x509)) {
    //  auto error = ERR_get_error();
    //  return vds::make_unexpected<crypto_exception>("X509_new", error);
    //}

    X509_set_version(x509, 2);
    ASN1_INTEGER_set(X509_get_serialNumber(x509), 1);
    X509_gmtime_adj(X509_get_notBefore(x509), 0);
    X509_gmtime_adj(X509_get_notAfter(x509), (long)60 * 60 * 24 * 365);
    auto key = new_certificate_public_key.impl_->key();
#if OPENSSL_VERSION_NUMBER < 0x10100000L
    CRYPTO_add(&key->references,1,CRYPTO_LOCK_EVP_PKEY);
#else
    EVP_PKEY_up_ref(key);
#endif
    X509_set_pubkey(x509, key);

    auto name = X509_get_subject_name(x509);
    X509_NAME_add_entry_by_txt(name, "C", MBSTRING_ASC, (const unsigned char *)options.country.c_str(), -1, -1, 0);
    X509_NAME_add_entry_by_txt(name, "O", MBSTRING_ASC, (const unsigned char *)options.organization.c_str(), -1, -1, 0);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC, (const unsigned char *)options.name.c_str(), -1, -1, 0);

    for(auto& extension : options.extensions) {
      add_ext(x509, extension.oid, extension.value.c_str());
    }

    if (nullptr == options.ca_certificate) {
      X509_set_issuer_name(x509, name);
    }
    else {
      X509_set_issuer_name(x509, X509_get_subject_name(options.ca_certificate->impl_->cert()));
    }

    /* Add various extensions: standard extensions */
    add_ext(x509, NID_basic_constraints, "critical,CA:TRUE");
    add_ext(x509, NID_key_usage, "critical,keyCertSign,cRLSign");
    add_ext(x509, NID_subject_key_identifier, "hash");

    /* Some Netscape specific extensions */
    add_ext(x509, NID_netscape_cert_type, "sslCA");
    add_ext(x509, NID_netscape_comment, "example comment extension");

    if (nullptr == options.ca_certificate) {
      if (!X509_sign(x509, new_certificate_private_key.impl_->key(), EVP_sha256())) {
        auto error = ERR_get_error();
        return vds::make_unexpected<crypto_exception>("X509_new", error);
      }
    }
    else {
      if (!X509_sign(x509, options.ca_certificate_private_key->impl_->key(), EVP_sha256())) {
        auto error = ERR_get_error();
        return vds::make_unexpected<crypto_exception>("X509_new", error);
      }
    }

    return certificate(new _certificate(x509));
}

vds::expected<vds::asymmetric_public_key> vds::_certificate::public_key() const
{
  auto key = X509_get_pubkey(this->cert_);
  if (nullptr == key) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("X509_get_pubkey", error);
  }
  return asymmetric_public_key(new _asymmetric_public_key(key));
}


bool vds::_certificate::is_ca_cert() const
{
  return (0 < X509_check_ca(this->cert_));
}

bool vds::_certificate::is_issued(const vds::certificate& issuer) const
{
  return (X509_V_OK == X509_check_issued(issuer.impl_->cert(), this->cert()));
}

bool vds::_certificate::add_ext(X509 * cert, int nid, const char * value)
{
  X509_EXTENSION * ex = X509V3_EXT_conf_nid(NULL, NULL, nid, const_cast<char *>(value));
  if (nullptr == ex) {
    return false;
  }

  X509_add_ext(cert, ex, -1);
  X509_EXTENSION_free(ex);

  return true;
}

int vds::_certificate::extension_count() const
{
  return X509_get_ext_count(this->cert_);
}

int vds::_certificate::extension_by_NID(int nid) const
{
  return X509_get_ext_by_NID(this->cert_, nid, -1);
}

vds::expected<vds::certificate_extension> vds::_certificate::get_extension(int index) const
{
  certificate_extension result;
  
  X509_EXTENSION * ext = X509_get_ext(this->cert_, index);
  if(nullptr != ext){
    auto obj = X509_EXTENSION_get_object(ext);
    result.oid = OBJ_obj2nid(obj);
    
    //char buf[256];
    //OBJ_obj2txt(buf, sizeof(buf), obj, 0);
    //result.name = buf;
    
    BIO *bio = BIO_new(BIO_s_mem());
    if(!X509V3_EXT_print(bio, ext, 0, 0)){
      //M_ASN1_OCTET_STRING_print(bio, ext);
      return vds::make_unexpected<std::runtime_error>("Unable get certificate extension");
    }
    (void)BIO_flush(bio);
    
    char buf[256];
    auto len = BIO_read(bio, buf, sizeof(buf));
    BIO_free(bio);
    
    result.value.assign(buf, len);
  }
  
  return result;
}

//////////////////////////////////////////////////////////
vds::certificate_store::certificate_store()
: impl_(new _certificate_store())
{
}

vds::certificate_store::certificate_store(_certificate_store* impl)
: impl_(impl) {
}

vds::certificate_store::certificate_store(certificate_store&& original) noexcept
: impl_(original.impl_){
  original.impl_ = nullptr;
}

vds::certificate_store::~certificate_store()
{
  delete this->impl_;
}

vds::expected<vds::certificate_store> vds::certificate_store::create() {
  auto impl = std::make_unique<_certificate_store>();
  CHECK_EXPECTED(impl->create());
  return certificate_store(impl.release());
}

vds::expected<void> vds::certificate_store::add(const vds::certificate& cert)
{
  return this->impl_->add(cert);
}


vds::expected<void> vds::certificate_store::load_locations(const std::string & location)
{
  return this->impl_->load_locations(location);
}

vds::expected<vds::certificate_store::verify_result> vds::certificate_store::verify(const vds::certificate& cert) const
{
  return this->impl_->verify(cert);
}

vds::certificate_store& vds::certificate_store::operator=(certificate_store&& original) noexcept {
  delete this->impl_;
  this->impl_ = original.impl_;
  original.impl_ = nullptr;
  return *this;
}

//////////////////////////////////////////////////////////
vds::_certificate_store::_certificate_store()
: store_(nullptr)
{
}

vds::_certificate_store::~_certificate_store()
{
  if (nullptr != this->store_) {
    X509_STORE_free(this->store_);
  }
}

vds::expected<void> vds::_certificate_store::create()
{
  vds_assert(nullptr == this->store_);
  this->store_ = X509_STORE_new();
  if (nullptr == this->store_) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("X509_get_pubkey", error);
  }

  return expected<void>();
}


vds::expected<void> vds::_certificate_store::add(const vds::certificate& cert)
{
  if(0 >= X509_STORE_add_cert(this->store_, cert.impl_->cert())){
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("unable to add certificate to store", error);
  }

  return expected<void>();
}


vds::expected<void> vds::_certificate_store::load_locations(const std::string & location)
{
  if(0 >= X509_STORE_load_locations(this->store_, location.c_str(), NULL)){
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("unable to load certificates at " + location + " to store", error);
  }

  return expected<void>();
}

vds::expected<vds::certificate_store::verify_result> vds::_certificate_store::verify(const vds::certificate& cert) const
{
  X509_STORE_CTX * vrfy_ctx = X509_STORE_CTX_new();
  if (nullptr == vrfy_ctx) {
    auto error = ERR_get_error();
    return vds::make_unexpected<crypto_exception>("X509_STORE_CTX_new", error);
  }

  X509_STORE_CTX_init(vrfy_ctx, this->store_, cert.impl_->cert(), NULL);

  certificate_store::verify_result result;
  if (0 == X509_verify_cert(vrfy_ctx)) {
    result.error_code = X509_STORE_CTX_get_error(vrfy_ctx);
    result.error = X509_verify_cert_error_string(result.error_code);
    auto error_cert = X509_STORE_CTX_get_current_cert(vrfy_ctx);
    
    char issuer[1024];
    X509_NAME_oneline(X509_get_issuer_name(error_cert), issuer, sizeof(issuer));
    
    result.issuer = issuer;
  }
  else {
    result.error_code = 0;
  }

  X509_STORE_CTX_free(vrfy_ctx);
  
  return result;
}


