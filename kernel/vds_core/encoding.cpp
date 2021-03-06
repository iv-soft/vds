/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include <locale>
#include <codecvt>
#include "encoding.h"
#include "resizable_data_buffer.h"
#include "vds_debug.h"

std::wstring vds::utf16::from_utf8(const std::string & original)
{
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> convert;
  return convert.from_bytes(original);
}

vds::expected<std::string> vds::utf16::to_utf8(const std::wstring & original)
{
  std::string result;
  result.reserve(original.length());

  for (auto ch : original) {
    CHECK_EXPECTED(utf8::add(result, ch));
  }
  return result;
}

vds::expected<size_t> vds::utf8::char_size(char first_utf8_symbol)
{
  if (0 == (0b10000000 & (uint8_t)first_utf8_symbol)) {
    return 1;
  }
  else if (0b11000000 == (0b11100000 & (uint8_t)first_utf8_symbol)) {
    return 2;
  }
  else if (0b11100000 == (0b11110000 & (uint8_t)first_utf8_symbol)) {
    return 3;
  }
  else if (0b11110000 == (0b11111000 & (uint8_t)first_utf8_symbol)) {
    return 4;
  }
  else {
    return vds::make_unexpected<std::runtime_error>("Invalid UTF8 string");
  }
}


vds::expected<wchar_t> vds::utf8::next_char(const char *& utf8string, size_t & len)
{
  if (len == 0) {
    return 0;
  }

  wchar_t result;
  if (0 == (0b10000000 & (uint8_t)*utf8string)) {
    result = (wchar_t)*utf8string++;
    --len;
  }
  else if (0b11000000 == (0b11100000 & (uint8_t)*utf8string)) {
    if (len < 2 || (0b10000000 != (0b11000000 & (uint8_t)utf8string[1]))) {
      return vds::make_unexpected<std::runtime_error>("Invalid UTF8 string");
    }

    result = (wchar_t)((((0b00011111 & (uint8_t)utf8string[0]) << 6) | (0b00111111 & (uint8_t)utf8string[1])));
    utf8string += 2;
    len -= 2;
  }
  else if (0b11100000 == (0b11110000 & (uint8_t)*utf8string)) {
    if (len < 3
      || (0b10000000 != (0b11000000 & (uint8_t)utf8string[1]))
      || (0b10000000 != (0b11000000 & (uint8_t)utf8string[2]))) {
      return vds::make_unexpected<std::runtime_error>("Invalid UTF8 string");
    }

    result = (wchar_t)(//0x800 +
      (((0b00011111 & (uint8_t)utf8string[0]) << 12)
        | ((0b00111111 & (uint8_t)utf8string[1]) << 6)
        | (0b00111111 & (uint8_t)utf8string[2])
      ));
    utf8string += 3;
    len -= 3;
  }
  else if (0b11110000 == (0b11111000 & (uint8_t)*utf8string)) {
    if (len < 4
      || (0b10000000 != (0b11000000 & (uint8_t)utf8string[1]))
      || (0b10000000 != (0b11000000 & (uint8_t)utf8string[2]))
      || (0b10000000 != (0b11000000 & (uint8_t)utf8string[3]))) {
      return vds::make_unexpected<std::runtime_error>("Invalid UTF8 string");
    }

    result = (wchar_t)(//0x10000 +
      (((0b00011111 & (uint8_t)utf8string[0]) << 18)
      | ((0b00111111 & (uint8_t)utf8string[1]) << 12)
      | ((0b00111111 & (uint8_t)utf8string[2]) << 6)
      | (0b00111111 & (uint8_t)utf8string[3])
      ));
    utf8string += 4;
    len -= 4;
  }
  else {
    return vds::make_unexpected<std::runtime_error>("Invalid UTF8 string");
  }

  return result;
}

vds::expected<void> vds::utf8::add(std::string& result, wchar_t ch)
{
  if (ch < 0x80) {
    result += (char)ch;
  }
  else if (ch < 0x800) {
    result += (char)(0b11000000 | (ch >> 6));
    result += (char)(0b10000000 | (0b00111111 & ch));
  }
  else if (ch < 0x10000) {
    result += (char)(0b11000000 | (ch >> 12));
    result += (char)(0b10000000 | (0b00111111 & (ch >> 6)));
    result += (char)(0b10000000 | (0b00111111 & ch));
  }
  else if (ch < 0x110000) {
    result += (char)(0b11000000 | (ch >> 18));
    result += (char)(0b10000000 | (0b00111111 & (ch >> 12)));
    result += (char)(0b10000000 | (0b00111111 & (ch >> 6)));
    result += (char)(0b10000000 | (0b00111111 & ch));
  }
  else {
    return vds::make_unexpected<std::runtime_error>("Invalid UTF16 string");
  }

  return expected<void>();
}


static const char encodeLookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static const char padCharacter = '=';
std::string vds::base64::from_bytes(const void * _data, size_t len)
{
  const uint8_t * data = reinterpret_cast<const uint8_t *>(_data);
  std::string encodedString;
  encodedString.reserve(((len/3) + ((len % 3 > 0) ? 1 : 0)) * 4);
  uint32_t temp;
  
  for(; 2 < len; len -= 3) {
    temp  = (*data++) << 16;
    temp += (*data++) << 8;
    temp += (*data++);
    encodedString.append(1,encodeLookup[(temp & 0x00FC0000) >> 18]);
    encodedString.append(1,encodeLookup[(temp & 0x0003F000) >> 12]);
    encodedString.append(1,encodeLookup[(temp & 0x00000FC0) >> 6 ]);
    encodedString.append(1,encodeLookup[(temp & 0x0000003F)      ]);
  }
  
  switch(len)
  {
  case 1:
    temp  = *data << 16;
    encodedString.append(1,encodeLookup[(temp & 0x00FC0000) >> 18]);
    encodedString.append(1,encodeLookup[(temp & 0x0003F000) >> 12]);
    encodedString.append(2,padCharacter);
    break;
  case 2:
    temp  = (*data++) << 16;
    temp += *data << 8;
    encodedString.append(1,encodeLookup[(temp & 0x00FC0000) >> 18]);
    encodedString.append(1,encodeLookup[(temp & 0x0003F000) >> 12]);
    encodedString.append(1,encodeLookup[(temp & 0x00000FC0) >> 6 ]);
    encodedString.append(1,padCharacter);
    break;
  }
  
  return encodedString;
}

std::string vds::base64::from_bytes(const const_data_buffer & data)
{
  return from_bytes(data.data(), data.size());
}

vds::expected<vds::const_data_buffer> vds::base64::to_bytes(const std::string& data)
{
  if (data.length() % 4){
    return vds::make_unexpected<std::runtime_error>("Non-Valid base64!");
  }
  
  size_t padding = 0;
  if (0 < data.length() && data[data.length()-1] == padCharacter){
    padding++;
    
    if (1 < data.length() && data[data.length()-2] == padCharacter){
      padding++;
    }
  }
  
  const_data_buffer result;
  result.resize(((data.length()/4)*3) - padding);
  
  uint32_t temp=0;
  size_t offset = 0;
  size_t quantumPosition = 0;
  for(auto ch : data) {
    temp <<= 6;
    if (ch >= 0x41 && ch <= 0x5A) {
      temp |= ch - 0x41;
    }
    else if  (ch >= 0x61 && ch <= 0x7A) {
      temp |= ch - 0x47;
    }
    else if  (ch >= 0x30 && ch <= 0x39) {
      temp |= ch + 0x04;
    }
    else if  (ch == 0x2B) {
      temp |= 0x3E; //change to 0x2D for URL alphabet
    }
    else if  (ch == 0x2F) {
      temp |= 0x3F; //change to 0x5F for URL alphabet
    }
    else if  (ch == padCharacter) {
      switch(padding) {
      case 1: //One pad character
        result[offset++] = (temp >> 16) & 0x000000FF;
        result[offset] = (temp >> 8 ) & 0x000000FF;
        return const_data_buffer(result);
      case 2: //Two pad characters
        result[offset] = (temp >> 10) & 0x000000FF;
        return const_data_buffer(result);
      default:
        return vds::make_unexpected<std::runtime_error>("Invalid Padding in Base 64!");
      }
    }
    else {
      return vds::make_unexpected<std::runtime_error>("Non-Valid Character in Base 64!");
    }
    
    if(4 == ++quantumPosition) {
      result[offset++] = (temp >> 16) & 0x000000FF;
      result[offset++] = (temp >> 8 ) & 0x000000FF;
      result[offset++] = (temp      ) & 0x000000FF;
      quantumPosition = 0;
    }
  }

  vds_assert(offset == ((data.length() / 4) * 3) - padding);
  vds_assert(result.size() == ((data.length() / 4) * 3) - padding);

  return result;
}

std::string vds::url_encode::encode(const std::string& original) {
  std::string escaped;

  for (char ch : original) {
    if (isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      escaped += ch;
      continue;
    }

    escaped += '%';
    if (10 <= (ch >> 4)) {
      escaped += 'A' + (ch >> 4) - 10;
    }
    else {
      escaped += '0' + (ch >> 4);
    }

    if (10 <= (ch & 0x0F)) {
      escaped += 'A' + (ch & 0x0F) - 10;
    }
    else {
      escaped += '0' + (ch & 0x0F);
    }
  }

  return escaped;
}

std::string vds::url_encode::decode(const std::string& original) {
  std::string decoded;

  auto state = 0;
  char val;
  for (char ch : original) {
    switch (state) {
    case 0: {
      if (ch == '%') {
        state = 1;
      }
      else {
        decoded += ch;
      }
      break;
    }

    case 1: {
      if ('0' <= ch && ch <= '9') {
        val = (ch - '0');
        state = 2;
      }
      else if ('a' <= ch && ch <= 'f') {
        val = (ch - 'a' + 10);
        state = 2;
      }
      else if ('A' <= ch && ch <= 'F') {
        val = (ch - 'A' + 10);
        state = 2;
      }
      else {
        decoded += '%';
        decoded += ch;

        state = 0;
      }
      break;
    }

    case 2: {
      if ('0' <= ch && ch <= '9') {
        decoded += ((val << 4) | (ch - '0'));
        state = 0;
      }
      else if ('a' <= ch && ch <= 'f') {
        decoded += ((val << 4) | (ch - 'a' + 10));
        state = 0;
      }
      else if ('A' <= ch && ch <= 'F') {
        decoded += ((val << 4) | (ch - 'A' + 10));
        state = 0;
      }
      else {
        decoded += '%';
        decoded += ch;

        state = 0;
      }
      break;
    }
    }
  }
  return decoded;
}

//static std::string url_unescape(const std::string & string) {
//  std::string result;
//  int state = 0;
//  char next_char;
//  for (auto ch : string) {
//    switch (state) {
//    case 0:
//      if ('%' == ch) {
//        state = 1;
//        next_char = 0;
//      }
//      else {
//        result += ch;
//      }
//      break;
//    case 1:
//      if (ch >= '0' && ch <= '9') {
//        next_char = (ch - '0') << 4;
//        state = 2;
//      }
//      else if (ch >= 'a' && ch <= 'f') {
//        next_char = (ch - 'a' + 10) << 4;
//        state = 2;
//      }
//      else if (ch >= 'A' && ch <= 'F') {
//        next_char = (ch - 'A' + 10) << 4;
//        state = 2;
//      }
//      else {
//        return vds::make_unexpected<std::runtime_error>("Invalid string");
//      }
//      break;
//    case 2:
//      if (ch >= '0' && ch <= '9') {
//        result += (char)(next_char | (ch - '0'));
//        state = 0;
//      }
//      else if (ch >= 'a' && ch <= 'f') {
//        result += (char)(next_char | (ch - 'a' + 10));
//        state = 0;
//      }
//      else if (ch >= 'A' && ch <= 'F') {
//        result += (char)(next_char | (ch - 'A' + 10));
//        state = 0;
//      }
//      else {
//        return vds::make_unexpected<std::runtime_error>("Invalid string");
//      }
//      break;
//    }
//  }
//
//  return result;
//}
//

std::string vds::hex::from_bytes(const void * d, size_t len)
{
  auto data = static_cast<const uint8_t *>(d);
  static const char hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

  auto result = new char[2 * len];
  for(size_t i = 0; i < len; ++i) {
    const uint8_t byte = *data++;

    result[2 * i] = hex_chars[(byte & 0xF0) >> 4];
    result[2 * i + 1] = hex_chars[(byte & 0x0F) >> 0];
  }
  std::string ret(result, 2 * len);
  delete[] result;
  return ret;
}

std::string vds::hex::from_bytes(const const_data_buffer & data)
{
  return from_bytes(data.data(), data.size());
}

vds::expected<vds::const_data_buffer> vds::hex::to_bytes(const std::string & data)
{
  resizable_data_buffer result;
  bool is_first = true;
  uint8_t value = 0;
  for (auto & ch : data) {
    uint8_t v;
    if ('0' <= ch && ch <= '9') {
      v = ch - '0';
    }
    else if ('a' <= ch && ch <= 'f') {
        v = ch - 'a' + 10;
    }
    else if ('A' <= ch && ch <= 'F') {
      v = ch - 'A' + 10;
    }
    else {
      return make_unexpected<std::runtime_error>("Invalid hex string");
    }

    if (is_first) {
      value = v;
      is_first = false;
    }
    else {
      CHECK_EXPECTED(result.add((value << 4) | v));
      is_first = true;
    }
  }

  if (!is_first) {
    return make_unexpected<std::runtime_error>("Invalid hex string");
  }

  return result.move_data();
}
