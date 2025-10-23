#include "Utils.h"

// Use hardware-accelerated crypto on ESP32-S3 when available
#if defined(ESP32) && defined(USE_HW_CRYPTO)
  #include "mbedtls/aes.h"
  #include "mbedtls/sha256.h"
  #include "mbedtls/md.h"
  #define USING_HW_CRYPTO 1
#else
  #include <AES.h>
  #include <SHA256.h>
  #define USING_HW_CRYPTO 0
#endif

#ifdef ARDUINO
  #include <Arduino.h>
#endif

namespace mesh {

uint32_t RNG::nextInt(uint32_t _min, uint32_t _max) {
  uint32_t num;
  random((uint8_t *) &num, sizeof(num));
  return (num % (_max - _min)) + _min;
}

void Utils::sha256(uint8_t *hash, size_t hash_len, const uint8_t* msg, int msg_len) {
#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated SHA256
  uint8_t full_hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0); 
  mbedtls_sha256_update(&ctx, msg, msg_len);
  mbedtls_sha256_finish(&ctx, full_hash);
  mbedtls_sha256_free(&ctx);
  memcpy(hash, full_hash, hash_len > 32 ? 32 : hash_len);
#else
  // fallback
  SHA256 sha;
  sha.update(msg, msg_len);
  sha.finalize(hash, hash_len);
#endif
}

void Utils::sha256(uint8_t *hash, size_t hash_len, const uint8_t* frag1, int frag1_len, const uint8_t* frag2, int frag2_len) {
#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated SHA256
  uint8_t full_hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0); 
  mbedtls_sha256_update(&ctx, frag1, frag1_len);
  mbedtls_sha256_update(&ctx, frag2, frag2_len);
  mbedtls_sha256_finish(&ctx, full_hash);
  mbedtls_sha256_free(&ctx);
  memcpy(hash, full_hash, hash_len > 32 ? 32 : hash_len);
#else
  // fallback
  SHA256 sha;
  sha.update(frag1, frag1_len);
  sha.update(frag2, frag2_len);
  sha.finalize(hash, hash_len);
#endif
}

int Utils::decrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated AES-128 ECB decryption
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, shared_secret, 128);  
  
  uint8_t* dp = dest;
  const uint8_t* sp = src;
  while (sp - src < src_len) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, sp, dp);
    dp += 16; sp += 16;
  }
  
  mbedtls_aes_free(&aes);
  return sp - src;  
#else
  // Software fallback
  AES128 aes;
  uint8_t* dp = dest;
  const uint8_t* sp = src;

  aes.setKey(shared_secret, CIPHER_KEY_SIZE);
  while (sp - src < src_len) {
    aes.decryptBlock(dp, sp);
    dp += 16; sp += 16;
  }

  return sp - src;  // will always be multiple of 16
#endif
}

int Utils::encrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated AES-128 ECB encryption
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, shared_secret, 128);  // 128-bit key
  
  uint8_t* dp = dest;
  while (src_len >= 16) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, src, dp);
    dp += 16; src += 16; src_len -= 16;
  }
  if (src_len > 0) { 
    uint8_t tmp[16];
    memset(tmp, 0, 16);
    memcpy(tmp, src, src_len);
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, tmp, dp);
    dp += 16;
  }
  
  mbedtls_aes_free(&aes);
  return dp - dest;  
#else
  // Software fallback
  AES128 aes;
  uint8_t* dp = dest;

  aes.setKey(shared_secret, CIPHER_KEY_SIZE);
  while (src_len >= 16) {
    aes.encryptBlock(dp, src);
    dp += 16; src += 16; src_len -= 16;
  }
  if (src_len > 0) {  // remaining partial block
    uint8_t tmp[16];
    memset(tmp, 0, 16);
    memcpy(tmp, src, src_len);
    aes.encryptBlock(dp, tmp);
    dp += 16;
  }
  return dp - dest;  // will always be multiple of 16
#endif
}

int Utils::encryptThenMAC(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
  int enc_len = encrypt(shared_secret, dest + CIPHER_MAC_SIZE, src, src_len);

#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated HMAC-SHA256
  uint8_t full_hmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);  
  mbedtls_md_hmac_starts(&ctx, shared_secret, PUB_KEY_SIZE);  // Initialize HMAC with key
  mbedtls_md_hmac_update(&ctx, dest + CIPHER_MAC_SIZE, enc_len);
  mbedtls_md_hmac_finish(&ctx, full_hmac);
  mbedtls_md_free(&ctx);
  memcpy(dest, full_hmac, CIPHER_MAC_SIZE);
#else
  // Software fallback
  SHA256 sha;
  sha.resetHMAC(shared_secret, PUB_KEY_SIZE);
  sha.update(dest + CIPHER_MAC_SIZE, enc_len);
  sha.finalizeHMAC(shared_secret, PUB_KEY_SIZE, dest, CIPHER_MAC_SIZE);
#endif

  return CIPHER_MAC_SIZE + enc_len;
}

int Utils::MACThenDecrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
  if (src_len <= CIPHER_MAC_SIZE) return 0;  // invalid src bytes

  uint8_t hmac[CIPHER_MAC_SIZE];
#if USING_HW_CRYPTO
  // ESP32-S3 hardware-accelerated HMAC-SHA256
  uint8_t full_hmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);  // 1 = HMAC
  mbedtls_md_hmac_starts(&ctx, shared_secret, PUB_KEY_SIZE);
  mbedtls_md_hmac_update(&ctx, src + CIPHER_MAC_SIZE, src_len - CIPHER_MAC_SIZE);
  mbedtls_md_hmac_finish(&ctx, full_hmac);
  mbedtls_md_free(&ctx);
  memcpy(hmac, full_hmac, CIPHER_MAC_SIZE);
#else
  // Software fallback
  {
    SHA256 sha;
    sha.resetHMAC(shared_secret, PUB_KEY_SIZE);
    sha.update(src + CIPHER_MAC_SIZE, src_len - CIPHER_MAC_SIZE);
    sha.finalizeHMAC(shared_secret, PUB_KEY_SIZE, hmac, CIPHER_MAC_SIZE);
  }
#endif
  
  if (memcmp(hmac, src, CIPHER_MAC_SIZE) == 0) {
    return decrypt(shared_secret, dest, src + CIPHER_MAC_SIZE, src_len - CIPHER_MAC_SIZE);
  }
  return 0; // invalid HMAC
}

static const char hex_chars[] = "0123456789ABCDEF";

void Utils::toHex(char* dest, const uint8_t* src, size_t len) {
  while (len > 0) {
    uint8_t b = *src++;
    *dest++ = hex_chars[b >> 4];
    *dest++ = hex_chars[b & 0x0F];
    len--;
  }
  *dest = 0;
}

void Utils::printHex(Stream& s, const uint8_t* src, size_t len) {
  while (len > 0) {
    uint8_t b = *src++;
    s.print(hex_chars[b >> 4]);
    s.print(hex_chars[b & 0x0F]);
    len--;
  }
}

static uint8_t hexVal(char c) {
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= '0' && c <= '9') return c - '0';
  return 0;
}

bool Utils::isHexChar(char c) {
  return c == '0' || hexVal(c) > 0;
}

bool Utils::fromHex(uint8_t* dest, int dest_size, const char *src_hex) {
  int len = strlen(src_hex);
  if (len != dest_size*2) return false;  // incorrect length

  uint8_t* dp = dest;
  while (dp - dest < dest_size) {
    char ch = *src_hex++;
    char cl = *src_hex++;
    *dp++ = (hexVal(ch) << 4) | hexVal(cl);
  }
  return true;
}

int Utils::parseTextParts(char* text, const char* parts[], int max_num, char separator) {
  int num = 0;
  char* sp = text;
  while (*sp && num < max_num) {
    parts[num++] = sp;
    while (*sp && *sp != separator) sp++;
    if (*sp) {
       *sp++ = 0;  // replace the seperator with a null, and skip past it
    }
  }
  // if we hit the maximum parts, make sure LAST entry does NOT have separator 
  while (*sp && *sp != separator) sp++;
  if (*sp) {
    *sp = 0;  // replace the separator with null
  }
  return num;
}

}