#if defined(C3HT_EU_BUILD)
#include "eu_internal.h"
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <mbedtls/platform_util.h>
#include <esp_system.h>

static const char CROCKFORD[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";
static const uint8_t HKDF_SALT[] = {'C','3','H','T','-','E','U','-','v','1'};

void euSha256(const uint8_t *data, size_t len, uint8_t out[32])
{
    mbedtls_sha256_ret(data, len, out, 0);
}

void euHmacSha256(const uint8_t *key, size_t klen, const uint8_t *msg, size_t mlen, uint8_t out[32])
{
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, klen, msg, mlen, out);
}

void euHkdf(const uint8_t *ikm, size_t ikmlen, const char *info, uint8_t *out, size_t outlen)
{
    // RFC 5869 with L <= HashLen: OKM = T(1) = HMAC(PRK, info || 0x01)
    uint8_t prk[32];
    uint8_t t[32];
    uint8_t buf[64];
    size_t ilen = strlen(info);
    if (outlen > 32 || ilen > sizeof(buf) - 1) {
        memset(out, 0, outlen);
        return;
    }
    euHmacSha256(HKDF_SALT, sizeof(HKDF_SALT), ikm, ikmlen, prk);
    memcpy(buf, info, ilen);
    buf[ilen] = 0x01;
    euHmacSha256(prk, sizeof(prk), buf, ilen + 1, t);
    memcpy(out, t, outlen);
    euWipe(prk, sizeof(prk));
    euWipe(t, sizeof(t));
}

void euCrockford(const uint8_t *data, size_t datalen, int nchars, char *out)
{
    // bit i (0 = MSB of data[0])
    for (int c = 0; c < nchars; c++) {
        unsigned v = 0;
        for (int b = 0; b < 5; b++) {
            size_t bit = (size_t)c * 5 + b;
            unsigned x = (bit / 8 < datalen) ? (data[bit / 8] >> (7 - (bit % 8))) & 1 : 0;
            v = (v << 1) | x;
        }
        out[c] = CROCKFORD[v];
    }
    out[nchars] = 0;
}

bool euCtEqual(const void *a, const void *b, size_t len)
{
    const volatile uint8_t *x = (const volatile uint8_t *)a;
    const volatile uint8_t *y = (const volatile uint8_t *)b;
    uint8_t d = 0;
    for (size_t i = 0; i < len; i++) d |= x[i] ^ y[i];
    return d == 0;
}

uint16_t euCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

void euRandom(void *buf, size_t len)
{
    // TRNG: valid because the RF subsystem is running whenever secrets are made
    // (AP mode or ESP-NOW mode). See DESIGN.md section 2.4.
    esp_fill_random(buf, len);
}

void euWipe(void *buf, size_t len)
{
    mbedtls_platform_zeroize(buf, len);
}

void euHex(const uint8_t *in, size_t len, char *out)
{
    static const char H[] = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
        out[2 * i] = H[in[i] >> 4];
        out[2 * i + 1] = H[in[i] & 15];
    }
    out[2 * len] = 0;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool euUnhex(const char *in, uint8_t *out, size_t len)
{
    if (strlen(in) != 2 * len) return false;
    for (size_t i = 0; i < len; i++) {
        int h = hexval(in[2 * i]), l = hexval(in[2 * i + 1]);
        if (h < 0 || l < 0) return false;
        out[i] = (uint8_t)(h << 4 | l);
    }
    return true;
}
#endif
