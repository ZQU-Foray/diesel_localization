// Copyright 2026 ZQU-Foray

#include "diesel/localization/sha256.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace diesel::localization {
namespace {

constexpr std::array<std::uint32_t, 64> kRoundConstants = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U,
    0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU,
    0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU,
    0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U,
    0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U,
    0xc67178f2U,
};

std::uint32_t rotate_right(std::uint32_t value, std::uint32_t count) {
    return (value >> count) | (value << (32U - count));
}

void transform(std::array<std::uint32_t, 8>& state, const std::uint8_t* block) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index) {
        words[index] = (static_cast<std::uint32_t>(block[index * 4]) << 24U) |
                       (static_cast<std::uint32_t>(block[index * 4 + 1]) << 16U) |
                       (static_cast<std::uint32_t>(block[index * 4 + 2]) << 8U) |
                       static_cast<std::uint32_t>(block[index * 4 + 3]);
    }
    for (std::size_t index = 16; index < 64; ++index) {
        const std::uint32_t s0 = rotate_right(words[index - 15], 7U) ^
                                 rotate_right(words[index - 15], 18U) ^ (words[index - 15] >> 3U);
        const std::uint32_t s1 = rotate_right(words[index - 2], 17U) ^
                                 rotate_right(words[index - 2], 19U) ^ (words[index - 2] >> 10U);
        words[index] = words[index - 16] + s0 + words[index - 7] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];

    for (std::size_t index = 0; index < 64; ++index) {
        const std::uint32_t sigma1 =
            rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
        const std::uint32_t choose = (e & f) ^ ((~e) & g);
        const std::uint32_t temp1 = h + sigma1 + choose + kRoundConstants[index] + words[index];
        const std::uint32_t sigma0 =
            rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
        const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t temp2 = sigma0 + majority;

        h = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

class Sha256Context {
  public:
    Sha256Context()
        : state_{{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                  0x1f83d9abU, 0x5be0cd19U}} {}

    void update(const std::uint8_t* data, std::size_t size) {
        total_bytes_ += size;
        while (size > 0) {
            const std::size_t copy_size = std::min(size, block_.size() - block_size_);
            std::memcpy(block_.data() + block_size_, data, copy_size);
            block_size_ += copy_size;
            data += copy_size;
            size -= copy_size;
            if (block_size_ == block_.size()) {
                transform(state_, block_.data());
                block_size_ = 0;
            }
        }
    }

    std::string final_hex() {
        const std::uint64_t total_bits = total_bytes_ * 8U;
        block_[block_size_++] = 0x80U;
        if (block_size_ > 56) {
            while (block_size_ < 64) {
                block_[block_size_++] = 0U;
            }
            transform(state_, block_.data());
            block_size_ = 0;
        }
        while (block_size_ < 56) {
            block_[block_size_++] = 0U;
        }
        for (std::size_t index = 0; index < 8; ++index) {
            block_[63 - index] = static_cast<std::uint8_t>((total_bits >> (8U * index)) & 0xffU);
        }
        transform(state_, block_.data());

        std::ostringstream stream;
        stream << std::hex << std::setfill('0');
        for (const std::uint32_t word : state_) {
            stream << std::setw(8) << word;
        }
        return stream.str();
    }

  private:
    std::array<std::uint32_t, 8> state_;
    std::array<std::uint8_t, 64> block_{};
    std::size_t block_size_{0};
    std::uint64_t total_bytes_{0};
};

} // namespace

std::string sha256_hex(const void* data, std::size_t size) {
    Sha256Context context;
    context.update(static_cast<const std::uint8_t*>(data), size);
    return context.final_hex();
}

std::string sha256_file(const std::filesystem::path& path, std::string* error) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        if (error != nullptr) {
            *error = "cannot open file for sha256: " + path.string();
        }
        return {};
    }

    Sha256Context context;
    std::vector<char> buffer(64 * 1024);
    while (stream) {
        stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize read_size = stream.gcount();
        if (read_size > 0) {
            context.update(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                           static_cast<std::size_t>(read_size));
        }
    }
    if (stream.bad()) {
        if (error != nullptr) {
            *error = "failed while reading file for sha256: " + path.string();
        }
        return {};
    }
    return context.final_hex();
}

} // namespace diesel::localization
