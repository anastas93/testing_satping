#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fec {

// --- Константы поля GF(2^8) ---
constexpr uint16_t kPrimitivePolynomial = 0x11D; // x^8 + x^4 + x^3 + x^2 + 1
constexpr uint8_t kPrimitiveElement = 0x02;      // примитивный элемент α
constexpr std::size_t kBlockBytes = 8;           // размер пакета данных/паритета в байтах

// --- Таблицы логарифмов/антилогарифмов ---
struct GfTables {
  std::array<uint8_t, 512> alog{}; // α^i
  std::array<uint8_t, 256> log{};  // log_α(x)
};

// --- Доступ к таблицам поля ---
const GfTables& gf_tables();

// --- Математика поля ---
uint8_t gf_mul(uint8_t a, uint8_t b);
uint8_t gf_inv(uint8_t a);

// --- Шаблонные функции кодирования/декодирования RS-окна ---
template <std::size_t K, std::size_t P>
void encode_window(const uint8_t (&data)[K][kBlockBytes], uint8_t (&parity)[P][kBlockBytes]);

template <std::size_t K, std::size_t P>
bool decode_window(const bool (&present)[K + P], const uint8_t (&in)[K + P][kBlockBytes],
                   uint8_t (&out)[K][kBlockBytes]);

} // namespace fec

#include "fec_rs.tpp"
