#include "libs/fec/fec_rs.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

// --- Проверка на всех комбинациях стираний до P включительно ---
template <std::size_t K, std::size_t P>
void run_exhaustive_erasures(std::mt19937_64& rng) {
  constexpr std::size_t window = K + P;
  uint8_t data[K][fec::kBlockBytes]{};
  uint8_t parity[P][fec::kBlockBytes]{};
  uint8_t buffer[window][fec::kBlockBytes]{};
  uint8_t decoded[K][fec::kBlockBytes]{};

  for (std::size_t block = 0; block < K; ++block) {
    for (std::size_t byte = 0; byte < fec::kBlockBytes; ++byte) {
      data[block][byte] = static_cast<uint8_t>(rng() & 0xFFU);
    }
  }

  fec::encode_window<K, P>(data, parity);

  for (std::size_t i = 0; i < K; ++i) {
    std::copy(std::begin(data[i]), std::end(data[i]), std::begin(buffer[i]));
  }
  for (std::size_t i = 0; i < P; ++i) {
    std::copy(std::begin(parity[i]), std::end(parity[i]), std::begin(buffer[K + i]));
  }

  std::vector<int> indices(window);
  std::iota(indices.begin(), indices.end(), 0);

  std::vector<int> current;
  current.reserve(P);

  std::function<void(std::size_t, std::size_t)> dfs = [&](std::size_t start, std::size_t remaining) {
    if (remaining == 0) {
      bool present[window];
      std::fill(std::begin(present), std::end(present), true);
      for (int idx : current) {
        present[idx] = false;
      }
      const bool ok = fec::decode_window<K, P>(present, buffer, decoded);
      if (!ok) {
        throw std::runtime_error("Не удалось восстановить данные при допустимом числе стираний");
      }
      for (std::size_t block = 0; block < K; ++block) {
        for (std::size_t byte = 0; byte < fec::kBlockBytes; ++byte) {
          if (decoded[block][byte] != data[block][byte]) {
            throw std::runtime_error("Неверно восстановлен байт при стираниях");
          }
        }
      }
      return;
    }
    for (std::size_t i = start; i <= window - remaining; ++i) {
      current.push_back(indices[i]);
      dfs(i + 1, remaining - 1);
      current.pop_back();
    }
  };

  for (std::size_t erased = 0; erased <= P; ++erased) {
    dfs(0, erased);
  }
}

// --- Проверка бурстовых стираний длиной до D = K/2 ---
template <std::size_t K, std::size_t P>
void run_burst_test(std::mt19937_64& rng) {
  constexpr std::size_t window = K + P;
  constexpr std::size_t depth = (K / 2 < P) ? (K / 2) : P;
  uint8_t data[K][fec::kBlockBytes]{};
  uint8_t parity[P][fec::kBlockBytes]{};
  uint8_t buffer[window][fec::kBlockBytes]{};
  uint8_t decoded[K][fec::kBlockBytes]{};

  for (std::size_t block = 0; block < K; ++block) {
    for (std::size_t byte = 0; byte < fec::kBlockBytes; ++byte) {
      data[block][byte] = static_cast<uint8_t>(rng() & 0xFFU);
    }
  }

  fec::encode_window<K, P>(data, parity);

  for (std::size_t i = 0; i < K; ++i) {
    std::copy(std::begin(data[i]), std::end(data[i]), std::begin(buffer[i]));
  }
  for (std::size_t i = 0; i < P; ++i) {
    std::copy(std::begin(parity[i]), std::end(parity[i]), std::begin(buffer[K + i]));
  }

  for (std::size_t start = 0; start + depth <= window; ++start) {
    bool present[window];
    std::fill(std::begin(present), std::end(present), true);
    for (std::size_t offset = 0; offset < depth; ++offset) {
      present[start + offset] = false;
    }
    const bool ok = fec::decode_window<K, P>(present, buffer, decoded);
    if (!ok) {
      throw std::runtime_error("Бурстовая ошибка допустимой длины не восстановлена");
    }
    for (std::size_t block = 0; block < K; ++block) {
      for (std::size_t byte = 0; byte < fec::kBlockBytes; ++byte) {
        if (decoded[block][byte] != data[block][byte]) {
          throw std::runtime_error("Бурстовая ошибка дала неверный байт");
        }
      }
    }
  }
}

} // namespace

int main() {
  try {
    std::mt19937_64 rng(0x12345678ULL);
    run_exhaustive_erasures<8, 2>(rng);
    run_exhaustive_erasures<8, 4>(rng);
    run_exhaustive_erasures<10, 6>(rng);
    run_burst_test<8, 2>(rng);
    run_burst_test<8, 4>(rng);
    run_burst_test<10, 6>(rng);
    std::cout << "Все тесты RS успешно пройдены" << std::endl;
  } catch (const std::exception& ex) {
    std::cerr << "Ошибка теста: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
