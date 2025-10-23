#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace fec {
namespace detail {

inline uint8_t gf_add(uint8_t a, uint8_t b) {
  return static_cast<uint8_t>(a ^ b);
}

// --- Предвычисление генераторной матрицы Вандермонда ---
template <std::size_t K, std::size_t P>
std::array<std::array<uint8_t, K>, K + P> BuildGenerator() {
  std::array<std::array<uint8_t, K>, K + P> matrix{};
  const auto& tables = gf_tables();
  std::array<std::array<uint8_t, K>, K> vandermonde{};
  for (std::size_t row = 0; row < K; ++row) {
    uint8_t value = 1U;
    const uint8_t point = tables.alog[row % 255U];
    for (std::size_t col = 0; col < K; ++col) {
      vandermonde[row][col] = value;
      value = gf_mul(value, point);
    }
  }

  auto mat = vandermonde;
  std::array<std::array<uint8_t, K>, K> inverse{};
  for (std::size_t row = 0; row < K; ++row) {
    inverse[row].fill(0U);
    inverse[row][row] = 1U;
  }

  for (std::size_t col = 0; col < K; ++col) {
    std::size_t pivot = col;
    while (pivot < K && mat[pivot][col] == 0U) {
      ++pivot;
    }
    if (pivot != col) {
      std::swap(mat[pivot], mat[col]);
      std::swap(inverse[pivot], inverse[col]);
    }
    const uint8_t pivotVal = mat[col][col];
    const uint8_t pivotInv = gf_inv(pivotVal);
    for (std::size_t j = 0; j < K; ++j) {
      mat[col][j] = gf_mul(mat[col][j], pivotInv);
      inverse[col][j] = gf_mul(inverse[col][j], pivotInv);
    }
    for (std::size_t row = 0; row < K; ++row) {
      if (row == col) {
        continue;
      }
      const uint8_t factor = mat[row][col];
      if (factor == 0U) {
        continue;
      }
      for (std::size_t j = 0; j < K; ++j) {
        mat[row][j] = gf_add(mat[row][j], gf_mul(factor, mat[col][j]));
        inverse[row][j] = gf_add(inverse[row][j], gf_mul(factor, inverse[col][j]));
      }
    }
  }

  for (std::size_t row = 0; row < K; ++row) {
    matrix[row].fill(0U);
    matrix[row][row] = 1U; // систематическая часть
  }

  for (std::size_t p = 0; p < P; ++p) {
    auto& row = matrix[K + p];
    uint8_t value = 1U;
    const uint8_t point = tables.alog[(K + p) % 255U];
    std::array<uint8_t, K> vandermondeRow{};
    for (std::size_t col = 0; col < K; ++col) {
      vandermondeRow[col] = value;
      value = gf_mul(value, point);
    }
    for (std::size_t col = 0; col < K; ++col) {
      uint8_t acc = 0U;
      for (std::size_t k = 0; k < K; ++k) {
        acc = gf_add(acc, gf_mul(vandermondeRow[k], inverse[k][col]));
      }
      row[col] = acc;
    }
  }
  return matrix;
}

template <std::size_t K, std::size_t P>
const std::array<std::array<uint8_t, K>, K + P>& GeneratorMatrix() {
  static const auto matrix = BuildGenerator<K, P>();
  return matrix;
}

} // namespace detail

template <std::size_t K, std::size_t P>
void encode_window(const uint8_t (&data)[K][kBlockBytes], uint8_t (&parity)[P][kBlockBytes]) {
  const auto& generator = detail::GeneratorMatrix<K, P>();
  for (std::size_t row = 0; row < P; ++row) {
    for (std::size_t byte = 0; byte < kBlockBytes; ++byte) {
      uint8_t acc = 0U;
      for (std::size_t col = 0; col < K; ++col) {
        const uint8_t coeff = generator[K + row][col];
        const uint8_t product = gf_mul(coeff, data[col][byte]);
        acc = detail::gf_add(acc, product);
      }
      parity[row][byte] = acc;
    }
  }
}

template <std::size_t K, std::size_t P>
bool decode_window(const bool (&present)[K + P], const uint8_t (&in)[K + P][kBlockBytes],
                   uint8_t (&out)[K][kBlockBytes]) {
  std::array<std::size_t, K + P> indices{};
  std::size_t count = 0;
  for (std::size_t idx = 0; idx < K + P; ++idx) {
    if (present[idx]) {
      indices[count++] = idx;
    }
  }
  if (count < K) {
    return false; // не хватает уравнений для восстановления
  }

  const auto& generator = detail::GeneratorMatrix<K, P>();

  for (std::size_t byte = 0; byte < kBlockBytes; ++byte) {
    std::array<std::array<uint8_t, K>, K + P> matrix{};
    std::array<uint8_t, K + P> rhs{};
    for (std::size_t row = 0; row < count; ++row) {
      matrix[row] = generator[indices[row]];
      rhs[row] = in[indices[row]][byte];
    }

    std::array<std::size_t, K> pivotColumn{};
    std::size_t rank = 0;

    for (std::size_t col = 0; col < K && rank < K; ++col) {
      std::size_t pivot = rank;
      while (pivot < count && matrix[pivot][col] == 0U) {
        ++pivot;
      }
      if (pivot == count) {
        continue; // в этом столбце нет опорной строки, пробуем следующий
      }
      if (pivot != rank) {
        std::swap(matrix[pivot], matrix[rank]);
        std::swap(rhs[pivot], rhs[rank]);
      }

      const uint8_t pivotValue = matrix[rank][col];
      const uint8_t pivotInv = gf_inv(pivotValue);
      for (std::size_t j = 0; j < K; ++j) {
        matrix[rank][j] = gf_mul(matrix[rank][j], pivotInv);
      }
      rhs[rank] = gf_mul(rhs[rank], pivotInv);

      for (std::size_t row = 0; row < count; ++row) {
        if (row == rank) {
          continue;
        }
        const uint8_t factor = matrix[row][col];
        if (factor == 0U) {
          continue;
        }
        for (std::size_t j = 0; j < K; ++j) {
          matrix[row][j] = detail::gf_add(matrix[row][j], gf_mul(factor, matrix[rank][j]));
        }
        rhs[row] = detail::gf_add(rhs[row], gf_mul(factor, rhs[rank]));
      }

      pivotColumn[rank] = col;
      ++rank;
    }

    if (rank < K) {
      return false; // текущий набор строк не образует полный ранг
    }

    std::array<uint8_t, K> solution{};
    solution.fill(0U);
    for (std::size_t r = 0; r < K; ++r) {
      solution[pivotColumn[r]] = rhs[r];
    }
    for (std::size_t col = 0; col < K; ++col) {
      out[col][byte] = solution[col];
    }
  }

  return true;
}

} // namespace fec
