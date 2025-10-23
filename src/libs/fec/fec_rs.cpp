#include "fec_rs.h"

#include <algorithm>

namespace fec {
namespace {

GfTables BuildTables() {
  GfTables tables{};
  tables.alog[0] = 1;
  for (std::size_t i = 1; i < 255; ++i) {
    uint16_t value = static_cast<uint16_t>(tables.alog[i - 1]) << 1U;
    if (value & 0x100U) {
      value ^= kPrimitivePolynomial;
    }
    tables.alog[i] = static_cast<uint8_t>(value);
  }
  for (std::size_t i = 255; i < tables.alog.size(); ++i) {
    tables.alog[i] = tables.alog[i - 255];
  }
  tables.log[0] = 0;
  for (std::size_t i = 0; i < 255; ++i) {
    tables.log[tables.alog[i]] = static_cast<uint8_t>(i);
  }
  return tables;
}

const GfTables& TablesInstance() {
  static const GfTables tables = BuildTables();
  return tables;
}

} // namespace

const GfTables& gf_tables() {
  return TablesInstance();
}

uint8_t gf_mul(uint8_t a, uint8_t b) {
  if (a == 0U || b == 0U) {
    return 0U;
  }
  const auto& tables = gf_tables();
  const uint16_t sum = static_cast<uint16_t>(tables.log[a]) + tables.log[b];
  return tables.alog[sum];
}

uint8_t gf_inv(uint8_t a) {
  if (a == 0U) {
    return 0U;
  }
  const auto& tables = gf_tables();
  const uint16_t power = 255U - tables.log[a];
  return tables.alog[power];
}

} // namespace fec
