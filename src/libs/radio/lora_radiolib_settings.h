#pragma once

#include <cstdint>

namespace LoRaRadioLibSettings {

// Структура описывает параметры драйвера SX1262, управляемые через RadioLib.
// Для каждого поля приведено краткое описание его назначения.
struct SX1262DriverOptions {
  float bandwidthKhz = 15.63f;        // Ширина полосы пропускания (кГц).
                                      // Меньшая полоса повышает чувствительность и увеличивает время передачи.
  uint8_t spreadingFactor = 7;        // Фактор расширения SF (5..12).
                                      // Увеличение повышает устойчивость, но снижает скорость.
  uint8_t codingRateDenom = 5;        // Делитель коэффициента кодирования CR (5..8).
                                      // CR=4/denom, задаёт избыточность и помехоустойчивость.
  int8_t lowPowerDbm = -5;            // Низкий уровень мощности передачи (dBm) для щадящего режима.
  int8_t highPowerDbm = 22;           // Высокий уровень мощности передачи (dBm) для максимальной дальности.
  bool useDio2AsRfSwitch = true;      // Включение аппаратного RF-переключателя на выводе DIO2.

  // Включение TCXO через DIO3 и его напряжение можно переопределить макросами сборки:
  // -DLOTEST_TCXO=1 и -DLOTEST_TCXO_VOLTAGE=1.8f в platformio.ini
#ifdef LOTEST_TCXO
  bool useDio3ForTcxo = (LOTEST_TCXO != 0);
#else
  bool useDio3ForTcxo = false;        // по умолчанию TCXO не используется
#endif
#ifdef LOTEST_TCXO_VOLTAGE
  float tcxoVoltage = LOTEST_TCXO_VOLTAGE;
#else
  float tcxoVoltage = 1.8f;           // Типовое напряжение TCXO (В). Уточните по даташиту модуля (1.6–3.0 В).
#endif
  // Задержка прогрева TCXO (мкс). По умолчанию 5000. Можно переопределить -DLOTEST_TCXO_DELAY_US=7500
#ifdef LOTEST_TCXO_DELAY_US
  uint32_t tcxoDelayUs = static_cast<uint32_t>(LOTEST_TCXO_DELAY_US);
#else
  uint32_t tcxoDelayUs = 5000U;
#endif
  bool enableRegulatorLDO = true;     // Принудительный перевод радиочасти в режим LDO-регулятора.
  bool enableRegulatorDCDC = false;   // Принудительный перевод радиочасти в режим DC-DC (если поддерживается аппаратно).
  bool autoLdro = true;               // Автоматический выбор оптимизации для низких скоростей передачи (LDRO).
  bool implicitHeader = true;         // Использовать фиксированный размер пакета (implicit header) вместо стандартного заголовка.
  uint8_t implicitPayloadLength = 8;  // Размер полезной нагрузки при implicit header (байты).
  bool enableCrc = false;             // Добавлять ли аппаратный CRC в конец LoRa-пакета.
  bool invertIq = false;              // Инверсия фаз (I/Q) для совместимости с определёнными сетями.
  bool publicNetwork = true;          // Использовать стандартное публичное синхрослово LoRa (true) либо приватное (false).
  uint16_t syncWord = 0x34;           // Значение синхрослова LoRa (8 или 16 бит в зависимости от режима).
  // Длина преамбулы в символах. Для модулей с TCXO рекомендуется большее значение
  // для прогрева/стабилизации: по умолчанию 12, с TCXO — 24. Можно переопределить
  // конкретным значением через -DLOTEST_PREAMBLE=NN в platformio.ini.
#ifdef LOTEST_PREAMBLE
  uint16_t preambleLength = LOTEST_PREAMBLE;
#else
#  ifdef LOTEST_TCXO
  uint16_t preambleLength = 24;
#  else
  uint16_t preambleLength = 12;
#  endif
#endif
  bool rxBoostedGain = true;          // Режим усиленного каскада LNA при приёме для повышения чувствительности.
};

// Набор параметров по умолчанию, применяемый при инициализации радиомодуля.
// Имя константы содержит суффикс OPTIONS, чтобы избежать конфликта с макросами Arduino (DEFAULT).
#if __cplusplus >= 201703L
inline constexpr SX1262DriverOptions DEFAULT_OPTIONS{};
inline constexpr bool DEFAULT_RX_BOOSTED_GAIN = DEFAULT_OPTIONS.rxBoostedGain;         // Режим усиленного приёма
inline constexpr uint16_t DEFAULT_PREAMBLE_LENGTH = DEFAULT_OPTIONS.preambleLength;    // Длина преамбулы по умолчанию
#else
constexpr SX1262DriverOptions DEFAULT_OPTIONS{};                                       // отдельная копия в каждом переводимом модуле
constexpr bool DEFAULT_RX_BOOSTED_GAIN = DEFAULT_OPTIONS.rxBoostedGain;                // Режим усиленного приёма без inline
constexpr uint16_t DEFAULT_PREAMBLE_LENGTH = DEFAULT_OPTIONS.preambleLength;           // Длина преамбулы по умолчанию без inline
#endif

} // namespace LoRaRadioLibSettings
