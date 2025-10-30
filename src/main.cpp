#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RadioLib.h>
#include <vector>
#include <array>
#include <cstddef>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <type_traits>
#include <utility>
#include <map>
#include <deque>
#include <numeric>
#include <cmath>
#include <limits>
#if !defined(ARDUINO)
#include <thread>
#endif

#include "libs/radio/lora_radiolib_settings.h"     // дефолтные настройки драйвера SX1262

// --- Переопределяем пины радиомодуля через макросы (по умолчанию — под ESP32 DevKit V1) ---
#ifndef RADIO_SCK
#  define RADIO_SCK   18   // SCK  (ESP32 DevKit V1 VSPI)
#endif
#ifndef RADIO_MISO
#  define RADIO_MISO  19   // MISO (ESP32 DevKit V1 VSPI)
#endif
#ifndef RADIO_MOSI
#  define RADIO_MOSI  23   // MOSI (ESP32 DevKit V1 VSPI)
#endif
#ifndef RADIO_CS
#  define RADIO_CS     5   // NSS/CS
#endif
#ifndef RADIO_DIO1
#  define RADIO_DIO1  14   // DIO1 (IRQ)
#endif
#ifndef RADIO_RST
#  define RADIO_RST   27   // RESET
#endif
#ifndef RADIO_BUSY
#  define RADIO_BUSY  25   // BUSY
#endif

namespace {
constexpr auto kRadioDefaults = LoRaRadioLibSettings::DEFAULT_OPTIONS; // Статический набор настроек RadioLib
constexpr size_t kImplicitPayloadLength = static_cast<size_t>(kRadioDefaults.implicitPayloadLength); // размер implicit-пакета
constexpr float kDefaultBandwidthKhz = kRadioDefaults.bandwidthKhz;    // полоса пропускания LoRa по умолчанию
constexpr uint8_t kDefaultSpreadingFactor = kRadioDefaults.spreadingFactor; // фактор расширения SF
constexpr uint8_t kDefaultCodingRate = kRadioDefaults.codingRateDenom;      // делитель коэффициента кодирования CR
constexpr int8_t kLowPowerDbm = kRadioDefaults.lowPowerDbm;                 // низкий уровень мощности
constexpr int8_t kHighPowerDbm = kRadioDefaults.highPowerDbm;               // высокий уровень мощности
} // namespace

// --- Константы частот банка HOME ---
namespace frequency_tables {

// Локальный набор частот банка HOME, продублированный здесь, чтобы проект был самодостаточным
static constexpr std::size_t HOME_BANK_SIZE = 14;
static constexpr float RX_HOME[HOME_BANK_SIZE] = {
    250.000F,263.450F, 257.700F, 257.200F, 256.450F, 267.250F, 250.090F, 249.850F,
    257.250F, 255.100F, 246.700F, 260.250F, 263.400F, 263.500F};
static constexpr float TX_HOME[HOME_BANK_SIZE] = {
    250.000F,311.400F, 316.150F, 308.800F, 313.850F, 308.250F, 312.600F, 298.830F,
    316.900F, 318.175F, 297.700F, 314.400F, 316.400F, 315.200F};

} // namespace frequency_tables

// --- Проверка наличия обязательных макросов для Wi-Fi ---
#ifndef LOTEST_WIFI_SSID
#  define LOTEST_WIFI_SSID "sat_ap"
#endif
#ifndef LOTEST_WIFI_PASS
#  define LOTEST_WIFI_PASS "12345678"
#endif
#ifndef LOTEST_PROJECT_NAME
#  define LOTEST_PROJECT_NAME "Lotest"
#endif

// --- Глобальные объекты периферии ---
// На ESP32-C3 нет VSPI/HSPI, используем глобальный SPI; на классическом ESP32 — VSPI.
#if defined(CONFIG_IDF_TARGET_ESP32C3)
SPIClass& radioSPI = SPI;                         // общий SPI для ESP32-C3
#else
SPIClass radioSPI(VSPI);                          // аппаратный SPI-порт, обслуживающий радиомодуль
#endif
SX1262 radio = new Module(RADIO_CS, RADIO_DIO1, RADIO_RST, RADIO_BUSY, radioSPI); // пины берём из макросов
WebServer server(80);                             // встроенный HTTP-сервер ESP32

// --- Константы проекта ---
constexpr uint8_t kHomeBankSize = static_cast<uint8_t>(frequency_tables::HOME_BANK_SIZE); // число каналов банка HOME
constexpr size_t kMaxEventHistory = 120;          // ограничение истории событий для веб-чата
constexpr size_t kFullPacketSize = 245;           // максимальная длина пакета SX1262
constexpr size_t kFixedFrameSize = 8;             // фиксированная длина кадра LoRa
constexpr size_t kFramePayloadSize = 5;           // полезная часть кадра согласно спецификации ARQ
constexpr size_t kPartCountFieldSize = 2;         // число байт, которыми кодируется количество частей
constexpr size_t kPartCountMetadataBytes = kPartCountFieldSize * 2; // метаданные хранятся в первом и последнем пакете
constexpr unsigned long kInterFrameDelayMs = 8;   // пауза между кадрами (снижена для ускорения)
constexpr size_t kArqWindowSize = 8;              // размер окна подтверждения ACK
constexpr size_t kBitmapWidth = 16;               // ширина BITMAP16 для запроса переотправки
constexpr uint16_t kBitmapFullMask = (1U << kBitmapWidth) - 1U; // полный набор битов BITMAP16
constexpr uint8_t kInterleaverStep = 4;           // шаг интерливера для борьбы с бурстами
constexpr size_t kHarqDataBlock = 11;             // число DATA-пакетов в HARQ-блоке RS(15,11)
constexpr size_t kHarqParityCount = 4;            // число PAR-пакетов в HARQ-блоке
constexpr size_t kLongPacketSize = 124;           // длина длинного пакета с буквами A-Z
constexpr const char* kIncomingColor = "#5CE16A"; // цвет отображения принятых сообщений

// --- Константы формата кадров Lotest ---
constexpr uint8_t kFrameTypeMask = 0xC0;          // верхние два бита определяют тип кадра
constexpr uint8_t kFrameTypeData = 0x00;          // DATA-кадр
constexpr uint8_t kFrameTypeAck = 0x40;           // ACK-кадр
constexpr uint8_t kFrameTypeParity = 0x80;        // PAR-кадр (HARQ)
constexpr uint8_t kFrameTypeFin = 0xC0;           // FIN-кадр с итоговым CRC

constexpr uint8_t kDataFlagAckRequest = 0x01;     // запрашивать ACK после окна
constexpr uint8_t kDataFlagIsParity = 0x02;       // полезная нагрузка содержит паритетные данные
constexpr uint8_t kDataFlagLengthShift = 2;       // смещение длины полезной нагрузки
constexpr uint8_t kDataFlagLengthMask = 0x3C;     // маска длины полезной нагрузки (0..15 байт)

constexpr uint8_t kAckFlagNeedParity = 0x01;      // получателю требуется PAR-передача

constexpr uint8_t kAckTypeSuccess = 0x01;         // тип ACK: все пакеты успешно получены
constexpr uint8_t kAckTypeMissing = 0x02;         // тип ACK: присутствуют потери, требуется переотправка

constexpr uint8_t kFinFlagHarqUsed = 0x01;        // в ходе передачи использовался HARQ

// Магическая сигнатура «стартового» кадра управления (DATA с особой полезной нагрузкой)
constexpr std::array<uint8_t, kFramePayloadSize> kStartMagic = {0xA5, 0x5A, 0xC3, 0x3C, 0x7E};

// Кодирование SEQ: старшие 4 бита — идентификатор сообщения, младшие 12 — порядковый номер части
constexpr uint16_t kSeqPartMask = 0x0FFFU;
constexpr uint8_t  kSeqMsgIdShift = 12U;
inline uint16_t composeSeq(uint8_t msgId, uint16_t part) {
  return static_cast<uint16_t>((static_cast<uint16_t>(msgId & 0x0F) << kSeqMsgIdShift) |
                               (part & kSeqPartMask));
}
inline uint8_t extractMsgId(uint16_t seq) {
  return static_cast<uint8_t>((seq >> kSeqMsgIdShift) & 0x0F);
}
inline uint16_t extractPart(uint16_t seq) {
  return static_cast<uint16_t>(seq & kSeqPartMask);
}

struct DataBlock {
  std::array<uint8_t, kFramePayloadSize> bytes{}; // полезная нагрузка DATA-пакета
  uint8_t dataLength = 0;                         // фактическое число информационных байт
};

// --- Пакет паритета для восстановления одного потерянного DATA-блока ---
struct ParityPacket {
  DataBlock block;        // XOR всех DATA-блоков окна
  uint8_t lengthXor = 0;  // XOR фактических длин DATA-блоков окна
};

// --- Структура, описывающая событие в веб-чате ---
struct ChatEvent {
  unsigned long id = 0;   // уникальный идентификатор сообщения
  String text;            // сам текст события
  String color;           // цвет текста в веб-интерфейсе
};

// --- Текущее состояние приложения ---
struct ProtocolConfig {
  bool interleaving = false;           // включён ли интерливинг
  bool harq = false;                   // активен ли HARQ (адаптивный FEC)
  bool phyFec = false;                 // использовать ли фиксированный FEC PHY
  bool payloadCrc8 = false;            // добавлять ли CRC-8 к каждому DATA-пакету
};

struct RxWindowState {
  uint16_t baseSeq = 0;                // база текущего окна подтверждения
  uint16_t receivedMask = 0;           // биты успешно принятых пакетов
  uint8_t windowSize = 0;              // фактическое количество пакетов в окне
  bool active = false;                 // инициализировано ли окно
  bool parityValid = false;            // получен ли паритет для текущего окна
  DataBlock parityBlock;               // сохранённый паритетный блок
  uint8_t parityLengthXor = 0;         // XOR длин блоков в окне
  unsigned long adaptiveAckTimeoutMs = 1000; // динамический тайм-аут ожидания ACK
  unsigned long lastAckRttMs = 0;             // последнее измеренное RTT до ACK
};

struct RxMessageState {
  bool active = false;                 // идёт ли сборка файла
  uint16_t nextExpectedSeq = 0;        // следующий ожидаемый SEQ
  std::map<uint16_t, DataBlock> pending; // буфер ожидания
  std::vector<uint8_t> buffer;         // текущие собранные данные
  uint16_t declaredLength = 0;         // ожидаемая длина файла из FIN
  uint16_t declaredCrc = 0;            // ожидаемый CRC-16 из FIN
  bool finReceived = false;            // получен ли FIN
  uint16_t announcedPartCount = 0;     // заявленное количество частей
  bool partCountFromPrefix = false;    // метаданные первой части обработаны
  bool partCountFromSuffix = false;    // метаданные последней части обработаны
  unsigned long lastActivityMs = 0;    // время последней активности (принят DATA/PAR/FIN)
  unsigned long lastProgressMs = 0;    // время последнего продвижения (собран следующий SEQ)
};

struct AckNotification {
  bool hasValue = false;               // получен ли ACK от удалённой стороны
  uint16_t baseSeq = 0;                // база окна
  uint16_t missingBitmap = 0;          // биты недостающих пакетов (1 = требуется повтор)
  bool needParity = false;             // требуется ли PAR-передача
  uint8_t reportedWindow = 0;          // размер окна, который видел получатель
  uint8_t ackType = 0;                 // тип подтверждения (успех/потери)
};

// --- Диагностика и настройки длинных окон приёма ---
struct RxTimingDiagnostics {
  uint16_t preambleSymbols = static_cast<uint16_t>(kRadioDefaults.preambleLength); // текущая длина преамбулы
  uint32_t rxTimeoutSymbols = 0;                  // установленный тайм-аут в символах (0 = бесконечный)
  uint16_t symbTimeout = 0;                       // ограничение LoRaSymbNumTimeout, если поддерживается
  bool stopTimerOnPreamble = false;               // активирован ли StopRxTimerOnPreamble
  bool forceContinuousRx = true;                  // временно держим приёмник в бесконечном SetRx для диагностики
  bool rxContinuous = false;                      // удалось ли перевести в непрерывный режим
  bool ldroForced = false;                        // включена ли принудительная оптимизация LDRO
  bool ldroOverrideActive = false;               // ручное управление LDRO активно
  bool ldroOverrideValue = false;                // целевое значение LDRO при ручном управлении
  unsigned long lastSetRxMs = 0;                  // отметка времени последнего SetRx
  unsigned long lastSetRxLogMs = 0;               // отметка времени последнего логирования SetRx (для подавления спама)
  bool reportedMissingStopTimerSupport = false;   // уже сообщали об отсутствии API StopRxTimerOnPreamble
  bool reportedMissingRxTimeoutSupport = false;   // уже сообщали об отсутствии API установки тайм-аута
  bool reportedMissingSymbTimeoutSupport = false; // уже сообщали об отсутствии API SymbNumTimeout
  bool reportedMissingLdroSupport = false;        // уже сообщали об отсутствии API принудительного LDRO
};

// Группа состояний приёма, ассоциированная с конкретным msgId
struct PerMessageRx {
  RxWindowState win;  // состояние окна для ACK/паритета
  RxMessageState msg; // состояние сборки сообщения
};

struct FhssHop {
  uint8_t channel = 0;        // базовый канал HOME для прыжка
  float offsetKhz = 0.0f;     // дополнительное смещение частоты в килогерцах
};

constexpr size_t kMaxFhssHops = 32; // максимальное количество шагов в последовательности FHSS

struct FhssConfig {
  bool enabled = false;                                              // включён ли программный FHSS
  unsigned long dwellTimeMs = 80;                                   // длительность стоянки на одном прыжке
  std::array<FhssHop, kMaxFhssHops> hopSequence{};                   // последовательность прыжков с учётом смещений
  size_t hopCount = 0;                                               // фактическое количество активных шагов
  size_t currentHopIndex = 0;                                        // индекс текущего прыжка в массиве
  size_t nextHopIndex = 0;                                           // индекс следующего прыжка для передачи/приёма
  uint8_t currentHopChannel = 0;                                     // номер канала HOME, на котором сейчас работаем
  float currentHopOffsetKhz = 0.0f;                                  // применённое смещение к текущему каналу
  uint8_t previousChannel = 0;                                       // сохранённый «фиксированный» канал для возврата при выключении FHSS
  unsigned long lastHopMillis = 0;                                   // отметка времени последнего переключения частоты
  bool limitToTenKhz = true;                                         // ограничивать прыжки диапазоном ±5 кГц
  uint8_t baseChannelForTenKhz = 0;                                  // базовый канал для узкополосного FHSS
  uint8_t suspendDepth = 0;                                          // глубина временной заморозки прыжков
};

// Повторная отправка и голосование по тройным копиям
struct TripleVoteEntry {
  std::array<std::array<uint8_t, kFixedFrameSize>, 3> frames{}; // копии принятого кадра
  uint8_t count = 0;                                             // сколько копий накоплено (0..3)
  unsigned long firstMs = 0;                                     // отметка времени первой копии
};

struct TripleVoteState {
  bool enabledTx = false;                                        // тройная отправка на TX
  bool enabledRx = false;                                        // голосование на RX
  unsigned long holdMs = 25;                                     // максимальная задержка ожидания 3-й копии
  std::map<uint16_t, TripleVoteEntry> pending;                   // по SEQ для DATA-кадров
};

struct AppState {
  uint8_t channelIndex = 0;            // выбранный канал банка HOME (фиксированный режим)
  bool highPower = false;              // признак использования мощности 22 dBm (иначе -5 dBm)
  bool rxBoostedGain = kRadioDefaults.rxBoostedGain; // режим усиленного приёма LNA
  uint8_t selectedSpreadingFactor = kDefaultSpreadingFactor; // выбранный пользователем SF
  float currentRxFreq = frequency_tables::RX_HOME[0]; // текущая частота приёма
  float currentTxFreq = frequency_tables::TX_HOME[0]; // текущая частота передачи
  bool symmetricTxRx = true;           // лабораторный режим: TX = RX (без транспондера)
  unsigned long nextEventId = 1;       // счётчик идентификаторов для событий
  std::vector<ChatEvent> events;       // журнал событий для веб-интерфейса
  ProtocolConfig protocol;             // параметры протокола Lotest
  uint16_t nextTxSequence = 0;         // следующий SEQ для передачи DATA
  RxWindowState rxWindow;              // состояние окна приёма
  RxMessageState rxMessage;            // состояние сборки сообщения
  AckNotification pendingAck;          // последнее полученное подтверждение
  float radioBandwidthKhz = kDefaultBandwidthKhz; // активная полоса пропускания
  uint8_t currentSpreadingFactor = kDefaultSpreadingFactor; // фактический SF
  RxTimingDiagnostics rxTiming;        // состояние тайминговых оптимизаций
  FhssConfig fhss;                     // параметры программного частотного прыжкового режима
  size_t implicitPayloadLength = kImplicitPayloadLength; // актуальная длина фиксированного LoRa-пакета
  uint8_t nextMessageId = 0;           // идентификатор для следующего исходящего сообщения (0..15)
  std::map<uint8_t, PerMessageRx> rxSessions; // параллельные сборки по msgId
  TripleVoteState triple;              // состояние тройной защиты (TX/ RX)
} state;

// --- Флаги приёма LoRa ---
volatile bool packetReceivedFlag = false;   // устанавливается обработчиком DIO1 при приёме
volatile bool packetProcessingEnabled = true; // защита от повторного входа в обработчик
volatile bool irqStatusPending = false;     // есть ли необработанные IRQ-флаги SX1262

// --- Поддержка RadioLib: используем современный API напрямую ---

// --- Вспомогательные функции объявления ---
void IRAM_ATTR onRadioDio1Rise();
String formatSx1262IrqFlags(uint32_t flags);
void addEvent(const String& message, const String& color = String());
void appendEventBuffer(const String& message, unsigned long id, const String& color = String());
void handleRoot();
void handleLog();
void handleChannelChange();
void handlePowerToggle();
void handleRxBoostToggle();
void handleSendLongPacket();
void handleSendRandomPacket();
void handleSendCustom();
void handleNotFound();
void handleProtocolToggle();
void handleSpreadingFactorChange();
void handleFhssToggle();
void handleTcxoConfig();
void handleLdroToggle();
void handleTcxoCheck();
String buildIndexHtml();
String buildChannelOptions(uint8_t selected);
String escapeJson(const String& value);
String makeAccessPointSsid();
void handleImplicitLenChange();
bool applyRadioChannel(uint8_t newIndex);
bool tuneToHomeChannel(uint8_t newIndex, bool updateSelection, float offsetKhz = 0.0f);
bool applyRadioPower(bool highPower);
bool applySpreadingFactor(uint8_t spreadingFactor);
bool applyPhyFec(bool enable);
bool ensureReceiveMode();
void processRadioEvents();
bool sendPayload(const std::vector<uint8_t>& payload, const String& context);
bool transmitFrame(const std::array<uint8_t, kFixedFrameSize>& frame, const String& context);
void processIncomingFrame(const std::vector<uint8_t>& frame);
void processIncomingDataFrame(const std::vector<uint8_t>& frame);
void processIncomingAckFrame(const std::vector<uint8_t>& frame);
void processIncomingParityFrame(uint16_t seq,
                                uint8_t lengthField,
                                bool ackRequest,
                                const std::vector<uint8_t>& frame);
void processIncomingFinFrame(const std::vector<uint8_t>& frame);
void prepareAckFor(PerMessageRx& ctx, uint16_t seq, uint8_t windowSize, bool forceSend);
void sendAck(uint16_t baseSeq, uint16_t missingBitmap, bool needParity, uint8_t windowSize);
void flushPendingDataFor(PerMessageRx& ctx);
bool sendDataWindow(const std::vector<DataBlock>& blocks,
                    size_t offset,
                    uint16_t baseSeq,
                    uint8_t windowSize);
bool waitForAck(uint16_t baseSeq, uint8_t windowSize, uint16_t& missingBitmap, bool& needParity);
void retransmitMissing(const std::vector<DataBlock>& blocks,
                       size_t offset,
                       uint16_t baseSeq,
                       uint16_t missingBitmap);
std::vector<DataBlock> splitPayloadIntoBlocks(const std::vector<uint8_t>& payload,
                                              bool appendCrc8);
std::vector<uint8_t> buildTransportPayloadWithPartMarkers(const std::vector<uint8_t>& payload,
                                                          bool appendCrc8,
                                                          uint16_t& announcedPartCount);
std::array<uint8_t, kFixedFrameSize> buildDataFrame(uint16_t seq,
                                                    const DataBlock& block,
                                                    bool ackRequest,
                                                    bool isParity,
                                                    uint8_t parityLengthXor = 0);
std::array<uint8_t, kFixedFrameSize> buildStartFrame(uint8_t msgId);
std::array<uint8_t, kFixedFrameSize> buildAckFrame(uint16_t baseSeq,
                                                   uint16_t missingBitmap,
                                                   bool needParity,
                                                   uint8_t windowSize,
                                                   uint8_t ackType);
std::array<uint8_t, kFixedFrameSize> buildFinFrame(uint16_t length,
                                                   uint16_t crc,
                                                   bool harqUsed,
                                                   uint8_t msgId);
ParityPacket computeWindowParity(const std::vector<DataBlock>& blocks,
                                 size_t offset,
                                 uint8_t windowSize);
bool attemptParityRecoveryFor(PerMessageRx& ctx);
uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF);
uint8_t crc8Dallas(const uint8_t* data, size_t length);
void resetReceiveState();
void logReceivedMessage(const std::vector<uint8_t>& payload);
void logRadioError(const String& context, int16_t code);
void initFhssDefaults();
void rebuildFhssSequence();
String formatFhssHopDescription(uint8_t channel, float offsetKhz);
bool setFhssEnabled(bool enable);
void updateFhss();
bool applyFhssHopByIndex(size_t hopIndex);
bool advanceFhssIfDue();
void fhssSuspend();
void fhssResume();
void waitInterFrameDelay();
unsigned long computeInterFrameDelayMs();
float estimateLoRaAirTimeMs(uint8_t payloadSize, uint16_t preambleSymbols);
unsigned long computeAckTimeoutMs(uint8_t windowSize);
void updateAckTiming(unsigned long measuredRttMs);
void handleAckTimeoutExpansion();
String formatByteArray(const std::vector<uint8_t>& data);
String formatTextPayload(const std::vector<uint8_t>& data);
uint16_t readUint16Le(const uint8_t* data);
void configureNarrowbandRxOptions();
void logRxTimingEvent(const String& message);
void maybeResetStalledReception();
void finalizeSessionWithoutFin(uint8_t msgId, PerMessageRx& pm);
// Тройная защита: функции
bool tripleRxMaybeDecide(uint16_t seq, const std::vector<uint8_t>& frame, std::vector<uint8_t>& decided);
void tripleCleanupStale();

// --- Формирование имени Wi‑Fi: BASE_UPPER + '_' + MAC6 ---
String makeAccessPointSsid() {
  String base = LOTEST_WIFI_SSID;              // базовый SSID из настроек теста
  base.toUpperCase();                          // требуемый формат: SAT_AP_XXXXXX
#if defined(ARDUINO)
  uint8_t macBytes[6] = {0};
#  if defined(ESP32)
  WiFi.macAddress(macBytes);                   // байтовый MAC для максимальной переносимости
#  elif defined(ESP8266)
  uint32_t chip = ESP.getChipId();
  macBytes[3] = (chip >> 16) & 0xFF;
  macBytes[4] = (chip >> 8) & 0xFF;
  macBytes[5] = (chip) & 0xFF;
#  else
  WiFi.macAddress(macBytes);
#  endif
  char mac12[13];
  std::snprintf(mac12, sizeof(mac12), "%02X%02X%02X%02X%02X%02X",
                macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
  // Формат SSID: SAT_AP_AABBCCDDEEFF — полные 6 байт MAC для исключения коллизий
  return base + "_" + String(mac12);
#else
  return base + String("_000000000000");       // стабы для хостовых тестов без Arduino
#endif
}

// --- Инициализация оборудования ---
void setup() {
  Serial.begin(115200);
  delay(200);
  addEvent("Запуск устройства " LOTEST_PROJECT_NAME);
  randomSeed(esp_random());

  initFhssDefaults();

  // Настройка тройной защиты по макросам сборки
#ifdef LOTEST_TRIPLE_RETX
  state.triple.enabledTx = (LOTEST_TRIPLE_RETX != 0);
#endif
#ifdef LOTEST_TRIPLE_RXVOTE
  state.triple.enabledRx = (LOTEST_TRIPLE_RXVOTE != 0);
#endif
#ifdef LOTEST_TRIPLE_HOLD_MS
  state.triple.holdMs = static_cast<unsigned long>(LOTEST_TRIPLE_HOLD_MS);
#endif
  if (state.triple.enabledTx || state.triple.enabledRx) {
    addEvent(String("Тройная защита: TX=") + (state.triple.enabledTx ? "on" : "off") +
             ", RX=" + (state.triple.enabledRx ? "on" : "off") +
             ", hold=" + String(state.triple.holdMs) + " мс");
  }

  // Настройка SPI для радиомодуля SX1262
  radioSPI.begin(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_CS);

  // Подключаем обработчик прерывания по DIO1 для асинхронного приёма
  radio.setDio1Action(onRadioDio1Rise);

  // Стартуем радиомодуль и применяем параметры, аналогичные основной прошивке
  const float initialRxFreq = frequency_tables::RX_HOME[state.channelIndex];
  const uint8_t syncWord = static_cast<uint8_t>(kRadioDefaults.syncWord & 0xFFU);
  const float tcxoVoltage = (kRadioDefaults.useDio3ForTcxo && kRadioDefaults.tcxoVoltage > 0.0f)
                               ? kRadioDefaults.tcxoVoltage
                               : 0.0f;
  const int8_t initialPowerDbm = state.highPower ? kHighPowerDbm : kLowPowerDbm; // стартовая мощность
  int16_t beginState = radio.begin(initialRxFreq,
                                   kDefaultBandwidthKhz,
                                   kDefaultSpreadingFactor,
                                   kDefaultCodingRate,
                                   syncWord,
                                   initialPowerDbm,
                                   kRadioDefaults.preambleLength,
                                   tcxoVoltage,
                                   kRadioDefaults.enableRegulatorLDO);
  if (beginState != RADIOLIB_ERR_NONE) {
    logRadioError("radio.begin", beginState);
  } else {
    addEvent("Радиомодуль успешно инициализирован");

    // Применяем настройки LoRa согласно требованиям задачи
    state.selectedSpreadingFactor = kDefaultSpreadingFactor;
    if (!applySpreadingFactor(state.selectedSpreadingFactor)) {
      addEvent("Не удалось применить стартовый SF — используем параметры по умолчанию");
    }
    int16_t bwState = radio.setBandwidth(kDefaultBandwidthKhz);
    if (bwState != RADIOLIB_ERR_NONE) {
      logRadioError("setBandwidth", bwState);
    } else {
      state.radioBandwidthKhz = kDefaultBandwidthKhz;
    }
    int16_t crState = radio.setCodingRate(kDefaultCodingRate);
    if (crState != RADIOLIB_ERR_NONE) {
      logRadioError("setCodingRate", crState);
    }

    radio.setDio2AsRfSwitch(kRadioDefaults.useDio2AsRfSwitch);
    if (kRadioDefaults.useDio3ForTcxo && kRadioDefaults.tcxoVoltage > 0.0f) {
      int16_t tcxoRc = radio.setTCXO(kRadioDefaults.tcxoVoltage, kRadioDefaults.tcxoDelayUs); // включаем внешний TCXO и задаём задержку прогрева
      if (tcxoRc == RADIOLIB_ERR_NONE) {
        addEvent(String("TCXO включён на DIO3: V=") + String(kRadioDefaults.tcxoVoltage, 2) +
                 String(" В, задержка=") + String(static_cast<unsigned long>(kRadioDefaults.tcxoDelayUs)) +
                 String(" мкс, преамбула=") + String(kRadioDefaults.preambleLength));
      } else {
        logRadioError("setTCXO (boot)", tcxoRc);
        addEvent(String("TCXO включение не подтверждено, продолжаем с указанными параметрами: V=") +
                 String(kRadioDefaults.tcxoVoltage, 2) + String(" В, задержка=") +
                 String(static_cast<unsigned long>(kRadioDefaults.tcxoDelayUs)) + String(" мкс"));
      }
    } else {
      addEvent(String("TCXO отключён (XTAL), преамбула=") + String(kRadioDefaults.preambleLength));
    }
    if (kRadioDefaults.implicitHeader) {
      radio.implicitHeader(kImplicitPayloadLength);
    } else {
      radio.explicitHeader();
    }
    radio.setCRC(kRadioDefaults.enableCrc ? 2 : 0);          // длина CRC в байтах: 2 либо 0
    radio.invertIQ(kRadioDefaults.invertIq);                 // включаем или выключаем инверсию IQ
    int16_t preambleState = radio.setPreambleLength(kRadioDefaults.preambleLength);
    if (preambleState != RADIOLIB_ERR_NONE) {
      logRadioError("setPreambleLength", preambleState);
    } else {
      state.rxTiming.preambleSymbols = kRadioDefaults.preambleLength;
    }
    radio.setRxBoostedGainMode(state.rxBoostedGain); // режим усиленного приёма SX1262
    radio.setSyncWord(kRadioDefaults.syncWord);

    configureNarrowbandRxOptions();

    if (!applyRadioChannel(state.channelIndex)) {
      addEvent("Ошибка инициализации канала — проверьте модуль SX1262");
    }
    if (!applyRadioPower(state.highPower)) {
      addEvent("Ошибка установки мощности — проверьте подключение модуля");
    }
  }

  // Поднимаем точку доступа для веб-интерфейса
  WiFi.mode(WIFI_MODE_AP);
  String ssid = makeAccessPointSsid();
#if defined(ESP32) || defined(ESP8266)
  uint8_t macBytes[6] = {0};
  WiFi.macAddress(macBytes);
  char mac12[13];
  std::snprintf(mac12, sizeof(mac12), "%02X%02X%02X%02X%02X%02X",
                macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
  addEvent(String("HW MAC (eFuse/NVS): ") + String(mac12));
#endif
  bool apStarted = WiFi.softAP(ssid.c_str(), LOTEST_WIFI_PASS);
  if (apStarted) {
    addEvent(String("Точка доступа запущена: ") + ssid);
    addEvent(String("IP адрес веб-интерфейса: ") + WiFi.softAPIP().toString());
  } else {
    addEvent("Не удалось запустить точку доступа Wi-Fi");
  }

  // Регистрируем HTTP-маршруты
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/channel", HTTP_POST, handleChannelChange);
  server.on("/api/power", HTTP_POST, handlePowerToggle);
  server.on("/api/rxboost", HTTP_POST, handleRxBoostToggle);
  server.on("/api/send/five", HTTP_POST, handleSendLongPacket); // совместимость с прежним маршрутом
  server.on("/api/send/fixed", HTTP_POST, handleSendLongPacket);
  server.on("/api/send/long", HTTP_POST, handleSendLongPacket);
  server.on("/api/send/random", HTTP_POST, handleSendRandomPacket);
  server.on("/api/send/custom", HTTP_POST, handleSendCustom);
  server.on("/api/sf", HTTP_POST, handleSpreadingFactorChange);
  server.on("/api/fhss", HTTP_POST, handleFhssToggle);
  server.on("/api/tcxo", HTTP_POST, handleTcxoConfig);
  server.on("/api/tcxo/check", HTTP_POST, handleTcxoCheck);
  server.on("/api/ldro", HTTP_POST, handleLdroToggle);
  server.on("/api/implicit", HTTP_POST, handleImplicitLenChange);
  server.on("/api/protocol", HTTP_POST, handleProtocolToggle);
  server.onNotFound(handleNotFound);
  server.begin();
  addEvent("HTTP-сервер запущен на порту 80");
}

// --- Основной цикл ---
void loop() {
  server.handleClient();
  processRadioEvents();
  updateFhss();
}

// --- Обработка радиособытий вне основного цикла ---
void processRadioEvents() {
  bool processIrq = false;                    // требуется ли перенос IRQ-статуса в основной поток
#if defined(ARDUINO)
  noInterrupts();                              // защищаем флаг от изменения в ISR
#endif
  if (irqStatusPending) {                     // фиксируем запрос на логирование IRQ
    irqStatusPending = false;
    processIrq = true;
  }
#if defined(ARDUINO)
  interrupts();                               // возвращаем обработку прерываний
#endif

  if (processIrq) {
    const uint32_t irqFlags = radio.getIrqFlags(); // чтение IRQ-флагов RadioLib
    (void)radio.clearIrqFlags(irqFlags);           // очистка прочитанных флагов
    String irqMessage = formatSx1262IrqFlags(irqFlags);               // публикуем расшифровку событий с таймингом
    if (state.rxTiming.lastSetRxMs != 0) {
      const unsigned long delta = millis() - state.rxTiming.lastSetRxMs;
      irqMessage += String(" | прошло ") + String(delta) + " мс с последнего SetRx";
    }
    logRxTimingEvent(irqMessage);
  }

  if (!packetReceivedFlag) {
    // Периодически проверяем зависшую сборку сообщения
    tripleCleanupStale();
    maybeResetStalledReception();
    return;
  }

  packetProcessingEnabled = false;
  packetReceivedFlag = false;

  std::vector<uint8_t> buffer(state.implicitPayloadLength, 0);
  int16_t stateCode = radio.readData(buffer.data(), buffer.size());
  if (stateCode == RADIOLIB_ERR_NONE) {
    size_t actualLength = radio.getPacketLength();
    if (actualLength > buffer.size()) {
      actualLength = buffer.size();
    }
    buffer.resize(actualLength);
    // Записываем метрики уровня сигнала для принятого пакета
    float rssiDbm = radio.getRSSI();
    float snrDb = radio.getSNR();
    String rssiLine = String("Принят пакет: RSSI=") + String(rssiDbm, 1) +
                      " dBm, SNR=" + String(snrDb, 1) + " dB, len=" + String(static_cast<unsigned long>(actualLength));
    addEvent(rssiLine, String(kIncomingColor));
    processIncomingFrame(buffer);
  } else {
    logRadioError("readData", stateCode);
  }

  ensureReceiveMode();
  packetProcessingEnabled = true;

  // После обработки пакета также проверим зависание сборки
  tripleCleanupStale();
  maybeResetStalledReception();
}

// --- Обработчик линии DIO1 радиомодуля ---
void IRAM_ATTR onRadioDio1Rise() {
  irqStatusPending = true;                   // отмечаем необходимость чтения IRQ-статуса
  if (!packetProcessingEnabled) {
    return;
  }
  packetReceivedFlag = true;
}

// --- Тройная защита: удалить устаревшие заготовки ---
void tripleCleanupStale() {
  if (!state.triple.enabledRx || state.triple.pending.empty()) {
    return;
  }
  const unsigned long now = millis();
  for (auto it = state.triple.pending.begin(); it != state.triple.pending.end(); ) {
    if (now - it->second.firstMs > state.triple.holdMs) {
      it = state.triple.pending.erase(it);
    } else {
      ++it;
    }
  }
}

// --- Тройная защита: накопить до 3 копий и принять решение ---
bool tripleRxMaybeDecide(uint16_t seq, const std::vector<uint8_t>& frame, std::vector<uint8_t>& decided) {
  // Безопасность
  if (frame.size() != kFixedFrameSize) {
    decided = frame; // fallback: не пытаемся голосовать
    return true;
  }

  TripleVoteEntry& e = state.triple.pending[seq];
  if (e.count == 0) {
    std::copy_n(frame.begin(), kFixedFrameSize, e.frames[0].begin());
    e.count = 1;
    e.firstMs = millis();
    return false; // ждём вторую/третью копию
  }

  // Если совпадает с уже имеющейся копией — принимаем немедленно
  for (uint8_t i = 0; i < e.count; ++i) {
    if (std::equal(frame.begin(), frame.end(), e.frames[i].begin())) {
      decided.assign(e.frames[i].begin(), e.frames[i].end());
      state.triple.pending.erase(seq);
      return true;
    }
  }

  if (e.count < 3) {
    std::copy_n(frame.begin(), kFixedFrameSize, e.frames[e.count].begin());
    ++e.count;
  }

  if (e.count < 3) {
    // Пока есть только 2 разные копии — ждём третью
    return false;
  }

  // Есть три копии, применяем помехоустойчивое решение по байтам
  std::array<uint8_t, kFixedFrameSize> out{};
  for (size_t b = 0; b < kFixedFrameSize; ++b) {
    uint8_t v0 = e.frames[0][b];
    uint8_t v1 = e.frames[1][b];
    uint8_t v2 = e.frames[2][b];
    if (v0 == v1 || v0 == v2) {
      out[b] = v0;
    } else if (v1 == v2) {
      out[b] = v1;
    } else {
      out[b] = v0; // нет большинства — берём первую копию
    }
  }
  decided.assign(out.begin(), out.end());
  state.triple.pending.erase(seq);
  return true;
}

// --- Формирование человекочитаемого описания IRQ SX1262 ---
String formatSx1262IrqFlags(uint32_t flags) {
  const uint32_t effectiveMask = flags & 0xFFFFU;                  // интересуют только младшие 16 бит
  if (effectiveMask == RADIOLIB_SX126X_IRQ_NONE) {
    return F("SX1262 IRQ: флаги отсутствуют (маска=0x0000)");
  }

  struct IrqEntry {
    uint32_t mask;                                                  // битовая маска события
    const char* name;                                               // краткое имя IRQ
    const char* description;                                        // пояснение для оператора
  };

  static const IrqEntry kIrqMap[] = {
      {RADIOLIB_SX126X_IRQ_TX_DONE, "IRQ_TX_DONE", "передача завершена"},
      {RADIOLIB_SX126X_IRQ_RX_DONE, "IRQ_RX_DONE", "приём завершён"},
      {RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED, "IRQ_PREAMBLE_DETECTED", "обнаружена преамбула"},
      {RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID, "IRQ_SYNCWORD_VALID", "сошёлся sync word"},
      {RADIOLIB_SX126X_IRQ_HEADER_VALID, "IRQ_HEADER_VALID", "валидный заголовок"},
      {RADIOLIB_SX126X_IRQ_HEADER_ERR, "IRQ_HEADER_ERR", "ошибка заголовка"},
      {RADIOLIB_SX126X_IRQ_CRC_ERR, "IRQ_CRC_ERR", "ошибка CRC"},
#if defined(RADIOLIB_IRQ_RX_TIMEOUT)
      {RADIOLIB_IRQ_RX_TIMEOUT, "IRQ_RX_TIMEOUT", "тайм-аут приёма"},
#endif
#if defined(RADIOLIB_IRQ_TX_TIMEOUT)
      {RADIOLIB_IRQ_TX_TIMEOUT, "IRQ_TX_TIMEOUT", "тайм-аут передачи"},
#endif
#if !defined(RADIOLIB_IRQ_RX_TIMEOUT) && !defined(RADIOLIB_IRQ_TX_TIMEOUT)
      {RADIOLIB_SX126X_IRQ_TIMEOUT, "IRQ_RX_TX_TIMEOUT", "тайм-аут RX/TX"},
#endif
      {RADIOLIB_SX126X_IRQ_CAD_DONE, "IRQ_CAD_DONE", "CAD завершён"},
      {RADIOLIB_SX126X_IRQ_CAD_DETECTED, "IRQ_CAD_DETECTED", "CAD обнаружил передачу"},
#ifdef RADIOLIB_SX126X_IRQ_LR_FHSS_HOP
      {RADIOLIB_SX126X_IRQ_LR_FHSS_HOP, "IRQ_LR_FHSS_HOP", "LR-FHSS hop"},
#endif
  };

  String decoded;
  decoded.reserve(192);                                             // предотвращаем повторные аллокации
  uint32_t knownMask = 0U;                                          // известные биты
  bool first = true;
  for (const auto& entry : kIrqMap) {
    if ((effectiveMask & entry.mask) == 0U) {
      continue;                                                     // пропускаем неактивные события
    }
    if (!first) {
      decoded += F(" | ");
    }
    decoded += entry.name;
    decoded += F(" — ");
    decoded += entry.description;
    first = false;
    knownMask |= entry.mask;
  }

  String result;
  result.reserve(224);
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "SX1262 IRQ=0x%04lX", static_cast<unsigned long>(effectiveMask));
  result += buffer;

  if (decoded.length() > 0) {
    result += F(", расшифровка: [");
    result += decoded;
    result += ']';
  }

  const uint32_t unknownMask = effectiveMask & ~knownMask;
  if (unknownMask != 0U) {
    std::snprintf(buffer, sizeof(buffer), ", неизвестные биты=0x%04lX", static_cast<unsigned long>(unknownMask));
    result += buffer;
  }

  return result;
}

// --- Добавление события в лог ---
void addEvent(const String& message, const String& color) {
  appendEventBuffer(message, state.nextEventId++, color);
  Serial.println(message);
}

// --- Вспомогательная функция: дописываем событие и ограничиваем историю ---
void appendEventBuffer(const String& message, unsigned long id, const String& color) {
  ChatEvent event;
  event.id = id;
  event.text = message;
  event.color = color;
  state.events.push_back(event); // явное построение объекта для совместимости со старыми стандартами C++
  if (state.events.size() > kMaxEventHistory) {
    state.events.erase(state.events.begin());
  }
}

// --- Логирование тайминговых событий RX с отметкой времени ---
void logRxTimingEvent(const String& message) {
  const unsigned long now = millis();
  String formatted = String("[RX t=") + String(now) + F(" мс] ") + message;
  addEvent(formatted);
}

// --- HTML главной страницы ---
void handleRoot() {
  server.send(200, "text/html; charset=UTF-8", buildIndexHtml());
}

// --- API: выдача журналов событий ---
void handleLog() {
  unsigned long after = 0;
  if (server.hasArg("after")) {
    after = strtoul(server.arg("after").c_str(), nullptr, 10);
  }

  String payload = "{\"events\":[";
  bool first = true;
  unsigned long lastId = after;
  for (const auto& evt : state.events) {
    if (evt.id <= after) {
      continue;
    }
    if (!first) {
      payload += ',';
    }
    payload += "{\"id\":" + String(evt.id) + ",\"text\":\"" + escapeJson(evt.text) + "\"";
    if (evt.color.length() > 0) {
      payload += ",\"color\":\"" + escapeJson(evt.color) + "\"";
    }
    payload += "}";
    first = false;
    lastId = evt.id;
  }
  payload += "],\"lastId\":" + String(lastId) + "}";

  server.send(200, "application/json", payload);
}

// --- API: смена канала ---
void handleChannelChange() {
  if (!server.hasArg("channel")) {
    server.send(400, "application/json", "{\"error\":\"Не указан параметр channel\"}");
    return;
  }
  int channel = server.arg("channel").toInt();
  if (channel < 0 || channel >= kHomeBankSize) {
    server.send(400, "application/json", "{\"error\":\"Недопустимый канал\"}");
    return;
  }
  const uint8_t newIndex = static_cast<uint8_t>(channel);
  bool success = false;
  if (state.fhss.enabled) {
    state.channelIndex = newIndex;
    state.fhss.previousChannel = newIndex;
    rebuildFhssSequence();
    size_t startIndex = 0;
    float bestOffset = std::numeric_limits<float>::max();
    for (size_t i = 0; i < state.fhss.hopCount; ++i) {
      const FhssHop& hop = state.fhss.hopSequence[i];
      if (hop.channel != state.channelIndex) {
        continue;
      }
      const float absOffset = std::fabs(hop.offsetKhz);
      if (absOffset < bestOffset) {
        bestOffset = absOffset;
        startIndex = i;
        if (absOffset < 0.001f) {
          break;                                                   // стремимся начать с минимального смещения
        }
      }
    }
    success = applyFhssHopByIndex(startIndex);
  } else {
    success = applyRadioChannel(newIndex);
  }

  if (success) {
    addEvent(String("Выбран канал HOME #") + String(channel) + ", RX=" + String(state.currentRxFreq, 3) + " МГц, TX=" + String(state.currentTxFreq, 3) + " МГц");
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Не удалось применить канал\"}");
  }
}

// --- API: переключение мощности ---
void handlePowerToggle() {
  bool newHighPower = server.hasArg("high") && server.arg("high") == "1";
  if (applyRadioPower(newHighPower)) {
    state.highPower = newHighPower;
    addEvent(String("Мощность передачи установлена в ") + (newHighPower ? "22 dBm" : "-5 dBm"));
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Ошибка установки мощности\"}");
  }
}

// --- API: переключение усиленного приёма (LNA Boost) ---
void handleRxBoostToggle() {
  bool enable = server.hasArg("enable") && server.arg("enable") == "1";
  int16_t rc = radio.setRxBoostedGainMode(enable);
  if (rc != RADIOLIB_ERR_NONE) {
    logRadioError("setRxBoostedGainMode", rc);
    server.send(500, "application/json", "{\"error\":\"Не удалось переключить усиленный приём\"}");
    return;
  }
  state.rxBoostedGain = enable;
  addEvent(String("Усиленный приём (LNA Boost) ") + (enable ? "включён" : "выключен"));
  ensureReceiveMode();
  server.send(200, "application/json", "{\"ok\":true}");
}

// --- API: установка фактора расширения ---
void handleSpreadingFactorChange() {
  if (!server.hasArg("sf")) {
    server.send(400, "application/json", "{\"error\":\"Не указан параметр sf\"}");
    return;
  }

  int requested = server.arg("sf").toInt();
  if (requested < 5 || requested > 12) {
    server.send(400, "application/json", "{\"error\":\"SF допустим в диапазоне 5-12\"}");
    return;
  }

  if (applySpreadingFactor(static_cast<uint8_t>(requested))) {
    addEvent(String("Фактор расширения установлен: SF") + String(requested));
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Ошибка установки SF\"}");
  }
}

// --- API: переключение программного FHSS ---
void handleFhssToggle() {
  bool enable = server.hasArg("enable") && server.arg("enable") == "1";
  if (setFhssEnabled(enable)) {
    if (enable) {
      const String hopInfo = formatFhssHopDescription(state.fhss.currentHopChannel,
                                                     state.fhss.currentHopOffsetKhz);
      addEvent(String("FHSS включён: ") + hopInfo + ", RX=" + String(state.currentRxFreq, 3) +
               " МГц, TX=" + String(state.currentTxFreq, 3) + " МГц");
    } else {
      const String hopInfo = formatFhssHopDescription(state.channelIndex, 0.0f);
      addEvent(String("FHSS выключен, фиксированный ") + hopInfo + ", RX=" +
               String(state.currentRxFreq, 3) + " МГц, TX=" + String(state.currentTxFreq, 3) +
               " МГц");
    }
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Не удалось переключить FHSS\"}");
  }
}

// --- API: ручное управление LDRO ---
void handleLdroToggle() {
  bool handled = false;
  if (server.hasArg("auto") && server.arg("auto") == "1") {
    int16_t rc = radio.autoLDRO();
    if (rc != RADIOLIB_ERR_NONE) {
      logRadioError("autoLDRO", rc);
      server.send(500, "application/json", "{\"error\":\"Не удалось вернуть LDRO в AUTO\"}");
      return;
    }
    state.rxTiming.ldroOverrideActive = false;
    handled = true;
  }
  if (server.hasArg("on")) {
    bool en = (server.arg("on") == "1");
    state.rxTiming.ldroOverrideActive = true;
    state.rxTiming.ldroOverrideValue = en;
    int16_t rc = radio.forceLDRO(en);
    if (rc != RADIOLIB_ERR_NONE) {
      logRadioError("forceLDRO", rc);
      server.send(500, "application/json", "{\"error\":\"Не удалось применить LDRO\"}");
      return;
    }
    state.rxTiming.ldroForced = en;
    handled = true;
  }

  if (!handled) {
    server.send(400, "application/json", "{\"error\":\"Параметры не переданы\"}");
    return;
  }

  configureNarrowbandRxOptions();
  ensureReceiveMode();
  addEvent(String("LDRO ") + (state.rxTiming.ldroOverrideActive
                               ? String(state.rxTiming.ldroOverrideValue ? "принудительно включён" : "принудительно выключен")
                               : String("в режиме AUTO")));
  server.send(200, "application/json", "{\"ok\":true}");
}

// --- API: конфигурация TCXO (вкл/выкл и напряжение) ---
void handleTcxoConfig() {
  const bool enable = server.hasArg("enable") && server.arg("enable") == "1";
  float voltage = 0.0f;
  if (enable) {
    voltage = server.hasArg("v") ? server.arg("v").toFloat() : 1.8f; // типично 1.8 В
    if (voltage < 1.6f || voltage > 3.3f) {
      server.send(400, "application/json", "{\"error\":\"v допустим 1.6..3.3\"}");
      return;
    }
  }

  int16_t rc = radio.setTCXO(enable ? voltage : 0.0f);
  if (rc != RADIOLIB_ERR_NONE) {
    // Отключена строгая проверка TCXO: логируем, но не считаем ошибкой
    logRadioError("setTCXO (ignored)", rc);
  }

  addEvent(String("TCXO ") + (enable ? "включён, V=" + String(voltage, 2) + " В" : "выключен"));
  // Обновим частоты, чтобы минимизировать фазовый сдвиг после смены TCXO
  (void)applyRadioChannel(state.channelIndex);
  ensureReceiveMode();
  server.send(200, "application/json", "{\"ok\":true}");
}

// --- API: проверка статуса TCXO и логирование ---
void handleTcxoCheck() {
  bool configured = (kRadioDefaults.useDio3ForTcxo && kRadioDefaults.tcxoVoltage > 0.0f);
  if (!configured) {
    addEvent(String("Проверка TCXO: выключен (XTAL), преамбула=") + String(kRadioDefaults.preambleLength));
    server.send(200, "application/json",
                String("{\"enabled\":false,\"preamble\":") + String(kRadioDefaults.preambleLength) + "}");
    return;
  }

  int16_t rc = radio.setTCXO(kRadioDefaults.tcxoVoltage, kRadioDefaults.tcxoDelayUs);
  if (rc == RADIOLIB_ERR_NONE) {
    addEvent(String("Проверка TCXO: OK, V=") + String(kRadioDefaults.tcxoVoltage, 2) +
             String(" В, задержка=") + String(static_cast<unsigned long>(kRadioDefaults.tcxoDelayUs)) +
             String(" мкс, преамбула=") + String(kRadioDefaults.preambleLength));
  } else {
    logRadioError("setTCXO (check)", rc);
    addEvent(String("Проверка TCXO: ошибка ") + String(rc));
  }

  String json = String("{\"enabled\":true,\"voltage\":") + String(kRadioDefaults.tcxoVoltage, 2) +
                String(",\"delay_us\":") + String(static_cast<unsigned long>(kRadioDefaults.tcxoDelayUs)) +
                String(",\"preamble\":") + String(kRadioDefaults.preambleLength) +
                String(",\"rc\":") + String(rc) + "}";
  server.send(200, "application/json", json);
}

// --- API: настройка длины фиксированного LoRa-пакета (implicit header) ---
void handleImplicitLenChange() {
  if (!server.hasArg("len")) {
    server.send(400, "application/json", "{\"error\":\"Не указан параметр len\"}");
    return;
  }

  int requested = server.arg("len").toInt();
  if (requested < 1 || requested > static_cast<int>(kFullPacketSize)) {
    server.send(400, "application/json",
                String("{\"error\":\"Длина должна быть в диапазоне 1-") +
                    String(static_cast<unsigned long>(kFullPacketSize)) + "\"}");
    return;
  }

  int16_t code = radio.implicitHeader(static_cast<uint8_t>(requested));
  if (code != RADIOLIB_ERR_NONE) {
    logRadioError("implicitHeader(len)", code);
    server.send(500, "application/json", "{\"error\":\"Ошибка применения implicit header\"}");
    return;
  }

  state.implicitPayloadLength = static_cast<size_t>(requested);
  addEvent(String("Фиксированная длина LoRa-пакета установлена: ") + String(requested) + " байт");
  ensureReceiveMode();
  server.send(200, "application/json", "{\"ok\":true}");
}

// --- API: переключение параметров протокола ---
void handleProtocolToggle() {
  bool handled = false;

  if (server.hasArg("interleaving")) {
    bool enabled = server.arg("interleaving") == "1";
    state.protocol.interleaving = enabled;
    addEvent(String("Интерливинг ") + (enabled ? "включён" : "выключен"));
    handled = true;
  }

  if (server.hasArg("harq")) {
    bool enabled = server.arg("harq") == "1";
    state.protocol.harq = enabled;
    addEvent(String("HARQ ") + (enabled ? "включён" : "выключен"));
    handled = true;
  }

  if (server.hasArg("crc8")) {
    bool enabled = server.arg("crc8") == "1";
    state.protocol.payloadCrc8 = enabled;
    addEvent(String("CRC-8 для DATA ") + (enabled ? "включён (payload=4 байта)" : "выключен"));
    handled = true;
  }

  if (server.hasArg("phyfec")) {
    bool enabled = server.arg("phyfec") == "1";
    if (applyPhyFec(enabled)) {
      state.protocol.phyFec = enabled;
      addEvent(String("PHY FEC ") + (enabled ? "включён (CR=4/7)" : "выключен"));
    } else {
      server.send(500, "application/json", "{\"error\":\"Не удалось переключить FEC\"}");
      return;
    }
    handled = true;
  }

  if (server.hasArg("sym")) {
    bool enabled = server.arg("sym") == "1";
    state.symmetricTxRx = enabled;
    // Пересчитать частоты на текущем канале
    if (!state.fhss.enabled) {
      (void)applyRadioChannel(state.channelIndex);
    }
    addEvent(String("Режим TX=RX ") + (enabled ? "включён (лаборатория)" : "выключен (кросс-диапазон)"));
    handled = true;
  }

  if (!handled) {
    server.send(400, "application/json", "{\"error\":\"Параметры не переданы\"}");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

// --- API: отправка длинного пакета с буквами A-Z ---
void handleSendLongPacket() {
  std::vector<uint8_t> data(kLongPacketSize, 0);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>('A' + (i % 26));
  }
  if (sendPayload(data, String("Длинный пакет (") + String(static_cast<unsigned long>(data.size())) + " байт)")) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Отправка не удалась\"}");
  }
}

// --- API: отправка полного пакета случайных чередующихся байт ---
void handleSendRandomPacket() {
  std::vector<uint8_t> data(kFullPacketSize, 0);
  uint8_t evenByte = static_cast<uint8_t>(random(0, 256));
  uint8_t oddByte = static_cast<uint8_t>(random(0, 256));
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = (i % 2 == 0) ? evenByte : oddByte;
  }
  if (sendPayload(data, "Полный пакет с чередованием случайных байт")) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Отправка не удалась\"}");
  }
}

// --- API: отправка пользовательского сообщения ---
void handleSendCustom() {
  if (!server.hasArg("payload")) {
    server.send(400, "application/json", "{\"error\":\"Не передано поле payload\"}");
    return;
  }
  const String text = server.arg("payload");
  if (text.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"Введите сообщение\"}");
    return;
  }
  std::vector<uint8_t> data(text.length());
  std::memcpy(data.data(), text.c_str(), text.length());
  if (sendPayload(data, String("Пользовательский пакет (" + String(data.size()) + " байт)"))) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Отправка не удалась\"}");
  }
}

// --- API: ответ на неизвестный маршрут ---
void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"Маршрут не найден\"}");
}

// --- Построение HTML главной страницы ---
String buildIndexHtml() {
  String html;
  html.reserve(8192);
  html += F("<!DOCTYPE html><html lang='ru'><head><meta charset='UTF-8'><title>Lotest</title><style>");
  html += F("body{font-family:Arial,sans-serif;margin:0;padding:0;background:#10141a;color:#f0f0f0;}");
  html += F("header{background:#1f2a38;padding:16px 24px;font-size:20px;font-weight:bold;}");
  html += F("main{padding:24px;display:flex;gap:24px;flex-wrap:wrap;}");
  html += F("section{background:#1b2330;border-radius:12px;padding:16px;box-shadow:0 4px 12px rgba(0,0,0,0.3);flex:1 1 320px;}");
  html += F("button,select,input[type=text]{background:#2b3648;border:none;border-radius:8px;color:#f0f0f0;padding:8px 12px;margin:4px 0;}");
  html += F("button{cursor:pointer;transition:background 0.2s;}");
  html += F("button:hover{background:#3b4860;}");
  html += F("label{display:block;margin-top:8px;}");
  html += F("fieldset{border:1px solid #2b3648;border-radius:8px;margin-top:12px;padding:12px;}");
  html += F("legend{padding:0 8px;color:#9fb1d1;}");
  html += F("#log{height:360px;overflow-y:auto;background:#0f1722;border-radius:8px;padding:12px;font-family:monospace;white-space:pre-wrap;}");
  html += F(".message{margin-bottom:8px;padding-bottom:8px;border-bottom:1px solid rgba(255,255,255,0.1);}");
  html += F(".controls button{width:100%;margin-top:8px;}");
  html += F(".status{font-size:14px;color:#9fb1d1;margin-top:8px;}");
  html += F("</style></head><body><header>Lotest — тестирование LoRa + веб</header><main>");

  html += F("<section><h2>Управление радиомодулем</h2><label>Канал банка HOME:</label><select id='channel'>");
  html += buildChannelOptions(state.channelIndex);
  html += F("</select>");
  html += "<label><input type='checkbox' id='power'";
  if (state.highPower) {
    html += " checked";
  }
  html += "> Мощность 22 dBm (выкл — -5 dBm)</label>";
  html += "<label>Фактор расширения:</label><select id='sf'>";
  for (uint8_t sf = 5; sf <= 12; ++sf) {
    html += "<option value='" + String(sf) + "'";
    if (state.selectedSpreadingFactor == sf) {
      html += " selected";
    }
    html += ">SF" + String(sf) + "</option>";
  }
  html += "</select>";
  html += "<label><input type='checkbox' id='rxboost'";
  if (state.rxBoostedGain) {
    html += " checked";
  }
  html += "> Усиленный приём (LNA Boost)</label>";
  html += "<label><input type='checkbox' id='fhss'";
  if (state.fhss.enabled) {
    html += " checked";
  }
  html += "> Программный FHSS</label>";
  html += "<label><input type='checkbox' id='sym'";
  if (state.symmetricTxRx) {
    html += " checked";
  }
  html += "> Лабораторный режим: TX=RX</label>";
  html += F("<fieldset><legend>Надёжность</legend>");
  html += "<label><input type='checkbox' id='interleaving'";
  if (state.protocol.interleaving) {
    html += " checked";
  }
  html += "> Интерливинг (шаг 4)</label>";
  html += "<label><input type='checkbox' id='harq'";
  if (state.protocol.harq) {
    html += " checked";
  }
  html += "> HARQ (адаптивный RS(15,11))</label>";
  html += "<label><input type='checkbox' id='phyfec'";
  if (state.protocol.phyFec) {
    html += " checked";
  }
  html += "> PHY FEC (LoRa CR=4/7)</label>";
  // Принудительное управление LDRO: чекбокс включает LDRO, снятие — авто
  html += "<label><input type='checkbox' id='ldro'";
  if (state.rxTiming.ldroOverrideActive && state.rxTiming.ldroOverrideValue) {
    html += " checked";
  }
  html += "> LDRO (принудительно; снято = AUTO)</label>";
  html += "<label><input type='checkbox' id='crc8'";
  if (state.protocol.payloadCrc8) {
    html += " checked";
  }
  html += "> CRC-8 на DATA (payload=4 байта)</label>";
  // Параметр: длина фиксированного LoRa-пакета (implicit header)
  html += "<label>Длина фиксированного пакета (implicit):</label>";
  html += "<div style='display:flex;gap:8px;align-items:center;'>";
  html += String("<input type='number' id='implen' min='1' max='") +
          String(static_cast<unsigned long>(kFullPacketSize)) + "' value='" +
          String(static_cast<unsigned long>(state.implicitPayloadLength)) + "'>";
  html += "<button id='implenApply'>Применить</button></div>";
  html += F("</fieldset>");
  html += F("<div class='status' id='status'></div><div class='controls'>");
  html += F("<button id='sendLong'>Отправить длинный пакет 124 байта</button>");
  html += F("<button id='sendRandom'>Отправить полный пакет</button>");
  html += F("<label>Пользовательский пакет (текст):</label><input type='text' id='custom' placeholder='Введите сообщение'>");
  html += F("<button id='sendCustom'>Отправить пользовательский пакет</button>");
  html += F("</div></section>");

  html += F("<section><h2>Журнал событий</h2><div id='log'></div></section></main><script>");
  html += F("const logEl=document.getElementById('log');const channelSel=document.getElementById('channel');const powerCb=document.getElementById('power');const sfSelect=document.getElementById('sf');const rxboostCb=document.getElementById('rxboost');const fhssCb=document.getElementById('fhss');const symCb=document.getElementById('sym');const interCb=document.getElementById('interleaving');const harqCb=document.getElementById('harq');const phyFecCb=document.getElementById('phyfec');const crc8Cb=document.getElementById('crc8');const ldroCb=document.getElementById('ldro');const statusEl=document.getElementById('status');let lastId=0;");
  html += F("function appendLog(entry){const div=document.createElement('div');div.className='message';div.textContent=entry.text;if(entry.color){div.style.color=entry.color;}logEl.appendChild(div);logEl.scrollTop=logEl.scrollHeight;}");
  html += F("async function refreshLog(){try{const resp=await fetch(`/api/log?after=${lastId}`);if(!resp.ok)return;const data=await resp.json();data.events.forEach(evt=>{appendLog(evt);lastId=evt.id;});}catch(e){console.error(e);}}");
  html += F("async function postForm(url,body){const resp=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(body)});if(!resp.ok){const err=await resp.json().catch(()=>({error:'Неизвестная ошибка'}));throw new Error(err.error||'Ошибка');}}");
  html += F("async function updateProtocol(field,value){const payload={};payload[field]=value?'1':'0';try{await postForm('/api/protocol',payload);statusEl.textContent='Настройки протокола обновлены';refreshLog();}catch(e){statusEl.textContent=e.message;}}");
  html += F("channelSel.addEventListener('change',async()=>{try{await postForm('/api/channel',{channel:channelSel.value});statusEl.textContent='Канал применён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("powerCb.addEventListener('change',async()=>{try{await postForm('/api/power',{high:powerCb.checked?'1':'0'});statusEl.textContent='Мощность обновлена';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("rxboostCb.addEventListener('change',async()=>{try{await postForm('/api/rxboost',{enable:rxboostCb.checked?'1':'0'});statusEl.textContent='Усиленный приём обновлён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("sfSelect.addEventListener('change',async()=>{try{await postForm('/api/sf',{sf:sfSelect.value});statusEl.textContent='Фактор расширения обновлён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("document.getElementById('implenApply').addEventListener('click',async()=>{const val=document.getElementById('implen').value;try{await postForm('/api/implicit',{len:val});statusEl.textContent='Длина фиксированного пакета обновлена';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("fhssCb.addEventListener('change',async()=>{try{await postForm('/api/fhss',{enable:fhssCb.checked?'1':'0'});statusEl.textContent='Статус FHSS обновлён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("symCb.addEventListener('change',async()=>{try{await postForm('/api/protocol',{sym:symCb.checked?'1':'0'});statusEl.textContent='Режим TX=RX обновлён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("interCb.addEventListener('change',()=>{updateProtocol('interleaving',interCb.checked);});");
  html += F("harqCb.addEventListener('change',()=>{updateProtocol('harq',harqCb.checked);});");
  html += F("phyFecCb.addEventListener('change',()=>{updateProtocol('phyfec',phyFecCb.checked);});");
  html += F("ldroCb.addEventListener('change',async()=>{try{if(ldroCb.checked){await postForm('/api/ldro',{on:'1'});}else{await postForm('/api/ldro',{auto:'1'});}statusEl.textContent='LDRO обновлён';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("crc8Cb.addEventListener('change',()=>{updateProtocol('crc8',crc8Cb.checked);});");
  html += F("document.getElementById('sendLong').addEventListener('click',async()=>{try{await postForm('/api/send/long',{});statusEl.textContent='Длинный пакет отправлен';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("document.getElementById('sendRandom').addEventListener('click',async()=>{try{await postForm('/api/send/random',{});statusEl.textContent='Полный пакет отправлен';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("document.getElementById('sendCustom').addEventListener('click',async()=>{const input=document.getElementById('custom');const payload=input.value;if(!payload.trim()){statusEl.textContent='Введите сообщение';return;}try{await postForm('/api/send/custom',{payload});statusEl.textContent='Пользовательский пакет отправлен';refreshLog();}catch(e){statusEl.textContent=e.message;}});");
  html += F("setInterval(refreshLog,1500);refreshLog();");
  html += F("</script></body></html>");
  return html;
}

// --- Формирование HTML-опций для списка каналов ---
String buildChannelOptions(uint8_t selected) {
  String html;
  for (uint8_t i = 0; i < kHomeBankSize; ++i) {
    html += "<option value='" + String(i) + "'";
    if (i == selected) {
      html += " selected";
    }
    html += ">#" + String(i) + " — RX " + String(frequency_tables::RX_HOME[i], 3) + " МГц / TX " + String(frequency_tables::TX_HOME[i], 3) + " МГц</option>";
  }
  return html;
}

// --- Экранирование строки для JSON ---
String escapeJson(const String& value) {
  String out;
  out.reserve(value.length() + 4);
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value.charAt(i);
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<uint8_t>(c) < 32) {
          char buf[7];
          std::snprintf(buf, sizeof(buf), "\\u%04X", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

// --- Применение радиоканала ---
bool tuneToHomeChannel(uint8_t newIndex, bool updateSelection, float offsetKhz) {
  if (newIndex >= kHomeBankSize) {
    return false;
  }

  const float offsetMhz = offsetKhz / 1000.0f;                        // перевод смещения из кГц в МГц
  const float rx = frequency_tables::RX_HOME[newIndex] + offsetMhz;   // итоговая частота приёма
  float tx = frequency_tables::TX_HOME[newIndex] + offsetMhz;         // итоговая частота передачи
  if (state.symmetricTxRx) {
    tx = rx;                                                          // лабораторный режим: одна частота для TX/RX
  }

  const int16_t rxState = radio.setFrequency(rx);
  if (rxState != RADIOLIB_ERR_NONE) {
    logRadioError("setFrequency(RX)", rxState);
    return false;
  }

  state.currentRxFreq = rx;
  state.currentTxFreq = tx;

  if (updateSelection) {
    state.channelIndex = newIndex;
  }

  return ensureReceiveMode();
}

bool applyRadioChannel(uint8_t newIndex) {
  if (!tuneToHomeChannel(newIndex, true, 0.0f)) {
    return false;
  }

  state.fhss.previousChannel = state.channelIndex;
  state.fhss.currentHopChannel = state.channelIndex;
  state.fhss.currentHopOffsetKhz = 0.0f;
  state.fhss.lastHopMillis = millis();
  state.fhss.baseChannelForTenKhz = state.channelIndex;
  rebuildFhssSequence();
  return true;
}

void initFhssDefaults() {
  state.fhss.enabled = false;
  state.fhss.limitToTenKhz = true;                // по умолчанию прыгаем в пределах ±5 кГц от канала
  state.fhss.previousChannel = state.channelIndex;
  state.fhss.baseChannelForTenKhz = state.channelIndex;
  rebuildFhssSequence();
}

void rebuildFhssSequence() {
  state.fhss.baseChannelForTenKhz = state.channelIndex;               // актуализируем базовый канал для узкополосного режима
  const uint8_t baseChannel = state.fhss.baseChannelForTenKhz;

  if (state.fhss.limitToTenKhz) {
    constexpr std::array<float, 7> kTenKhzOffsets = {0.0f, -5.0f, 5.0f, -3.0f, 3.0f, -1.0f, 1.0f};
    const size_t count = std::min(kTenKhzOffsets.size(), state.fhss.hopSequence.size());
    state.fhss.hopCount = count;
    for (size_t i = 0; i < count; ++i) {
      state.fhss.hopSequence[i].channel = baseChannel;                // все прыжки вокруг выбранного канала
      state.fhss.hopSequence[i].offsetKhz = kTenKhzOffsets[i];        // смещение в пределах ±5 кГц
    }
  } else {
    const size_t count = std::min(static_cast<size_t>(kHomeBankSize), state.fhss.hopSequence.size());
    state.fhss.hopCount = count;
    for (size_t i = 0; i < count; ++i) {
      state.fhss.hopSequence[i].channel = static_cast<uint8_t>(i);    // классический перебор по всему банку HOME
      state.fhss.hopSequence[i].offsetKhz = 0.0f;                     // без дополнительного смещения
    }
  }

  state.fhss.currentHopIndex = 0;
  state.fhss.nextHopIndex = (state.fhss.hopCount > 1) ? 1 : 0;
  state.fhss.currentHopChannel = baseChannel;
  state.fhss.currentHopOffsetKhz = 0.0f;
  state.fhss.lastHopMillis = millis();
}

String formatFhssHopDescription(uint8_t channel, float offsetKhz) {
  String text = String("канал HOME #") + String(channel);                 // базовое описание канала
  if (std::fabs(offsetKhz) > 0.0005f) {
    text += String(" (смещение ") + String(offsetKhz, 2) + F(" кГц)");   // отображаем смещение, если оно заметно
  } else {
    text += F(" (без смещения)");
  }
  return text;
}

bool applyFhssHopByIndex(size_t hopIndex) {
  if (state.fhss.hopCount == 0) {
    return false;
  }
  if (hopIndex >= state.fhss.hopCount) {
    hopIndex %= state.fhss.hopCount;
  }

  const FhssHop& hop = state.fhss.hopSequence[hopIndex];
  if (!tuneToHomeChannel(hop.channel, false, hop.offsetKhz)) {
    return false;
  }

  state.fhss.currentHopIndex = hopIndex;
  state.fhss.currentHopChannel = hop.channel;
  state.fhss.currentHopOffsetKhz = hop.offsetKhz;
  state.fhss.nextHopIndex = (hopIndex + 1U) % state.fhss.hopCount;
  state.fhss.lastHopMillis = millis();
  return true;
}

bool setFhssEnabled(bool enable) {
  if (enable == state.fhss.enabled) {
    return true;
  }

  if (enable) {
    if (state.fhss.hopCount == 0) {
      initFhssDefaults();
    }

    state.fhss.previousChannel = state.channelIndex;
    rebuildFhssSequence();
    size_t startIndex = 0;
    float bestOffset = std::numeric_limits<float>::max();
    for (size_t i = 0; i < state.fhss.hopCount; ++i) {
      const FhssHop& hop = state.fhss.hopSequence[i];
      if (hop.channel != state.channelIndex) {
        continue;
      }
      const float absOffset = std::fabs(hop.offsetKhz);
      if (absOffset < bestOffset) {
        bestOffset = absOffset;
        startIndex = i;
        if (absOffset < 0.001f) {
          break;                                                   // нашли практически нулевое смещение
        }
      }
    }

    state.fhss.enabled = true;
    if (!applyFhssHopByIndex(startIndex)) {
      state.fhss.enabled = false;
      return false;
    }
    return true;
  }

  state.fhss.enabled = false;
  return applyRadioChannel(state.fhss.previousChannel);
}

void updateFhss() {
  (void)advanceFhssIfDue();
}

bool advanceFhssIfDue() {
  if (!state.fhss.enabled || state.fhss.suspendDepth > 0) {
    return false;
  }

  if (state.fhss.hopCount <= 1) {
    return false;
  }

  const unsigned long now = millis();
  if (state.fhss.lastHopMillis == 0) {
    state.fhss.lastHopMillis = now;
    return false;
  }

  const unsigned long elapsed = now - state.fhss.lastHopMillis;
  if (elapsed < state.fhss.dwellTimeMs) {
    return false;
  }

  const size_t nextIndex = state.fhss.nextHopIndex % state.fhss.hopCount;
  if (!applyFhssHopByIndex(nextIndex)) {
    state.fhss.enabled = false;
    addEvent(F("FHSS отключён: не удалось переключить частоту"));
    if (!applyRadioChannel(state.fhss.previousChannel)) {
      addEvent(F("Не удалось вернуть фиксированный канал после ошибки FHSS"));
    }
    return false;
  }

  return true;
}

// --- Настройка мощности передачи ---
// --- Настройка длительного окна приёма для узкой полосы ---
void configureNarrowbandRxOptions() {
  const float bandwidthKhz = state.radioBandwidthKhz;
  const uint8_t sf = state.currentSpreadingFactor;
  const float bandwidthHz = bandwidthKhz * 1000.0f;
  const float symbolDurationMs =
      (bandwidthHz > 0.0f) ? (static_cast<float>(1UL << sf) / bandwidthHz) * 1000.0f : 0.0f;
  const bool narrowBandwidth = (bandwidthKhz <= 10.0f);
  const bool veryNarrowBandwidth = (bandwidthKhz <= 8.0f);
  // Поддерживаем только современный API RadioLib

  // Масштабируем минимальную преамбулу от SF (а не только от BW):
  // SF5-6 → ≥12, SF7 → ≥16, SF8-9 → ≥24, SF10-12 → ≥32
  uint16_t targetPreamble = state.rxTiming.preambleSymbols;
  if (sf >= 11) {
    targetPreamble = std::max<uint16_t>(targetPreamble, static_cast<uint16_t>(32));
  } else if (sf >= 9) {
    targetPreamble = std::max<uint16_t>(targetPreamble, static_cast<uint16_t>(24));
  } else if (sf >= 7) {
    targetPreamble = std::max<uint16_t>(targetPreamble, static_cast<uint16_t>(16));
  } else {
    targetPreamble = std::max<uint16_t>(targetPreamble, static_cast<uint16_t>(12));
  }

  if (targetPreamble != state.rxTiming.preambleSymbols) {
    const int16_t code = radio.setPreambleLength(targetPreamble);
    if (code == RADIOLIB_ERR_NONE) {
      state.rxTiming.preambleSymbols = targetPreamble;
      logRxTimingEvent(String("Преамбула увеличена до ") + String(targetPreamble) +
                       String(" символов для BW=") + String(bandwidthKhz, 2) + " кГц");
    } else {
      logRadioError("setPreambleLength", code);
    }
  }

  // Управление остановкой RX-таймера по преамбуле отсутствует в RadioLib API SX126x
  const bool desiredStop = narrowBandwidth;
  if (desiredStop != state.rxTiming.stopTimerOnPreamble) {
    state.rxTiming.stopTimerOnPreamble = desiredStop;
    if (!state.rxTiming.reportedMissingStopTimerSupport) {
      state.rxTiming.reportedMissingStopTimerSupport = true;
      logRxTimingEvent(F("StopRxTimerOnPreamble недоступен в RadioLib — пропускаем"));
    }
  }

  const bool computedLdro = (symbolDurationMs >= 16.0f);
  const bool needLdro = state.rxTiming.ldroOverrideActive ? state.rxTiming.ldroOverrideValue : computedLdro;
  if (state.rxTiming.ldroOverrideActive) {
    if (needLdro != state.rxTiming.ldroForced) {
      const int16_t code = radio.forceLDRO(needLdro);
      if (code == RADIOLIB_ERR_NONE) {
        state.rxTiming.ldroForced = needLdro;
        String src = String(" (manual)");
        logRxTimingEvent(String("LDRO ") + (needLdro ? "включён" : "выключен") +
                         String(" (Ts=") + String(symbolDurationMs, 2) + " мс)" + src);
      } else {
        logRadioError("forceLDRO", code);
      }
    }
  } else {
    // AUTO режим: библиотека сама включает LDRO при Ts>=16 мс
    const int16_t code = radio.autoLDRO();
    if (code == RADIOLIB_ERR_NONE) {
      state.rxTiming.ldroForced = needLdro;
      logRxTimingEvent(String("LDRO AUTO, Ts=") + String(symbolDurationMs, 2) + " мс → " + (needLdro ? "on" : "off"));
    } else {
      logRadioError("autoLDRO", code);
    }
  }

  const bool needExtendedTimeout = narrowBandwidth || (symbolDurationMs >= 10.0f);
  const bool useContinuous = needExtendedTimeout && state.rxTiming.forceContinuousRx;
  const float targetWindowMs = 2000.0f + 150.0f; // 2 секунды окна + запас
  uint32_t timeoutSymbols = 0;
  if (symbolDurationMs > 0.0f) {
    timeoutSymbols = static_cast<uint32_t>(std::ceil(targetWindowMs / symbolDurationMs));
    if (timeoutSymbols == 0U) {
      timeoutSymbols = 1U;
    }
  }
  const uint32_t appliedSymbols = useContinuous ? 0U : timeoutSymbols;
  const float timeoutSeconds = useContinuous ? 0.0f : (symbolDurationMs * timeoutSymbols) / 1000.0f;

  // SX126x в RadioLib не имеет отдельного API setRxTimeout/symb timeout; 
  // будем просто сохранять желаемые значения и использовать startReceive() при необходимости
  const bool symbolsChangeNeeded =
      (appliedSymbols != state.rxTiming.rxTimeoutSymbols) || (state.rxTiming.rxContinuous != useContinuous);
  if (symbolsChangeNeeded) {
    state.rxTiming.rxTimeoutSymbols = appliedSymbols;
    state.rxTiming.rxContinuous = useContinuous;
    if (useContinuous) {
      logRxTimingEvent(F("RX в непрерывном режиме (timeout=INF)"));
    } else {
      logRxTimingEvent(String("Желаемый RX тайм-аут ≈ ") + String(timeoutSeconds, 2) + " с (будет применён при startReceive)");
    }
  }

  // Таймаут поиска заголовка масштабируем от SF: для SF≥8 даём больший запас символов
  const uint8_t headerReserve = (sf >= 8) ? 16U : 8U;
  const uint16_t desiredSymbTimeout = static_cast<uint16_t>(
      std::min<uint32_t>(255U, static_cast<uint32_t>(targetPreamble + headerReserve)));
  if (desiredSymbTimeout != 0 && desiredSymbTimeout != state.rxTiming.symbTimeout) {
    state.rxTiming.symbTimeout = desiredSymbTimeout;
    if (!state.rxTiming.reportedMissingSymbTimeoutSupport) {
      state.rxTiming.reportedMissingSymbTimeoutSupport = true;
      logRxTimingEvent(F("LoRaSymbNumTimeout недоступен в RadioLib — пропускаем"));
    }
  }
}

// --- Настройка мощности передачи ---
// --- Вспомогательные функции для временной заморозки FHSS ---
void fhssSuspend() {
  if (state.fhss.suspendDepth < 0xFF) {
    ++state.fhss.suspendDepth;
  }
}

void fhssResume() {
  if (state.fhss.suspendDepth > 0) {
    --state.fhss.suspendDepth;
  }
  if (state.fhss.suspendDepth == 0) {
    state.fhss.lastHopMillis = millis(); // сброс таймера, чтобы не прыгнуть сразу после резюма
  }
}

// --- Настройка мощности передачи ---
bool applyRadioPower(bool highPower) {
  int8_t targetDbm = highPower ? kHighPowerDbm : kLowPowerDbm;
  int16_t result = radio.setOutputPower(targetDbm);
  if (result != RADIOLIB_ERR_NONE) {
    logRadioError("setOutputPower", result);
    return false;
  }
  return true;
}

// --- Настройка фактора расширения SF ---
bool applySpreadingFactor(uint8_t spreadingFactor) {
  if (spreadingFactor < 5 || spreadingFactor > 12) {
    addEvent(String("Недопустимый SF: ") + String(spreadingFactor));
    return false;
  }

  int16_t result = radio.setSpreadingFactor(spreadingFactor);
  if (result != RADIOLIB_ERR_NONE) {
    logRadioError("setSpreadingFactor", result);
    return false;
  }

  state.selectedSpreadingFactor = spreadingFactor;
  state.currentSpreadingFactor = spreadingFactor;
  configureNarrowbandRxOptions();
  return true;
}

// --- Переключение PHY FEC ---
bool applyPhyFec(bool enable) {
  uint8_t targetCr = enable ? 7 : kDefaultCodingRate;
  int16_t result = radio.setCodingRate(targetCr);
  if (result != RADIOLIB_ERR_NONE) {
    logRadioError("setCodingRate", result);
    return false;
  }
  return true;
}

// --- Гарантируем, что радио ожидает приём ---
bool ensureReceiveMode() {
  int16_t stateCode = radio.startReceive();
  if (stateCode != RADIOLIB_ERR_NONE) {
    logRadioError("startReceive", stateCode);
    return false;
  }
  const unsigned long now = millis();
  state.rxTiming.lastSetRxMs = now;

  bool shouldLog = true;
  const unsigned long kSetRxLogIntervalMs = (state.fhss.enabled && state.fhss.hopCount > 1U) ? 1000UL : 20UL;
  if (state.rxTiming.lastSetRxLogMs != 0U) {
    const unsigned long delta = now - state.rxTiming.lastSetRxLogMs;
    if (delta < kSetRxLogIntervalMs) {
      shouldLog = false;
    }
  }

  if (shouldLog) {
    state.rxTiming.lastSetRxLogMs = now;
    logRxTimingEvent(F("Команда SetRx отправлена (переход в ожидание пакета)"));
  }
  return true;
}


// --- Вывод кодов ошибок RadioLib в лог ---
void logRadioError(const String& context, int16_t code) {
  String message = String("RadioLib ошибка ") + context + " => " + String(code);
  switch (code) {
    case RADIOLIB_ERR_SPI_CMD_FAILED:
      message += " (SPI команда не выполнена — проверьте питание и линии CS/CLK/MISO/MOSI/BUSY)";
      break;
    default:
      break;
  }
  addEvent(message);
}
// --- Отправка подготовленного буфера ---
bool sendPayload(const std::vector<uint8_t>& payload, const String& context) {
  if (payload.empty()) {
    addEvent(context + ": пустой буфер");
    return false;
  }

  // На время передачи всего сообщения замораживаем FHSS, чтобы частота не менялась
  struct FhssScope { FhssScope(){ fhssSuspend(); } ~FhssScope(){ fhssResume(); } } fhssGuard;

  addEvent(context + ": " + formatByteArray(payload) + " | \"" + formatTextPayload(payload) + "\"");

  const uint16_t crc = crc16Ccitt(payload.data(), payload.size());
  uint16_t announcedParts = 0;
  auto transportPayload = buildTransportPayloadWithPartMarkers(payload,
                                                               state.protocol.payloadCrc8,
                                                               announcedParts);
  addEvent(String("Полезная нагрузка разбита на ") +
           String(static_cast<unsigned long>(announcedParts)) + " частей");
  auto blocks = splitPayloadIntoBlocks(transportPayload, state.protocol.payloadCrc8);
  if (blocks.empty()) {
    addEvent("Не удалось подготовить DATA-пакеты для передачи");
    return false;
  }

  // Назначаем идентификатор сообщения заранее
  const uint8_t msgId = static_cast<uint8_t>(state.nextMessageId++ & 0x0F);

  // Перед началом окна отправим уникальный START-кадр (для данного msgId),
  // чтобы приёмник сбросил состояние сборки именно для этого сообщения
  {
    auto startFrame = buildStartFrame(msgId);
    if (!transmitFrame(startFrame, F("START"))) {
      return false;
    }
    waitInterFrameDelay();
  }

  size_t totalBlocks = blocks.size();
  size_t offset = 0;
  uint16_t windowBaseSeq = composeSeq(msgId, 0);
  bool harqUsed = false;

  while (offset < totalBlocks) {
    const uint8_t windowSize = static_cast<uint8_t>(std::min(kArqWindowSize, totalBlocks - offset));
    if (!sendDataWindow(blocks, offset, windowBaseSeq, windowSize)) {
      return false;
    }

    uint16_t missingBitmap = 0;
    bool needParity = false;
    if (!waitForAck(windowBaseSeq, windowSize, missingBitmap, needParity)) {
      addEvent("ACK не получен — считаем потерянным всё окно");
      missingBitmap = static_cast<uint16_t>((1U << windowSize) - 1U);
    }

    uint8_t retries = 0;
    while (missingBitmap != 0 && retries < 3) {
      const uint8_t missingCount = static_cast<uint8_t>(__builtin_popcount(missingBitmap));
      addEvent(String("Повторная отправка ") + String(static_cast<unsigned long>(missingCount)) +
               " пакетов по BITMAP16 0x" + String(missingBitmap, 16));
      retransmitMissing(blocks, offset, windowBaseSeq, missingBitmap);
      if (!waitForAck(windowBaseSeq, windowSize, missingBitmap, needParity)) {
        addEvent("ACK после переотправки не пришёл — считаем окно потерянным");
        missingBitmap = static_cast<uint16_t>((1U << windowSize) - 1U);
      }
      ++retries;
    }

    if (missingBitmap != 0) {
      addEvent("Не удалось доставить все DATA-пакеты окна — остановка передачи");
      return false;
    }

    if (needParity) {
      if (state.protocol.harq) {
        harqUsed = true;
        addEvent("Получен запрос HARQ: отправляем дополнительный паритет");
        const ParityPacket parity = computeWindowParity(blocks, offset, windowSize);
        auto parityFrame = buildDataFrame(static_cast<uint16_t>(windowBaseSeq + windowSize - 1),
                                          parity.block,
                                          true,
                                          true,
                                          parity.lengthXor);
        if (!transmitFrame(parityFrame, F("PARITY retry"))) {
          return false;
        }
        waitInterFrameDelay();
        if (!waitForAck(windowBaseSeq, windowSize, missingBitmap, needParity)) {
          addEvent("ACK после HARQ не получен — прекращаем передачу окна");
          return false;
        }
        if (missingBitmap != 0) {
          addEvent("HARQ не помог — окно всё ещё неполное");
          return false;
        }
      } else {
        addEvent("Получен запрос HARQ, но HARQ отключён");
      }
    }

    offset += windowSize;
    windowBaseSeq = static_cast<uint16_t>(windowBaseSeq + windowSize);
  }

  state.nextTxSequence = 0;

  auto finFrame = buildFinFrame(static_cast<uint16_t>(payload.size()), crc, harqUsed, msgId);
  if (!transmitFrame(finFrame, F("FIN"))) {
    return false;
  }

  waitInterFrameDelay();
  return true;
}

// --- Передача окна DATA-пакетов ---
bool sendDataWindow(const std::vector<DataBlock>& blocks,
                    size_t offset,
                    uint16_t baseSeq,
                    uint8_t windowSize) {
  std::vector<size_t> order(windowSize);
  std::iota(order.begin(), order.end(), 0U);

  if (state.protocol.interleaving && windowSize > 1) {
    std::vector<size_t> interleaved;
    for (size_t start = 0; start < kInterleaverStep && start < windowSize; ++start) {
      for (size_t pos = start; pos < windowSize; pos += kInterleaverStep) {
        interleaved.push_back(pos);
      }
    }
    auto it = std::find(interleaved.begin(), interleaved.end(), windowSize - 1);
    if (it != interleaved.end()) {
      interleaved.erase(it);
      interleaved.push_back(windowSize - 1);
    }
    order = std::move(interleaved);
  }

  const bool parityEnabled = state.protocol.harq && windowSize > 0;

  for (size_t idx : order) {
    if (idx >= windowSize) {
      continue;
    }
    const size_t blockIndex = offset + idx;
    if (blockIndex >= blocks.size()) {
      break;
    }
    const bool ackRequest = (!parityEnabled && idx == windowSize - 1);
    auto frame = buildDataFrame(static_cast<uint16_t>(baseSeq + idx),
                                blocks[blockIndex],
                                ackRequest,
                                false);
    if (!transmitFrame(frame, ackRequest ? F("DATA (ACK)") : F("DATA"))) {
      return false;
    }
    waitInterFrameDelay();
  }

  if (parityEnabled && windowSize > 0) {
    const ParityPacket parity = computeWindowParity(blocks, offset, windowSize);
    auto parityFrame = buildDataFrame(static_cast<uint16_t>(baseSeq + windowSize - 1),
                                      parity.block,
                                      true,
                                      true,
                                      parity.lengthXor);
    if (!transmitFrame(parityFrame, F("PARITY"))) {
      return false;
    }
    waitInterFrameDelay();
  }

  return true;
}

// --- Ожидание ACK ---
bool waitForAck(uint16_t baseSeq, uint8_t windowSize, uint16_t& missingBitmap, bool& needParity) {
  const unsigned long timeoutMs = computeAckTimeoutMs(windowSize);
  const unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    processRadioEvents();
    if (state.pendingAck.hasValue && state.pendingAck.baseSeq == baseSeq) {
      missingBitmap = state.pendingAck.missingBitmap;
      needParity = state.pendingAck.needParity;
      if (state.pendingAck.reportedWindow != windowSize) {
        addEvent(String("ACK сообщил окно ") +
                 String(static_cast<unsigned long>(state.pendingAck.reportedWindow)) +
                 " вместо ожидаемых " + String(windowSize));
      }
      state.pendingAck.hasValue = false;
      updateAckTiming(millis() - start);
      return true;
    }
#if defined(ARDUINO)
    delay(3);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
#endif
  }
  handleAckTimeoutExpansion();
  return false;
}

// --- Переотправка потерянных пакетов ---
void retransmitMissing(const std::vector<DataBlock>& blocks,
                       size_t offset,
                       uint16_t baseSeq,
                       uint16_t missingBitmap) {
  int8_t lastBitNeedingAck = -1;
  for (uint8_t bit = 0; bit < kBitmapWidth; ++bit) {
    if ((missingBitmap & (1U << bit)) == 0U) {
      continue;
    }
    const size_t blockIndex = offset + bit;
    if (blockIndex >= blocks.size()) {
      continue;
    }
    lastBitNeedingAck = static_cast<int8_t>(bit);
  }

  for (uint8_t bit = 0; bit < kBitmapWidth; ++bit) {
    if ((missingBitmap & (1U << bit)) == 0U) {
      continue;
    }
    const size_t blockIndex = offset + bit;
    if (blockIndex >= blocks.size()) {
      continue;
    }
    const bool ackRequest = (lastBitNeedingAck >= 0)
                                ? (bit == static_cast<uint8_t>(lastBitNeedingAck))
                                : (bit == 0);
    auto frame = buildDataFrame(static_cast<uint16_t>(baseSeq + bit), blocks[blockIndex], ackRequest, false);
    transmitFrame(frame, ackRequest ? F("DATA retry (ACK)") : F("DATA retry"));
    waitInterFrameDelay();
  }
}

// --- Непосредственная передача одного кадра ---
bool transmitFrame(const std::array<uint8_t, kFixedFrameSize>& frame, const String& context) {
  std::vector<uint8_t> bytes(frame.begin(), frame.end());
  addEvent(String("→ ") + context + ": " + formatByteArray(bytes));

  if (state.fhss.enabled) {
    advanceFhssIfDue();
  }
  // Если TX и RX частоты совпадают, не перенастраиваем PLL каждый раз
  const bool sameTxRx = (std::fabs(state.currentTxFreq - state.currentRxFreq) < 0.0001f);
  if (!sameTxRx) {
    int16_t freqState = radio.setFrequency(state.currentTxFreq);
    if (freqState != RADIOLIB_ERR_NONE) {
      logRadioError("setFrequency(TX)", freqState);
      return false;
    }
  }

  // Количество повторов передачи (1 или 3)
  const uint8_t repeats = state.triple.enabledTx ? 3 : 1;
  for (uint8_t i = 0; i < repeats; ++i) {
    if (i > 0) {
      addEvent(String("→ повтор ") + String(static_cast<unsigned long>(i + 1)) + "/" + String(static_cast<unsigned long>(repeats)));
    }
    int16_t result = radio.transmit(const_cast<uint8_t*>(frame.data()), kFixedFrameSize);
    if (result != RADIOLIB_ERR_NONE) {
      logRadioError("transmit", result);
      if (!sameTxRx) {
        (void)radio.setFrequency(state.currentRxFreq);
      }
      ensureReceiveMode();
      return false;
    }
    // Между повторами выдерживаем короткую паузу
    if (i + 1 < repeats) {
      waitInterFrameDelay();
    }
  }

  if (!sameTxRx) {
    int16_t backState = radio.setFrequency(state.currentRxFreq);
    if (backState != RADIOLIB_ERR_NONE) {
      logRadioError("setFrequency(RX restore)", backState);
      return false;
    }
  }

  return ensureReceiveMode();
}

// --- Построение DATA-кадра ---
std::array<uint8_t, kFixedFrameSize> buildDataFrame(uint16_t seq,
                                                    const DataBlock& block,
                                                    bool ackRequest,
                                                    bool isParity,
                                                    uint8_t parityLengthXor) {
  std::array<uint8_t, kFixedFrameSize> frame{};

  const bool hasCrc = state.protocol.payloadCrc8;
  uint8_t lengthField = block.dataLength;
  if (!isParity && hasCrc && lengthField < kFramePayloadSize) {
    ++lengthField; // добавляем байт CRC-8 только для DATA
  }
  if (isParity) {
    lengthField = std::min<uint8_t>(parityLengthXor & 0x0FU, 0x0FU); // для паритета поле несёт XOR длины
  }
  uint8_t flags = static_cast<uint8_t>((lengthField << kDataFlagLengthShift) & kDataFlagLengthMask);
  if (ackRequest) {
    flags |= kDataFlagAckRequest;
  }
  if (isParity) {
    flags |= kDataFlagIsParity;
  }

  frame[0] = static_cast<uint8_t>(kFrameTypeData | flags);
  frame[1] = static_cast<uint8_t>(seq & 0xFFU);
  frame[2] = static_cast<uint8_t>((seq >> 8) & 0xFFU);
  std::copy(block.bytes.begin(), block.bytes.end(), frame.begin() + 3);
  return frame;
}

// --- Построение ACK-кадра ---
std::array<uint8_t, kFixedFrameSize> buildAckFrame(uint16_t baseSeq,
                                                   uint16_t missingBitmap,
                                                   bool needParity,
                                                   uint8_t windowSize,
                                                   uint8_t ackType) {
  std::array<uint8_t, kFixedFrameSize> frame{};
  frame[0] = static_cast<uint8_t>(kFrameTypeAck | (needParity ? kAckFlagNeedParity : 0));
  frame[1] = static_cast<uint8_t>(baseSeq & 0xFFU);
  frame[2] = static_cast<uint8_t>((baseSeq >> 8) & 0xFFU);
  frame[3] = static_cast<uint8_t>(missingBitmap & 0xFFU);
  frame[4] = static_cast<uint8_t>((missingBitmap >> 8) & 0xFFU);
  frame[5] = windowSize;
  frame[6] = ackType;
  return frame;
}

// --- Построение FIN-кадра ---
std::array<uint8_t, kFixedFrameSize> buildFinFrame(uint16_t length,
                                                   uint16_t crc,
                                                   bool harqUsed,
                                                   uint8_t msgId) {
  std::array<uint8_t, kFixedFrameSize> frame{};
  frame[0] = static_cast<uint8_t>(kFrameTypeFin | (harqUsed ? kFinFlagHarqUsed : 0));
  frame[1] = static_cast<uint8_t>(length & 0xFFU);
  frame[2] = static_cast<uint8_t>((length >> 8) & 0xFFU);
  frame[3] = static_cast<uint8_t>(crc & 0xFFU);
  frame[4] = static_cast<uint8_t>((crc >> 8) & 0xFFU);
  frame[5] = static_cast<uint8_t>(msgId & 0x0F);  // в FIN передаём идентификатор сообщения
  return frame;
}

// --- Построение управляющего START-кадра ---
std::array<uint8_t, kFixedFrameSize> buildStartFrame(uint8_t msgId) {
  DataBlock block;
  block.bytes = kStartMagic;
  block.dataLength = kFramePayloadSize;
  return buildDataFrame(composeSeq(msgId, 0), block, false, false, 0);
}

// --- Расчёт паритета по окну DATA-блоков ---
ParityPacket computeWindowParity(const std::vector<DataBlock>& blocks,
                                 size_t offset,
                                 uint8_t windowSize) {
  ParityPacket parity;
  parity.block.dataLength = kFramePayloadSize;
  for (uint8_t i = 0; i < windowSize; ++i) {
    const size_t index = offset + i;
    if (index >= blocks.size()) {
      break;
    }
    const DataBlock& block = blocks[index];
    parity.lengthXor ^= block.dataLength;
    for (size_t byte = 0; byte < kFramePayloadSize; ++byte) {
      const uint8_t value = (byte < block.dataLength) ? block.bytes[byte] : 0U;
      parity.block.bytes[byte] ^= value;
    }
  }
  return parity;
}

// --- Формирование транспортного буфера с метаданными количества частей ---
std::vector<uint8_t> buildTransportPayloadWithPartMarkers(const std::vector<uint8_t>& payload,
                                                          bool appendCrc8,
                                                          uint16_t& announcedPartCount) {
  std::vector<uint8_t> transport;
  transport.reserve(payload.size() + kPartCountMetadataBytes);
  transport.resize(kPartCountFieldSize, 0);
  transport.insert(transport.end(), payload.begin(), payload.end());
  transport.resize(transport.size() + kPartCountFieldSize, 0);

  const size_t dataBytesPerBlock = appendCrc8 ? (kFramePayloadSize - 1U) : kFramePayloadSize;
  if (dataBytesPerBlock == 0) {
    announcedPartCount = 0;
    return transport;
  }

  announcedPartCount = static_cast<uint16_t>(
      (transport.size() + dataBytesPerBlock - 1U) / dataBytesPerBlock);

  for (size_t i = 0; i < kPartCountFieldSize; ++i) {
    const uint8_t byteValue = static_cast<uint8_t>((announcedPartCount >> (8U * i)) & 0xFFU);
    transport[i] = byteValue;
    transport[transport.size() - kPartCountFieldSize + i] = byteValue;
  }

  return transport;
}

// --- Разбиение сообщения на DATA-блоки ---
std::vector<DataBlock> splitPayloadIntoBlocks(const std::vector<uint8_t>& payload,
                                              bool appendCrc8) {
  std::vector<DataBlock> blocks;
  const size_t dataBytesPerBlock = appendCrc8 ? (kFramePayloadSize - 1) : kFramePayloadSize;
  size_t offset = 0;
  while (offset < payload.size()) {
    DataBlock block;
    block.bytes.fill(0);
    const size_t chunk = std::min(dataBytesPerBlock, payload.size() - offset);
    if (chunk > 0) {
      std::copy_n(payload.begin() + offset, chunk, block.bytes.begin());
    }
    block.dataLength = static_cast<uint8_t>(chunk);
    if (appendCrc8) {
      const size_t crcIndex = chunk;
      if (crcIndex >= block.bytes.size()) {
        addEvent("Ошибка подготовки DATA-пакета: недостаточно места под CRC-8");
        return {};
      }
      block.bytes[crcIndex] = crc8Dallas(block.bytes.data(), chunk);
    }
    blocks.push_back(block);
    offset += chunk;
  }
  return blocks;
}

// --- CRC-16 CCITT ---
uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t crc) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// --- CRC-8 Dallas/Maxim ---
uint8_t crc8Dallas(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    uint8_t byte = data ? data[i] : 0;
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x80U) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x31U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// --- Пауза между кадрами ---
void waitInterFrameDelay() {
#if defined(ARDUINO)
  delay(computeInterFrameDelayMs());
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(computeInterFrameDelayMs()));
#endif
}

// --- Динамический расчёт паузы между кадрами ---
unsigned long computeInterFrameDelayMs() {
  const float airtime = estimateLoRaAirTimeMs(static_cast<uint8_t>(kFixedFrameSize),
                                             state.rxTiming.preambleSymbols);
  if (!std::isfinite(airtime) || airtime <= 0.0f) {
    return kInterFrameDelayMs;
  }

  const bool highSf = (state.currentSpreadingFactor >= 8);
  const unsigned long minDelay = highSf ? 8UL : 3UL;
  const unsigned long maxDelay = highSf ? 16UL : kInterFrameDelayMs;
  const unsigned long guard = static_cast<unsigned long>(std::ceil(airtime * 0.12f));
  unsigned long delayMs = std::max<unsigned long>(guard, minDelay);
  if (state.rxWindow.lastAckRttMs > 0) {
    delayMs = std::max<unsigned long>(delayMs, state.rxWindow.lastAckRttMs / 12U + 1U);
  }
  return std::clamp<unsigned long>(delayMs, minDelay, maxDelay);
}

// --- Небольшая пауза перед отправкой ACK, чтобы удалённая сторона успела переключиться в RX ---
unsigned long computeAckTurnaroundDelayMs() {
  const float air = estimateLoRaAirTimeMs(static_cast<uint8_t>(kFixedFrameSize),
                                          state.rxTiming.preambleSymbols);
  if (!std::isfinite(air) || air <= 0.0f) {
    return 4UL;
  }
  const unsigned long guard = static_cast<unsigned long>(std::ceil(air * 0.12f));
  const bool highSf = (state.currentSpreadingFactor >= 8);
  const unsigned long minSifs = highSf ? 12UL : 6UL;
  const unsigned long maxSifs = highSf ? 20UL : 12UL;
  return std::clamp<unsigned long>(std::max<unsigned long>(guard, minSifs), minSifs, maxSifs);
}

// --- Оценка времени передачи кадра LoRa ---
float estimateLoRaAirTimeMs(uint8_t payloadSize, uint16_t preambleSymbols) {
  const float bandwidthHz = state.radioBandwidthKhz * 1000.0f;
  if (bandwidthHz <= 0.0f) {
    return static_cast<float>(kInterFrameDelayMs);
  }

  const uint8_t sf = state.currentSpreadingFactor;
  const float symbolDurationMs = static_cast<float>(std::pow(2.0f, sf)) / bandwidthHz * 1000.0f;
  const bool ldro = (symbolDurationMs > 16.0f) || state.rxTiming.ldroForced;
  const uint8_t de = ldro ? 1U : 0U;
  const uint8_t ih = 0U;        // всегда явный заголовок
  const uint8_t crcOn = 1U;     // аппаратный CRC включён
  const uint8_t crDenom = kDefaultCodingRate;
  const uint8_t cr = static_cast<uint8_t>(std::max<int>(static_cast<int>(crDenom) - 4, 1));

  const int32_t numerator = static_cast<int32_t>(8 * payloadSize - 4 * sf + 28 + 16 * crcOn - 20 * ih);
  const int32_t denominator = static_cast<int32_t>(4 * (sf - 2 * de));
  float payloadSymbols = 8.0f;
  if (denominator > 0) {
    const float fraction = std::max<float>(static_cast<float>(numerator), 0.0f) / static_cast<float>(denominator);
    payloadSymbols = 8.0f + std::ceil(fraction) * static_cast<float>(cr + 4U);
  }

  const float preambleMs = (static_cast<float>(preambleSymbols) + 4.25f) * symbolDurationMs;
  const float payloadMs = payloadSymbols * symbolDurationMs;
  return preambleMs + payloadMs;
}

// --- Расчёт тайм-аута ожидания ACK ---
unsigned long computeAckTimeoutMs(uint8_t windowSize) {
  const float ackAirTime = estimateLoRaAirTimeMs(static_cast<uint8_t>(kFixedFrameSize),
                                                 state.rxTiming.preambleSymbols);
  const unsigned long airGuard = static_cast<unsigned long>(std::ceil(ackAirTime)) + 35U;
  const unsigned long windowGuard = static_cast<unsigned long>(windowSize) * (kInterFrameDelayMs);
  const unsigned long adaptive = state.rxWindow.adaptiveAckTimeoutMs;
  return std::max<unsigned long>({140UL, airGuard + windowGuard, adaptive});
}

// --- Обновление статистики ожидания ACK ---
void updateAckTiming(unsigned long measuredRttMs) {
  state.rxWindow.lastAckRttMs = measuredRttMs;
  const float ackAirTime = estimateLoRaAirTimeMs(static_cast<uint8_t>(kFixedFrameSize),
                                                 state.rxTiming.preambleSymbols);
  const unsigned long target = std::max<unsigned long>(
      static_cast<unsigned long>(std::ceil(ackAirTime)) + 30U,
      measuredRttMs + 15U);
  constexpr float kAlpha = 0.3f;
  const float updated = static_cast<float>(state.rxWindow.adaptiveAckTimeoutMs) * (1.0f - kAlpha) +
                        static_cast<float>(target) * kAlpha;
  state.rxWindow.adaptiveAckTimeoutMs = static_cast<unsigned long>(std::clamp(updated, 140.0f, 1800.0f));
}

// --- Реакция на истечение тайм-аута ACK ---
void handleAckTimeoutExpansion() {
  state.rxWindow.adaptiveAckTimeoutMs = std::min<unsigned long>(state.rxWindow.adaptiveAckTimeoutMs + 120U, 2000U);
}

// --- Формирование текстового представления полезной нагрузки ---
String formatTextPayload(const std::vector<uint8_t>& data) {
  String out;
  out.reserve(data.size() + 8);
  for (uint8_t byte : data) {
    if (byte == '\n') {
      out += "\\n";
    } else if (byte == '\r') {
      out += "\\r";
    } else if (byte == '\t') {
      out += "\\t";
    } else if (byte >= 0x20 && byte <= 0x7E) {
      out += static_cast<char>(byte);
    } else {
      char buf[5];
      std::snprintf(buf, sizeof(buf), "\\x%02X", static_cast<unsigned>(byte));
      out += buf;
    }
  }
  return out;
}

// --- Форматирование массива байт для вывода ---
String formatByteArray(const std::vector<uint8_t>& data) {
  String out;
  out.reserve(data.size() * 5);
  out += '[';
  for (size_t i = 0; i < data.size(); ++i) {
    if (i > 0) {
      out += ' ';
    }
    char buf[5];
    std::snprintf(buf, sizeof(buf), "0x%02X", data[i]);
    out += buf;
  }
  out += ']';
  return out;
}

// --- Чтение uint16 в формате little-endian ---
uint16_t readUint16Le(const uint8_t* data) {
  if (!data) {
    return 0;
  }
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1]) << 8;
}

// --- Формирование строки статуса оконных пакетов вида |✅|⛔️| ---
String formatWindowReceptionStatus(uint16_t receivedMask, uint8_t windowSize) {
  if (windowSize == 0) {
    return String('|');
  }

  String status;
  status.reserve(static_cast<size_t>(windowSize) * 5U);
  for (uint8_t bit = 0; bit < windowSize; ++bit) {
    status += '|';
    const bool received = (receivedMask & (1U << bit)) != 0U;
    status += received ? F("✅") : F("⛔️");
  }
  status += '|';
  return status;
}

// --- Логирование принятого сообщения ---
void logReceivedMessage(const std::vector<uint8_t>& payload) {
  addEvent(String("Принято сообщение (") + String(static_cast<unsigned long>(payload.size())) + " байт): " +
           formatByteArray(payload) + " | \"" + formatTextPayload(payload) + "\"", kIncomingColor);
}

// --- Полный сброс состояния приёмника ---
void resetReceiveState() {
  state.rxWindow = {}; // сохраняем тайминги ACK для TX
  // Очистить все параллельные сессии RX
  state.rxSessions.clear();
  state.pendingAck = {};
}

// --- Сторож: принудительный сброс зависшей сборки сообщения ---
void maybeResetStalledReception() {
  if (state.rxSessions.empty()) {
    return;
  }
  const unsigned long now = millis();
  const unsigned long threshold = std::max<unsigned long>(3000UL, state.rxWindow.adaptiveAckTimeoutMs * 3UL + 200UL);
  std::vector<uint8_t> toErase;
  for (const auto& kv : state.rxSessions) {
    const uint8_t msgId = kv.first;
    const PerMessageRx& pm = kv.second;
    // Не сбрасываем сессию, если есть «активность» (дубликаты окон/кадров),
    // даже если нет прогресса — ждём возможный FIN. Берём более свежую из activity/progress.
    const unsigned long last = std::max(pm.msg.lastProgressMs, pm.msg.lastActivityMs);
    if (last != 0 && now - last > threshold) {
      // Перед сбросом попробуем завершить сообщение без FIN, если данные выглядят полными
      finalizeSessionWithoutFin(msgId, state.rxSessions[msgId]);
      toErase.push_back(msgId);
    }
  }
  for (uint8_t id : toErase) {
    state.rxSessions.erase(id);
  }
}

// --- Попытка завершить сборку без FIN при тайм-ауте ---
void finalizeSessionWithoutFin(uint8_t msgId, PerMessageRx& pm) {
  if (!pm.msg.active) {
    return;
  }
  // Нужен хотя бы префикс с количеством частей
  if (pm.msg.buffer.size() < kPartCountFieldSize) {
    addEvent(String("Сброс сборки msgId=") + String(msgId) + ": нет FIN и недостаточно данных для вывода");
    return;
  }
  // Попробуем выделить полезную нагрузку между префиксом и (возможным) суффиксом
  const uint16_t prefixCount = readUint16Le(pm.msg.buffer.data());
  size_t payloadStart = kPartCountFieldSize;
  size_t payloadEnd = pm.msg.buffer.size();
  if (pm.msg.buffer.size() >= kPartCountFieldSize * 2) {
    const uint16_t tail = readUint16Le(pm.msg.buffer.data() + (pm.msg.buffer.size() - kPartCountFieldSize));
    if (tail == prefixCount) {
      payloadEnd -= kPartCountFieldSize; // убираем суффикс
    }
  }
  if (payloadEnd <= payloadStart) {
    addEvent(String("Сброс сборки msgId=") + String(msgId) + ": нет FIN, полезной нагрузки нет");
    return;
  }
  std::vector<uint8_t> payload;
  payload.assign(pm.msg.buffer.begin() + payloadStart, pm.msg.buffer.begin() + payloadEnd);
  addEvent(String("Принято сообщение без FIN (тайм-аут), msgId=") + String(msgId) +
           String(", байт=") + String(static_cast<unsigned long>(payload.size())));
  logReceivedMessage(payload);
}

// --- Обработка принятого кадра ---
void processIncomingFrame(const std::vector<uint8_t>& frame) {
  if (frame.empty()) {
    return;
  }

  addEvent(String("← Кадр: ") + formatByteArray(frame));

  const uint8_t type = frame[0] & kFrameTypeMask;
  switch (type) {
    case kFrameTypeData:
      if (state.triple.enabledRx) {
        const uint16_t seq = (frame.size() >= 3)
                               ? static_cast<uint16_t>(frame[1]) |
                                     (static_cast<uint16_t>(frame[2]) << 8)
                               : 0;
        std::vector<uint8_t> decided;
        if (tripleRxMaybeDecide(seq, frame, decided)) {
          processIncomingDataFrame(decided);
        }
      } else {
        processIncomingDataFrame(frame);
      }
      break;
    case kFrameTypeAck:
      processIncomingAckFrame(frame);
      break;
    case kFrameTypeParity:
      {
        const uint8_t lengthField = (frame.empty())
                                        ? 0
                                        : static_cast<uint8_t>((frame[0] & kDataFlagLengthMask) >>
                                                               kDataFlagLengthShift);
        const bool ackRequest = !frame.empty() && ((frame[0] & kDataFlagAckRequest) != 0);
        const uint16_t seq = (frame.size() >= 3)
                                 ? static_cast<uint16_t>(frame[1]) |
                                       (static_cast<uint16_t>(frame[2]) << 8)
                                 : 0;
        processIncomingParityFrame(seq, lengthField, ackRequest, frame);
      }
      break;
    case kFrameTypeFin:
      processIncomingFinFrame(frame);
      break;
    default:
      addEvent("Неизвестный тип кадра");
      break;
  }
}

// --- Обработка DATA-кадра ---
void processIncomingDataFrame(const std::vector<uint8_t>& frame) {
  if (frame.size() < 3) {
    addEvent("Получен усечённый DATA-кадр");
    return;
  }

  const uint8_t flags = frame[0] & static_cast<uint8_t>(~kFrameTypeMask);
  const uint16_t seq = static_cast<uint16_t>(frame[1]) | (static_cast<uint16_t>(frame[2]) << 8);
  const uint8_t lengthField = static_cast<uint8_t>((flags & kDataFlagLengthMask) >> kDataFlagLengthShift);
  const bool ackRequest = (flags & kDataFlagAckRequest) != 0;
  const bool isParity = (flags & kDataFlagIsParity) != 0;

  // Обнаружение управляющего START-кадра: DATA без флагов, длина = kFramePayloadSize и магическая полезная нагрузка
  if (!isParity && !ackRequest && lengthField == kFramePayloadSize && frame.size() >= 3 + kFramePayloadSize) {
    bool isStart = true;
    for (size_t i = 0; i < kFramePayloadSize; ++i) {
      if (frame[3 + i] != kStartMagic[i]) {
        isStart = false;
        break;
      }
    }
    if (isStart) {
      const uint8_t startMsgId = extractMsgId(seq);
      addEvent(String("Получен START для msgId=") + String(startMsgId) + F(" — очищаем буфер этого сообщения"));
      // Сброс только для данного msgId
      auto it = state.rxSessions.find(startMsgId);
      if (it != state.rxSessions.end()) {
        state.rxSessions.erase(it);
      }
      return;
    }
  }

  const uint8_t msgId = extractMsgId(seq);
  const uint16_t part = extractPart(seq);
  PerMessageRx& pm = state.rxSessions[msgId];
  if (!pm.win.active ||
      part < extractPart(pm.win.baseSeq) ||
      part >= static_cast<uint16_t>(extractPart(pm.win.baseSeq) + kBitmapWidth)) {
    pm.win.active = true;
    const uint16_t basePart = static_cast<uint16_t>(part - (part % kArqWindowSize));
    pm.win.baseSeq = composeSeq(msgId, basePart);
    pm.win.receivedMask = 0;
    pm.win.windowSize = 0;
    pm.win.parityValid = false;
    pm.win.parityBlock = {};
    pm.win.parityLengthXor = 0;
    if (!pm.msg.active) {
      pm.msg.lastActivityMs = millis();
      pm.msg.lastProgressMs = pm.msg.lastActivityMs;
    }
  }

  if (isParity) {
    processIncomingParityFrame(seq, lengthField, ackRequest, frame);
    return;
  }

  const uint8_t bit = static_cast<uint8_t>(extractPart(seq) - extractPart(pm.win.baseSeq));
  if (bit < kBitmapWidth) {
    pm.win.receivedMask |= static_cast<uint16_t>(1U << bit);
    pm.win.windowSize = static_cast<uint8_t>(std::max<uint8_t>(pm.win.windowSize, bit + 1));
  }

  if (!pm.msg.active) {
    pm.msg.active = true;
    pm.msg.nextExpectedSeq = seq;
    pm.msg.buffer.clear();
    pm.msg.pending.clear();
    pm.msg.declaredLength = 0;
    pm.msg.declaredCrc = 0;
    pm.msg.finReceived = false;
    pm.msg.partCountFromPrefix = false;
    pm.msg.partCountFromSuffix = false;
    pm.msg.announcedPartCount = 0;
    pm.msg.lastActivityMs = millis();
    pm.msg.lastProgressMs = pm.msg.lastActivityMs;
  }

  DataBlock block;
  block.bytes.fill(0);
  const size_t payloadAvailable = std::min<size_t>(kFramePayloadSize, frame.size() - 3);
  const uint8_t declaredLength = std::min<uint8_t>(lengthField, static_cast<uint8_t>(kFramePayloadSize));
  if (declaredLength > payloadAvailable) {
    addEvent("DATA-кадр повреждён: объявленная длина превышает полезную нагрузку");
    return;
  }
  if (declaredLength > 0) {
    std::copy_n(frame.begin() + 3, declaredLength, block.bytes.begin());
  }
  block.dataLength = declaredLength;

  if (state.protocol.payloadCrc8) {
    if (block.dataLength == 0) {
      addEvent("DATA-кадр содержит некорректную длину");
      return;
    }
    const uint8_t crcOffset = static_cast<uint8_t>(block.dataLength - 1U);
    const uint8_t expected = block.bytes[crcOffset];
    const uint8_t actualLength = crcOffset;
    const uint8_t computed = crc8Dallas(block.bytes.data(), actualLength);
    if (computed != expected) {
      addEvent("CRC-8 DATA не сошёлся, кадр отброшен");
      return;
    }
    block.bytes[crcOffset] = 0;
    block.dataLength = actualLength;
  }

  pm.msg.pending[seq] = block;
  pm.msg.lastActivityMs = millis();
  flushPendingDataFor(pm);

  if (pm.win.parityValid && attemptParityRecoveryFor(pm)) {
    addEvent("Потерянный DATA-блок восстановлен по паритету");
  }

  if (ackRequest) {
    const uint8_t windowCount = static_cast<uint8_t>(std::max<uint8_t>(pm.win.windowSize, bit + 1));
    prepareAckFor(pm, seq, windowCount, true);
  }
}

// --- Обработка ACK-кадра ---
void processIncomingAckFrame(const std::vector<uint8_t>& frame) {
  if (frame.size() < 6) {
    addEvent("Получен усечённый ACK");
    return;
  }
  AckNotification note;
  note.hasValue = true;
  note.baseSeq = static_cast<uint16_t>(frame[1]) | (static_cast<uint16_t>(frame[2]) << 8);
  note.missingBitmap = static_cast<uint16_t>(frame[3]) | (static_cast<uint16_t>(frame[4]) << 8);
  note.needParity = (frame[0] & kAckFlagNeedParity) != 0;
  note.reportedWindow = frame[5];
  note.ackType = (frame.size() > 6) ? frame[6] : ((note.missingBitmap == 0) ? kAckTypeSuccess : kAckTypeMissing);
  state.pendingAck = note;
  String log = String("Принят ACK: base=") + String(note.baseSeq) +
               ", missing=0x" + String(note.missingBitmap, 16) +
               ", window=" + String(note.reportedWindow) +
               ", тип=" + String(note.ackType) +
               String(", msgId=") + String(extractMsgId(note.baseSeq));

  if (note.ackType == kAckTypeSuccess) {
    log += " (все пакеты получены)";
  } else if (note.ackType == kAckTypeMissing) {
    log += " (есть потери: ";
    bool first = true;
    for (uint8_t bit = 0; bit < kBitmapWidth; ++bit) {
      if ((note.missingBitmap & (1U << bit)) == 0U) {
        continue;
      }
      if (!first) {
        log += ",";
      }
      first = false;
      log += String(static_cast<unsigned long>(note.baseSeq + bit));
    }
    log += ")";
  }

  addEvent(log);
}

// --- Обработка PAR-кадра (пока заглушка) ---
void processIncomingParityFrame(uint16_t seq,
                                uint8_t lengthField,
                                bool ackRequest,
                                const std::vector<uint8_t>& frame) {
  if (frame.size() < 3) {
    addEvent("Получен усечённый PAR-кадр");
    return;
  }
  const uint8_t msgId = extractMsgId(seq);
  PerMessageRx& pm = state.rxSessions[msgId];

  const size_t payloadAvailable = std::min<size_t>(kFramePayloadSize, frame.size() - 3);
  std::copy_n(frame.begin() + 3, payloadAvailable, pm.win.parityBlock.bytes.begin());
  if (payloadAvailable < kFramePayloadSize) {
    std::fill(pm.win.parityBlock.bytes.begin() + payloadAvailable,
              pm.win.parityBlock.bytes.end(),
              0U);
  }
  pm.win.parityBlock.dataLength = kFramePayloadSize;
  pm.win.parityLengthXor = lengthField;
  pm.win.parityValid = true;
  pm.msg.lastActivityMs = millis();

  addEvent(String("Получен паритет по окну base=") + String(pm.win.baseSeq) +
           ", xorLen=" + String(static_cast<unsigned long>(lengthField)));

  if (attemptParityRecoveryFor(pm)) {
    addEvent("Потерянный DATA-блок восстановлен по паритету");
  }

  if (ackRequest) {
    const uint8_t expectedCount = static_cast<uint8_t>(std::max<uint8_t>(
        pm.win.windowSize,
        (extractPart(seq) >= extractPart(pm.win.baseSeq))
            ? static_cast<uint8_t>((extractPart(seq) - extractPart(pm.win.baseSeq)) + 1)
            : pm.win.windowSize));
    prepareAckFor(pm, seq, expectedCount, true);
  }
}

// --- Попытка восстановить единственный потерянный блок окна ---
bool attemptParityRecoveryFor(PerMessageRx& pm) {
  if (!pm.win.parityValid || pm.win.windowSize == 0) {
    return false;
  }

  const uint8_t windowCount = pm.win.windowSize;
  const uint16_t mask = static_cast<uint16_t>((windowCount >= kBitmapWidth)
                                                  ? kBitmapFullMask
                                                  : ((1U << windowCount) - 1U));
  const uint16_t missingMask = static_cast<uint16_t>((~pm.win.receivedMask) & mask);
  if (missingMask == 0 || __builtin_popcount(missingMask) != 1) {
    return false; // либо всё доставлено, либо потерь несколько
  }

  const uint8_t missingBit = static_cast<uint8_t>(__builtin_ctz(missingMask));
  const uint16_t missingSeq = static_cast<uint16_t>(pm.win.baseSeq + missingBit);

  DataBlock recovered = pm.win.parityBlock;
  uint8_t missingLength = pm.win.parityLengthXor;

  for (uint8_t bit = 0; bit < windowCount; ++bit) {
    if (bit == missingBit) {
      continue;
    }
    const uint16_t seq = static_cast<uint16_t>(pm.win.baseSeq + bit);
    auto it = pm.msg.pending.find(seq);
    if (it == pm.msg.pending.end()) {
      return false; // ждём остальные кадры окна
    }
    const DataBlock& known = it->second;
    for (size_t byte = 0; byte < kFramePayloadSize; ++byte) {
      recovered.bytes[byte] ^= known.bytes[byte];
    }
    missingLength ^= known.dataLength;
  }

  recovered.dataLength = std::min<uint8_t>(missingLength, static_cast<uint8_t>(kFramePayloadSize));
  pm.msg.pending[missingSeq] = recovered;
  pm.win.receivedMask |= static_cast<uint16_t>(1U << missingBit);
  pm.win.parityValid = false;

  flushPendingDataFor(pm);
  return true;
}

// --- Обработка FIN-кадра ---
void processIncomingFinFrame(const std::vector<uint8_t>& frame) {
  if (frame.size() < 5) {
    addEvent("Получен усечённый FIN");
    return;
  }
  // Извлекаем идентификатор сообщения, если передан в байте 5
  uint8_t finMsgId = (frame.size() >= 6) ? static_cast<uint8_t>(frame[5] & 0x0F) : 0xFF;
  PerMessageRx* pmPtr = nullptr;
  if (finMsgId != 0xFF) {
    pmPtr = &state.rxSessions[finMsgId];
  } else {
    // Совместимость со старым передатчиком без msgId в FIN:
    // если активна ровно одна сессия — завершаем её; иначе — игнорируем FIN
    if (state.rxSessions.size() == 1) {
      pmPtr = &state.rxSessions.begin()->second;
      addEvent(F("FIN без msgId — завершаем единственную активную сессию"));
    } else {
      addEvent(F("FIN без msgId при нескольких сессиях — проигнорирован"));
      return;
    }
  }
  PerMessageRx& pm = *pmPtr;
  if (!pm.msg.active) {
    pm.msg.active = true;
    pm.msg.nextExpectedSeq = pm.win.baseSeq;
    pm.msg.buffer.clear();
    pm.msg.pending.clear();
    pm.msg.partCountFromPrefix = false;
    pm.msg.partCountFromSuffix = false;
    pm.msg.announcedPartCount = 0;
  }
  pm.msg.lastActivityMs = millis();
  uint16_t length = static_cast<uint16_t>(frame[1]) | (static_cast<uint16_t>(frame[2]) << 8);
  uint16_t crc = static_cast<uint16_t>(frame[3]) | (static_cast<uint16_t>(frame[4]) << 8);
  pm.msg.declaredLength = length;
  pm.msg.declaredCrc = crc;
  pm.msg.finReceived = true;
  flushPendingDataFor(pm);
}

// --- Подготовка и отправка ACK ---
void prepareAckFor(PerMessageRx& pm, uint16_t /*seq*/, uint8_t windowSize, bool forceSend) {
  if (!pm.win.active) {
    return;
  }
  const uint8_t effectiveWindow = std::max<uint8_t>(pm.win.windowSize, windowSize);
  uint16_t mask = static_cast<uint16_t>((effectiveWindow >= kBitmapWidth) ? kBitmapFullMask
                                                                         : ((1U << effectiveWindow) - 1U));
  const uint16_t missing = static_cast<uint16_t>((~pm.win.receivedMask) & mask);
  const bool needParity = (state.protocol.harq && missing != 0);
  if (!forceSend && missing == 0) {
    return;
  }
  const uint16_t base = pm.win.baseSeq;
  const uint8_t mid = extractMsgId(base);
  const String windowStatus = formatWindowReceptionStatus(pm.win.receivedMask, effectiveWindow);
  addEvent(String("[msgId=") + String(mid) + "] Статус пакетов окна base=" + String(base) + ": " + windowStatus);
  // Делаем короткую паузу (SIFS), чтобы передающая сторона успела перейти в RX и принять ACK
#if defined(ARDUINO)
  delay(computeAckTurnaroundDelayMs());
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(computeAckTurnaroundDelayMs()));
#endif
  // Предотвращаем прыжок FHSS на время отправки ACK
  fhssSuspend();
  sendAck(base, missing, needParity, effectiveWindow);
  fhssResume();
  pm.win.baseSeq = static_cast<uint16_t>(base + effectiveWindow);
  pm.win.receivedMask = 0;
  pm.win.windowSize = 0;
  pm.win.parityValid = false;
  pm.win.parityBlock = {};
  pm.win.parityLengthXor = 0;
}

void sendAck(uint16_t baseSeq, uint16_t missingBitmap, bool needParity, uint8_t windowSize) {
  const uint8_t ackType = (missingBitmap == 0) ? kAckTypeSuccess : kAckTypeMissing;
  auto frame = buildAckFrame(baseSeq, missingBitmap, needParity, windowSize, ackType);
  transmitFrame(frame, F("ACK"));
}

// --- Сборка последовательных блоков для конкретного msgId ---
void flushPendingDataFor(PerMessageRx& pm) {
  if (!pm.msg.active) {
    return;
  }

  bool progressed = true;
  while (progressed) {
    progressed = false;
    auto it = pm.msg.pending.find(pm.msg.nextExpectedSeq);
    if (it == pm.msg.pending.end()) {
      break;
    }
    const DataBlock& block = it->second;
    pm.msg.buffer.insert(pm.msg.buffer.end(),
                         block.bytes.begin(),
                         block.bytes.begin() + block.dataLength);
    pm.msg.pending.erase(it);
    pm.msg.nextExpectedSeq = static_cast<uint16_t>(pm.msg.nextExpectedSeq + 1);
    progressed = true;
    pm.msg.lastProgressMs = millis();
  }

  if (!pm.msg.partCountFromPrefix &&
      pm.msg.buffer.size() >= kPartCountFieldSize) {
    // Сохраняем предварительное значение из префикса, но не логируем его,
    // чтобы не вводить в заблуждение, если приём начался не с первой части
    pm.msg.announcedPartCount = readUint16Le(pm.msg.buffer.data());
    pm.msg.partCountFromPrefix = true;
  }

  if (pm.msg.finReceived && pm.msg.pending.empty()) {
    const size_t requiredBytes = static_cast<size_t>(pm.msg.declaredLength) + kPartCountMetadataBytes;
    bool payloadReady = false;
    std::vector<uint8_t> payload;

    if (pm.msg.buffer.size() >= requiredBytes) {
      if (!pm.msg.partCountFromSuffix &&
          pm.msg.buffer.size() >= kPartCountMetadataBytes) {
        const size_t suffixOffset = pm.msg.buffer.size() - kPartCountFieldSize;
        const uint16_t suffixValue = readUint16Le(pm.msg.buffer.data() + suffixOffset);
        pm.msg.partCountFromSuffix = true;
        if (pm.msg.partCountFromPrefix && suffixValue != pm.msg.announcedPartCount) {
          addEvent(String("Предупреждение: последний пакет сообщил ") +
                   String(static_cast<unsigned long>(suffixValue)) +
                   " частей вместо " +
                   String(static_cast<unsigned long>(pm.msg.announcedPartCount)));
        } else if (!pm.msg.partCountFromPrefix) {
          pm.msg.announcedPartCount = suffixValue;
          addEvent(String("Последний пакет сообщил ") +
                   String(static_cast<unsigned long>(suffixValue)) +
                   " частей");
        }
      }

      if (pm.msg.buffer.size() >= kPartCountFieldSize + pm.msg.declaredLength) {
        const auto payloadBegin = pm.msg.buffer.begin() + kPartCountFieldSize;
        const auto payloadEnd = payloadBegin + pm.msg.declaredLength;
        payload.assign(payloadBegin, payloadEnd);
        payloadReady = true;
      } else {
        addEvent("Недостаточно полезных данных после метаданных — сообщение отброшено");
      }
    } else {
      addEvent(String("FIN ожидал ") +
               String(static_cast<unsigned long>(requiredBytes)) +
               " байт с учётом метаданных, получено " +
               String(static_cast<unsigned long>(pm.msg.buffer.size())) +
               " — сообщение отброшено");
    }

    if (payloadReady) {
      const uint16_t crc = crc16Ccitt(payload.data(), payload.size());
      if (crc == pm.msg.declaredCrc) {
        const uint8_t mid = extractMsgId(pm.win.baseSeq);
        addEvent(String("Сообщение собрано (msgId=") + String(mid) + ", байт=" +
                 String(static_cast<unsigned long>(payload.size())) + ")");
        logReceivedMessage(payload);
      } else {
        addEvent("CRC-16 FIN не сошёлся — сообщение отброшено");
      }
    }

    pm.msg.active = false;
    pm.msg.pending.clear();
    pm.msg.buffer.clear();
    pm.msg.finReceived = false;
    pm.msg.partCountFromPrefix = false;
    pm.msg.partCountFromSuffix = false;
    pm.msg.announcedPartCount = 0;
    pm.win.active = false;
    pm.win.receivedMask = 0;
    pm.win.windowSize = 0;
    // Удаляем готовую сессию
    for (auto it = state.rxSessions.begin(); it != state.rxSessions.end(); ++it) {
      if (&(it->second) == &pm) {
        state.rxSessions.erase(it);
        break;
      }
    }
  }
}
