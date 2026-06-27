#ifndef PCAP01_HPP
#define PCAP01_HPP

#include <cstdint>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

enum class PCAP01_ERROR : int8_t
{
  OK = 0,
  INIT_ERROR = -1,
  COMM_TEST_FAILED = -2,
  FIRMWARE_LOAD_FAILED = -3,
  SPI_BUS_ERROR = -4
};

class PCAP01
{
public:
  PCAP01();
  ~PCAP01() = default;

  PCAP01_ERROR begin(const struct spi_dt_spec *spi_spec,
                     const struct gpio_dt_spec *intn_gpio_spec = nullptr);

  bool dataReady() const;
  void clearDataReady();
  bool waitDataReady(k_timeout_t timeout);
  void sendOpcode(uint8_t opcode);

  void writeRegister(uint8_t reg_addr, uint32_t data);
  float readRegister(uint8_t reg_addr, uint8_t fractional_bits = KFractionalBits);
  uint32_t readRawRegister(uint8_t reg_addr);

  static constexpr uint8_t CHANNEL_COUNT = 2;
  bool measureCapacitancesPF(k_timeout_t timeout,
                             float capacitance_pf[CHANNEL_COUNT],
                             uint32_t raw_count[CHANNEL_COUNT] = nullptr,
                             int64_t *wait_ms = nullptr,
                             uint32_t *status = nullptr);

  void initLowPassFilter(float cutoff_freq, float sampling_freq);
  float applyLowPassFilter(uint8_t node_index, float raw_capacitance);

  static constexpr uint8_t KOpcodeReset = 0x88;
  static constexpr uint8_t KOpcodePartialReset = 0x8A;
  static constexpr uint8_t KOpcodeStartCDCMeas = 0x8C;
  static constexpr uint8_t KOpcodeStartRDCMeas = 0x8E;

  static constexpr uint8_t KCmdWriteReg = 0xC0;
  static constexpr uint8_t KRunbitRegAddr = 0x14;
  static constexpr uint32_t KRunbitRegData = 0x000001;

  static constexpr uint8_t KCmdReadReg = 0x40;
  static constexpr uint8_t KC1DivC0RegAddr = 0x01;
  static constexpr uint8_t KC2DivC0RegAddr = 0x02;
  static constexpr uint8_t KC3DivC0RegAddr = 0x03;
  static constexpr uint8_t KStatusRegAddr = 0x08;
  static constexpr uint8_t KFractionalBits = 21;

  static constexpr float KRefCapPF = 100.0f;

private:
  const struct spi_dt_spec *spi_spec_ = nullptr;
  const struct gpio_dt_spec *intn_gpio_spec_ = nullptr;
  int last_spi_error_ = 0;
  struct k_sem data_ready_sem_;

  struct intn_context
  {
    struct gpio_callback cb;
    PCAP01 *instance;
  };
  struct intn_context intn_ctx_ = {};
  bool enable_filter_ = false;
  float filter_alpha_ = 1.0f;
  float filtered_data_[CHANNEL_COUNT] = {0.0f};
  bool is_filter_first_run_[CHANNEL_COUNT] = {true, true};

  int spiReadWrite(const uint8_t *tx, uint8_t *rx, size_t len);
  int spiWrite(const uint8_t *tx, size_t len);
  uint8_t spiReadWriteByte(uint8_t txdata);

  static void intnIsrHandler(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins);
  static uint8_t capacitanceRegister(uint8_t channel);

  uint8_t sramComm(uint8_t comm_type, uint16_t addr, uint8_t data);
  PCAP01_ERROR commTest();
  PCAP01_ERROR writeStdFirmware();
  void writeStdConfig();
};

#endif
