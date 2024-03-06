#pragma once

#define USE_ESP_IDF 1
#define USE_ESP32 1

#define STSREG 0x0000     // STATUS READ REGISTER (POR: 0x0000)
#define GENCREG 0x8000    // GENERAL CONFIGURATION REGISTER (POR: 0x8008)
#define AFCREG 0xC400     // AUTOMATIC FREQUENCY CONTROL CONFIGURATION REGISTER (POR: 0xC4F7)
#define TXCREG 0x9800     // TRANSMIT CONFIGURATION REGISTER (POR: 0x9800)
#define TXBREG 0xB800     // TRANSMIT BYTE REGISTER (POR: 0xB8AA)
#define CFSREG 0xA000     // CENTER FREQUENCY VALUE SET REGISTER (POR: 0xA680)
#define RXCREG 0x9000     // RECEIVE CONTROL REGISTER (POR: 0x9080)
#define BBFCREG 0xC200    // BASEBAND FILTER CONFIGURATION REGISTER (POR: 0xC22C)
#define RXFIFOREG 0xB000  // RECEIVER FIFO READ REGISTER (POR: 0xB000)
#define FIFOSTREG 0xCA00  // FIFO AND RESET MODE CONFIGURATION REGISTER (POR: 0xCA80)
#define SYNBREG 0xCE00    // SYNCHRONOUS BYTE CONFIGURATION REGISTER (POR: 0xCED4)
#define DRSREG 0xC600     // DATA RATE VALUE SET REGISTER (POR: 0xC623)
#define PMCREG 0x8200     // POWER MANAGEMENT CONFIGURATION REGISTER (POR: 0x8208)
#define WTSREG 0xE000     // WAKE-UP TIMER VALUE SET REGISTER (POR: 0xE196)
#define DCSREG 0xC800     // DUTY CYCLE VALUE SET REGISTER (POR: 0xC80E)
#define BCSREG 0xC000     // BATTERY THRESHOLD DETECT AND CLOCK OUTPUT VALUE SET REGISTER (POR: 0xC000)
#define PLLCREG 0xCC00    // PLL CONFIGURATION REGISTER (POR: 0xCC77)

#define STSREG_TXRXFIFO (1U << 15)  // Transmit Register or Receive FIFO bit
#define STSREG_POR (1U << 14)       // Power-on Reset bit
#define STSREG_TXOWRXOF (1U << 13)  // Transmit Overwrite Receive Overflow bit
#define STSREG_WUTINT (1U << 12)    // Wake-up Timer (Interrupt) Overflow bit
#define STSREG_LCEXINT (1U << 11)   // Logic Change on External Interrupt bit
#define STSREG_LBTD (1U << 10)      // Low Battery Threshold Detect bit
#define STSREG_FIFOEM (1U << 9)     // FIFO Empty bit
#define STSREG_ATRSSI (1U << 8)     // Antenna Tuning and Received Signal Strength Indicator bit
#define STSREG_DQDO (1U << 7)       // Data Quality Detect/Indicate Output bit
#define STSREG_CLKRL (1U << 6)      // Clock Recovery Lock bit
#define STSREG_AFCCT (1U << 5)      // Automatic Frequency Control Cycle Toggle bit
#define STSREG_OFFSV (1U << 4)      // Offset Sign Value bit
#define STSREG_OFFSB (7U << 0)      // Offset value to be added to the frequency control parameter (internal PLL)

#define GENCREG_TXDEN (1U << 7)       // TX Data Register Enable bit
#define GENCREG_FIFOEN (1U << 6)      // FIFO Enable bit
#define GENCREG_FBS (3U << 4)         // Frequency Band Select
#define GENCREG_FBS_915MHZ (3U << 4)  // 11 = 915 MHz
#define GENCREG_FBS_868MHZ (2U << 4)  // 10 = 868 MHz
#define GENCREG_FBS_433MHZ (1U << 4)  // 01 = 433 MHz
#define GENCREG_LCS (15U << 0)        // Load Capacitance Select
#define GENCREG_LCS_12 (0b0111 << 0)  // Capacitance 12.0pF

#define AFCCREG_AUTOMS (3U << 6)       // Automatic mode Selection (for AFC)
#define AFCCREG_AUTOMS_RO (0b10 << 6)  // Keeps offset only while receiving (DIO = High)
#define AFCCREG_ARFO (3U << 4)         // Allowable Range for Frequency Offset
#define AFCCREG_MFCS (1U << 3)         // Manual Frequency Control Strobe
#define AFCCREG_HAM (1U << 2)          // High-Accuracy (Fine) mode
#define AFCCREG_FOREN (1U << 1)        // Frequency Offset Register Enable
#define AFCCREG_FOFEN (1U << 0)        //  Frequency Offset Enable

#define TXCREG_MODPLY (1U << 7)        // Modulation Polarity bit (for FSK)
#define TXCREG_MODBW (7U << 4)         // Modulation Bandwidth
#define TXCREG_MODBW_90 (0b0101 << 4)  // Modulation Bandwidth

#define TXCREG_OTXPWR (7U << 0)  // Output Transmit Power Range: 111 = -17.5 dB, 000 = 0 dB

#define RXCREG_FINTDIO (1U << 10)      // Function Interrupt/Data Indicator Output
#define RXCREG_DIORT (3U << 8)         // Data Indicator Output Response Time
#define RXCREG_RXBW (7U << 5)          // Receiver Baseband Bandwidth
#define RXCREG_RXBW_200 (0b100 << 5)   // 200 KHz
#define RXCREG_RXLNA (3U << 3)         // Receiver LNA Gain
#define RXCREG_DRSSIT (7U << 0)        // Digital RSSI Threshold
#define RXCREG_DRSSIT_97 (0b001 << 0)  //-97 dB

#define BBFCREG_ACRLC (1U << 7)  // Automatic Clock Recovery Lock Control
#define BBFCREG_MCRLC (1U << 6)  // Manual Clock Recovery Lock Control
#define BBFCREG_FTYPE (1U << 4)  // Filter Type bit
#define BBFCREG_DQTI (7U << 0)   // Data Quality Threshold Indicator
#define BBFCREG_DQTI_Pos (0U)    // Data Quality Threshold Indicator

#define FIFOSTREG_FFBC (15U << 4)  // FIFO Fill Bit Count
#define FIFOSTREG_FFBC_Pos (4U)
#define FIFOSTREG_FFBC_15 (15U << FIFOSTREG_FFBC_Pos)  // 15 bits before interrupt
#define FIFOSTREG_FFBC_10 (10U << FIFOSTREG_FFBC_Pos)  // 10 bits before interrupt
#define FIFOSTREG_FFBC_8 (8U << FIFOSTREG_FFBC_Pos)    // 8 bits before interrupt
#define FIFOSTREG_FFBC_4 (4U << FIFOSTREG_FFBC_Pos)    // 4 bits before interrupt

#define FIFOSTREG_SYCHLEN (1U << 3)  // Synchronous Character Length
#define FIFOSTREG_FFSC (1U << 2)     // FIFO Fill Start Condition
#define FIFOSTREG_FSCF (1U << 1)     // FIFO Synchronous Character Fill
#define FIFOSTREG_DRSTM \
  (1U << 0)  // Disable (Sensitive) Reset mode. 0 = Enables System Reset for any glitches above 0.2V in the power supply

#define FIFOSTREG_SYCH 0x2DD4
#define FIFORSTREG 0xCA81  // Sync. latch cleared, limit=8bits, disable sensitive reset

#define DRSREG_DRPE (1U << 7)   // Date Rate Prescaler Enable
#define DRSREG_DRPV (63U << 0)  // Data Rate Parameter Value
#define DRSREG_DRPV_Pos (0U)    // Data Rate Parameter Value

#define PMCREG_RXCEN (1U << 7)   // Receiver Chain Enable
#define PMCREG_BBCEN (1U << 6)   // Baseband Circuit Enable
#define PMCREG_TXCEN (1U << 5)   // Transmit Chain Enable
#define PMCREG_SYNEN (1U << 4)   // Synthesizer Enable
#define PMCREG_OSCEN (1U << 3)   // Crystal Oscillator Enable
#define PMCREG_LBDEN (1U << 2)   // Low Battery Detector Enable
#define PMCREG_WUTEN (1U << 1)   // Wake-up Timer Enable
#define PMCREG_CLKOEN (1U << 0)  //  Clock Output Enable. 1 = Disables the clock output, 0 = Enables the clock output
// If the CLKOEN bit is cleared by enabling the clock output, the oscillator continues to run even if the
// OSCEN bit is cleared. The device will not fully enter into the Sleep mode.

#define BCSREG_LBDVB_Pos (0U)  // Low battery detection value

#define PLLCREG_PDDS (1U << 3)      // Phase Detector delay switch
#define PLLCREG_PLLDD (1U << 2)     //  PLL Dithering Disable
#define PLLCREG_PLLBWB (1U << 0)    // Higher PLL Bandwidth
#define PLLCREG_CBTC (3U << 5)      // Clock Buffer Time Control
#define PLLCREG_CBTC_5 (0b11 << 5)  // 5-10Mhz

#define COMMAND_DELAY ets_delay_us(60);  // 50
#define DATA_DELAY ets_delay_us(3600);   // 3500

#define FDATA_READ 0
#define FDATA_WRITE 1

#define DELUMO_ID 0x6520
// #define READ_ENABLED 1
//    #define USE_FSEL_DATA

#include "esphome/core/component.h"
#include "driver/spi_master.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <driver/gpio.h>

namespace esphome {
namespace delumo {

enum SPIMode {
  MODE0 = 0,
  MODE1 = 1,
  MODE2 = 2,
  MODE3 = 3,
};

#ifdef READ_ENABLED
class DelumoOutput : public PollingComponent {
 public:
  DelumoOutput() : PollingComponent(2){};
  void update() override;
#else
class DelumoOutput : public Component {
 public:

#endif
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void turn_on(uint16_t serial);
  void turn_off(uint16_t serial);

  void set_reset_pin(GPIOPin *pin) { reset_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }
  void set_fsel_pin(GPIOPin *pin) { fsel_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }
  void set_mosi_pin(GPIOPin *pin) { mosi_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }
  void set_miso_pin(GPIOPin *pin) { miso_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }
  void set_cs_pin(GPIOPin *pin) { cs_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }
  void set_sclk_pin(GPIOPin *pin) { sclk_pin_ = (gpio_num_t) ((InternalGPIOPin *) pin)->get_pin(); }

  // void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  // void set_fsel_pin(GPIOPin *pin) { fsel_pin_ = pin; }
  // void set_mosi_pin(GPIOPin *pin) { mosi_pin_ = pin; }
  // void set_miso_pin(GPIOPin *pin) { miso_pin_ = pin; }
  // void set_cs_pin(GPIOPin *pin) { cs_pin_ = pin; }
  // void set_sclk_pin(GPIOPin *pin) { sclk_pin_ = pin; }

 protected:
  void send_package_(uint16_t id, uint16_t serial, uint8_t command);
  void send_data_(uint8_t *data, uint8_t length);
  void send_command_(uint16_t command);
  void send_byte_(uint8_t byte);
  void setup_transmit_();
  void setup_receive_();
  void reset_();
  void reset_fifo_();
  void setup_mrf_();

  uint8_t read_message_();
  bool read_incoming_byte_();

  uint8_t byte_number_ = 0;
  uint8_t read_buffer_[6] = {0, 0, 0, 0, 0, 0};

  gpio_num_t reset_pin_;
  gpio_num_t fsel_pin_;
  gpio_num_t miso_pin_;
  gpio_num_t mosi_pin_;
  gpio_num_t cs_pin_;
  gpio_num_t sclk_pin_;

  int data_rate_ = 200000;
  uint8_t mode_ = 0;
  spi_device_handle_t spi_;
};

}  // namespace delumo
}  // namespace esphome
