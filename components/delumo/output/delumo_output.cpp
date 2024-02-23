#include "delumo_output.h"
#include "esphome/core/log.h"
#include <driver/gpio.h>
#include <string.h>

namespace esphome {
namespace delumo {

// uint8_t delumo_command = 0;
// uint8_t delumo_buffer[6];

volatile bool receive_enabled = true;

static const char *const TAG = "spi.delumo";

spi_device_handle_t spi;

#ifdef READ_ENABLED
void DelumoOutput::update() {
  if (this->read_message_() == 1) {
    ESP_LOGI(TAG, "Received %02x %02x %02x %02x %02x %02x", this->read_buffer_[0], this->read_buffer_[1],
             this->read_buffer_[2], this->read_buffer_[3], this->read_buffer_[4], this->read_buffer_[5]);
  }
}
#endif

void DelumoOutput::turn_on(uint16_t serial) {
  send_package_(DELUMO_ID, serial, 0x1);
  ESP_LOGI(TAG, "TURN_ON at output: %#04x", serial);
}

void DelumoOutput::turn_off(uint16_t serial) {
  send_package_(DELUMO_ID, serial, 0x4);
  ESP_LOGI(TAG, "TURN_OFF at output: %#04x", serial);
}

void DelumoOutput::dump_config() {
  // ESP_LOGCONFIG(TAG, "SPIDevice");
  // LOG_PIN("  CS pin: ", this->cs_pin_);
  // ESP_LOGCONFIG(TAG, "  Mode: %d", this->mode_);
  // if (this->data_rate_ < 1000000) {
  //   ESP_LOGCONFIG(TAG, "  Data rate: %dkHz", this->data_rate_ / 1000);
  // } else {
  //   ESP_LOGCONFIG(TAG, "  Data rate: %dMHz", this->data_rate_ / 1000000);
  // }
  // ESP_LOGCONFIG(TAG, "Delumo GPIO:");
  // LOG_PIN("  FSEL Pin: ", this->fsel_pin_);
  // LOG_PIN("  RESET Pin: ", this->reset_pin_);
}

void DelumoOutput::reset_() {
  this->reset_pin_->digital_write(false);
  ets_delay_us(10);
  this->reset_pin_->digital_write(true);
  ets_delay_us(300);
}

void spi_pre_transfer_callback(spi_transaction_t *t) {
  int fdata = (int) t->user;
  // gpio_set_level(MRF_FDATA_Pin, fdata);
}

void spi_post_transfer_callback(spi_transaction_t *t) {
  // gpio_set_level(MRF_FDATA_Pin, FDATA_WRITE);
}

void DelumoOutput::setup_mrf_() {
  // CHIP RESET
  // Power management - turn ON oscillator, low battery detector and DISABLE clock output (1)
  send_command_(PMCREG | PMCREG_OSCEN | PMCREG_LBDEN | PMCREG_CLKOEN);
  // TRANSMIT CONFIG: modulation bandwidth 90KHz
  send_command_(TXCREG | TXCREG_MODBW_90);
  // RECEIVE CONFIG: DIO output (00 - Fast), Rec BW 200kHz, Gain 0dB, RSSI thr -97dB
  send_command_(RXCREG | RXCREG_FINTDIO | RXCREG_RXBW_200 | RXCREG_DRSSIT_97);
  // GENERAL CONFIG: TX enable, FIFO enable, 868MHz, capacitance 12pF
  send_command_(GENCREG | GENCREG_TXDEN | GENCREG_FIFOEN | GENCREG_FBS_868MHZ | GENCREG_LCS_12);
  // Center Frequency
  send_command_(CFSREG | 0x620);  // Fval = 1658, F0 = 867.84Mhz
  // Automatic Freq: Receive only, High accuracy, Offset reg enable, Offset enable
  send_command_(AFCREG | AFCCREG_AUTOMS_RO | AFCCREG_HAM | AFCCREG_FOREN | AFCCREG_FOFEN);
  // FIFO bytes count = 8; Fill start condition, Sync char fill, disable reset
  send_command_(FIFOSTREG | FIFOSTREG_FFBC_8 | FIFOSTREG_FFSC | FIFOSTREG_FSCF | FIFOSTREG_DRSTM);
  // DATA RATE: Prescaler enable, data rate 2.26 Kbps(?)
  send_command_(DRSREG | DRSREG_DRPE | (0b0010010 << DRSREG_DRPV_Pos));
  // WAKE UP timer disable
  send_command_(WTSREG);
  // DUTY CYCLE: disable
  send_command_(DCSREG);
  // BASEBAND FILTER: ACR lock control,
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  send_command_(BBFCREG | BBFCREG_ACRLC | (0b100 << BBFCREG_DQTI_Pos) | 0b00101000);  // ADDED RESERVED BITS FROM SWITCH
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Synchronous byte: D4
  send_command_(SYNBREG | 0xD4);
  // BATTERY THRESHOLD AND CLOCK; LOW battery thr = 2.25V+ (0.1*LBDVB Value);
  send_command_(BCSREG | (0x01 << BCSREG_LBDVB_Pos));
  // PLLCREG
  send_command_(PLLCREG | PLLCREG_CBTC_5 | PLLCREG_PLLDD | PLLCREG_PLLBWB |
                0b00000010);  // ADDED RESERVED BITS FROM SWITCH
}

void DelumoOutput::setup() {
  ESP_LOGD(TAG, "Setting up SPIDevice...");
  // this->spi_setup();
  ESP_LOGCONFIG(TAG, "SPIDevice started!");

  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
  // esp_log_level_set("*", ESP_LOG_INFO);
  // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
  // esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
  // esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
  // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  // esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
  // esp_log_level_set("outbox", ESP_LOG_VERBOSE);

  // Initialize GPIO
  this->fsel_pin_->setup();
  this->reset_pin_->setup();
  this->cs_pin_->setup();
  this->mosi_pin_->setup();
  this->miso_pin_->setup();
  this->sclk_pin_->setup();

  this->fsel_pin_->digital_write(true);
  this->reset_pin_->digital_write(false);
  this->cs_pin_->digital_write(true);
  this->mosi_pin_->digital_write(true);
  this->miso_pin_->digital_write(true);
  this->sclk_pin_->digital_write(false);

  this->reset_pin_->digital_write(true);

  // Initialize the SPI bus
  esp_err_t ret;
  spi_bus_config_t buscfg = {.mosi_io_num = ((InternalGPIOPin *) mosi_pin_)->get_pin(),
                             .miso_io_num = ((InternalGPIOPin *) miso_pin_)->get_pin(),
                             .sclk_io_num = ((InternalGPIOPin *) sclk_pin_)->get_pin(),
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 0};

  spi_device_interface_config_t devcfg = {
      .mode = mode_,                        // SPI mode 0
      .clock_speed_hz = data_rate_,         // Clock in KHz
      .spics_io_num = -1,                   // PIN_NUM_CS,               //CS pin
      .queue_size = 1,                      // We want to be able to queue 7 transactions at a time
      .pre_cb = spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line
      .post_cb = spi_post_transfer_callback,
      //.input_delay_ns = 30,
  };
  ESP_LOGI(TAG, "SPI bus initialize");
  ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "SPI bus add device");
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  reset_();
  ESP_LOGI(TAG, "Setup MRF");
  setup_mrf_();
  MRF_DELAY;
  ESP_LOGI(TAG, "Setup receive");
  setup_receive_();
}

void DelumoOutput::setup_transmit_() {
  receive_enabled = false;

  // SET TRANSMITTER MODE
  send_command_(PMCREG | PMCREG_TXCEN | PMCREG_SYNEN | PMCREG_OSCEN | PMCREG_LBDEN | PMCREG_CLKOEN);
  MRF_DELAY;
  reset_fifo_();
}

void DelumoOutput::setup_receive_() {
  //  SET RECEIVER MODE
  send_command_(PMCREG | PMCREG_RXCEN | PMCREG_BBCEN | PMCREG_SYNEN | PMCREG_OSCEN | PMCREG_LBDEN | PMCREG_CLKOEN);
  ets_delay_us(100);

  reset_fifo_();

  receive_enabled = true;
}

void DelumoOutput::reset_fifo_() {
  // RESET FIFO BUFFER
  send_command_(FIFOSTREG | FIFOSTREG_FFBC_8 | FIFOSTREG_DRSTM);
  send_command_(FIFOSTREG | FIFOSTREG_FFBC_8 | FIFOSTREG_FSCF | FIFOSTREG_DRSTM);
  ets_delay_us(10);
}

void DelumoOutput::send_package_(uint16_t id, uint16_t serial, uint8_t command) {
  uint8_t buffer[16] = {0};
  buffer[0] = (uint8_t) ((id & 0xFF00) >> 8);
  buffer[1] = (uint8_t) (id & 0x00FF);
  buffer[2] = (uint8_t) ((serial & 0xFF00) >> 8);
  buffer[3] = (uint8_t) (serial & 0x00FF);
  buffer[4] = (command);
  buffer[5] = (uint8_t) (buffer[1] + buffer[2] + buffer[3] + buffer[4]) % 256;  // checksum
  send_data_(buffer, 6);
  MRF_DELAY;
}

void DelumoOutput::send_data_(uint8_t *data, uint8_t length) {
  setup_transmit_();
  ESP_LOGD(TAG, "Starting transmit");
  this->cs_pin_->digital_write(false);

  // INIT
  this->send_byte_(0xb8);
  this->send_byte_(0xaa);

  // TX PRE
  this->send_byte_(0xaa);
  this->send_byte_(0xaa);

  this->send_byte_(0x2d);
  this->send_byte_(0xd4);

  // SWITCH ID
  for (uint8_t i = 0; i < length; ++i) {
    this->send_byte_(data[i]);
  }

  // FINISH
  this->send_byte_(0x00);
  this->send_byte_(0x00);

  this->cs_pin_->digital_write(true);

  ESP_LOGD(TAG, "End of thansmit");
  MRF_DELAY;
  setup_receive_();
}

void DelumoOutput::send_byte_(uint8_t byte) {
  // esp_err_t ret;
  // spi_transaction_t t;

  // memset(&t, 0, sizeof(t));       // Zero out the transaction
  // t.length = 8;                   // Command is 8 bits
  // t.tx_buffer = &byte;            // The data is the cmd itself
  // t.user = (void *) FDATA_WRITE;  // FDATA kept high
  //                                 // if (keep_cs) t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

  // // while (miso_pin_->digital_read() == 0)
  // ;
  // // MRF reports READY by high MISO

  // ret = spi_device_polling_transmit(spi_, &t);  // Transmit!
  // assert(ret == ESP_OK);                        // Should have had no issues
}

void DelumoOutput::send_command_(uint16_t command) {
  // esp_err_t ret;
  // spi_transaction_t t;

  // // swapping lower and higher bytes to transfer at once reg bits for MRF49XA
  // uint16_t a = ((command & 0xFF00) >> 8);
  // uint16_t b = ((command & 0x00FF) << 8);
  // command = a | b;
  // memset(&t, 0, sizeof(t));  // Zero out the transaction
  // t.length = 16;             // Command is 8 bits
  // t.tx_buffer = &command;    // The data is the cmd itself

  // // while (miso_pin_->digital_read() == 0)
  // ;
  // // MRF reports READY by high MISO

  // this->cs_pin_->digital_write(false);
  // ret = spi_device_polling_transmit(spi_, &t);  // Transmit!
  // assert(ret == ESP_OK);                        // Should have had no issues.
  // this->cs_pin_->digital_write(true);
}

// returns 1 if 6 bytes are received
uint8_t DelumoOutput::read_message_() {
  bool byte_received = this->read_incoming_byte_();
  if (this->byte_number_ == 6) {
    this->byte_number_ = 0;
    reset_fifo_();
    return 1;
  } else {
    return 0;
  }
}

bool DelumoOutput::read_incoming_byte_()  // mrf49xa_PollFIFO()
{
  bool fifo_empty = true;

  uint8_t tx_data[2] = {0x00, 0x00};
  uint8_t rx_data[2] = {0, 0};
  uint8_t byte = 0;

  // this->cs_pin_->digital_write(false);

  // byte = this->delegate_->transfer((uint8_t) 0x00);  // zero is STATUS REGISTER
  byte = (gpio_get_level(GPIO_NUM_19) << 7);

  // this->cs_pin_->digital_write(true);

  if ((byte & 0b10000000) == 0) {  // FIFO empty
    return false;
  }

#ifdef USE_FSEL_DATA
  tx_data[0] = 0xff;
  tx_data[1] = 0xff;

  this->fsel_pin_->digital_write(false);
  ets_delay_us(1);
  this->delegate_->transfer(tx_data, rx_data, 2);
  ets_delay_us(1);
  this->fsel_pin_->digital_write(true);
  reset_fifo_();
  if (rx_data[1] == 0x65) {
    // DELUMO HEADER of 6518, 6519 or 6520
    byte_number_ = 0;
  }
  read_buffer_[byte_number_++] = rx_data[1];
#else
  tx_data[0] = 0xb0;
  // this->cs_pin_->digital_write(false);
  //  this->write_byte(0xb0);          // FIFOREG ADDRESS
  //  rx_data[1] = this->read_byte();  // INCOMING DATA
  // this->cs_pin_->digital_write(true);
  if (rx_data[1] == 0x65) {
    // DELUMO HEADER of 6518, 6519 or 6520
    this->byte_number_ = 0;
  }
  this->read_buffer_[this->byte_number_++] = rx_data[1];
#endif

  // ESP_LOGI(TAG, "Getting byte %02x %02x", rx_data[0], rx_data[1]);
  //  reset_fifo_();
  return true;
}

}  // namespace delumo
}  // namespace esphome
