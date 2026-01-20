#include "cse7761.h"

#include "esphome/core/log.h"

namespace esphome {
namespace cse7761 {

static const char *const TAG = "cse7761-mremy";

/*********************************************************************************************\
 * CSE7761 - Energy  (Sonoff Dual R3 Pow v1.x)
 *
 * Based on Tasmota source code
 * See https://github.com/arendst/Tasmota/discussions/10793
 * https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xnrg_energy/xnrg_19_cse7761.ino
\*********************************************************************************************/
bool DefaultCalibration = false;
static const int CSE7761_IAC_REF = 52107;    // Current RMS reference (RmsIAC)
static const int CSE7761_IBC_REF = 52068;    // Current RMS reference (RmsIBC)
static const int CSE7761_UC_REF  = 42416;    // Voltage RMS reference (RmsUc)
static const int CSE7761_PAC_REF = 44246;    // Active power reference (PowerPAC)
static const int CSE7761_PBC_REF = 44216;    // Active power reference (PowerPBC)
static const int CSE7761_SC_REF  = 44246;    // Aparent power reference (PowerSC)
static const int CSE7761_EAC_REF = 59451;    // Energy reference (ENERGY_AC)
static const int CSE7761_EBC_REF = 59410;    // Energy reference (ENERGY_BC)
static const int CSE7761_FREF = 3579545;     // System clock (3.579545MHz) as used in frequency calculation

static const uint8_t CSE7761_REG_SYSCON = 0x00;     // (2) System Control Register (0x0A04)
static const uint8_t CSE7761_REG_EMUCON = 0x01;     // (2) Metering control register (0x0000)
static const uint8_t CSE7761_REG_EMUCON2 = 0x13;    // (2) Metering control register 2 (0x0001)
static const uint8_t CSE7761_REG_PULSE1SEL = 0x1D;  // (2) Pin function output select register (0x3210)

static const uint8_t CSE7761_REG_UFREQ = 0x23;      // (2) Voltage Frequency (0x0000)
static const uint8_t CSE7761_REG_RMSIA = 0x24;      // (3) The effective value of channel A current (0x000000)
static const uint8_t CSE7761_REG_RMSIB = 0x25;      // (3) The effective value of channel B current (0x000000)
static const uint8_t CSE7761_REG_RMSU = 0x26;       // (3) Voltage RMS (0x000000)
static const uint8_t CSE7761_REG_POWERPA = 0x2C;    // (4) Channel A active power, update rate 27.2Hz (0x00000000)
static const uint8_t CSE7761_REG_POWERPB = 0x2D;    // (4) Channel B active power, update rate 27.2Hz (0x00000000)
static const uint8_t CSE7761_REG_SYSSTATUS = 0x43;  // (1) System status register

// static const uint8_t CSE7761_REG_COEFFOFFSET = 0x6E;     // (2) Coefficient checksum offset (0xFFFF)
static const uint8_t CSE7761_REG_COEFFCHKSUM = 0x6F;  // (2) Coefficient checksum
static const uint8_t CSE7761_REG_RMSIAC = 0x70;       // (2) Channel A effective current conversion coefficient
// static const uint8_t CSE7761_REG_RMSIBC         = 0x71;     // (2) Channel B effective current conversion coefficient
// static const uint8_t CSE7761_REG_RMSUC          = 0x72;     // (2) Effective voltage conversion coefficient
// static const uint8_t CSE7761_REG_POWERPAC       = 0x73;     // (2) Channel A active power conversion coefficient
// static const uint8_t CSE7761_REG_POWERPBC       = 0x74;     // (2) Channel B active power conversion coefficient
// static const uint8_t CSE7761_REG_POWERSC        = 0x75;     // (2) Apparent power conversion coefficient
// static const uint8_t CSE7761_REG_ENERGYAC       = 0x76;     // (2) Channel A energy conversion coefficient
// static const uint8_t CSE7761_REG_ENERGYBC       = 0x77;     // (2) Channel B energy conversion coefficient

// ==============================
// CSE7761 Special Commands
// ==============================

static const uint8_t CSE7761_SPECIAL_COMMAND  = 0xEA; // Start special command
static const uint8_t CSE7761_CMD_RESET        = 0x96; // Reset command; chip resets after receiving this

// Channel selection commands (currently unused)
// static const uint8_t CSE7761_CMD_CHAN_A_SELECT = 0x5A; // Ch A selection for apparent power, PF, phase, instantaneous P & overload signal
// static const uint8_t CSE7761_CMD_CHAN_B_SELECT = 0xA5; // Ch B selection for apparent power, PF, phase, instantaneous P & overload signal

static const uint8_t CSE7761_CMD_CLOSE_WRITE  = 0xDC; // Close write operation
static const uint8_t CSE7761_CMD_ENABLE_WRITE = 0xE5; // Enable write operation

// ==============================
// CSE7761 Register / Measurement Enum
// ==============================

enum CSE7761 {
    RMS_IAC,    // RMS current of channel A (phase A current measurement)
    RMS_IBC,    // RMS current of channel B (phase B current measurement)
    RMS_UC,     // RMS voltage (line voltage measurement)
    POWER_PAC,  // Instantaneous active power on channel A
    POWER_PBC,  // Instantaneous active power on channel B
    POWER_SC,   // Instantaneous apparent power (combined)
    ENERGY_AC,  // Accumulated energy for channel A (active energy)
    ENERGY_BC   // Accumulated energy for channel B (active energy)
};

// CSE7761 Component Setup
void CSE7761Component::setup() {
    this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_RESET); // Send reset command
    uint16_t syscon = this->read_(0x00, 2); // Read SYSCON register (default 0x0A04)
    if ((0x0A04 == syscon) && this->chip_init_()) { // Check if chip responds and initialize
        this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_CLOSE_WRITE); // Close write mode
        ESP_LOGD(TAG, "CSE7761 found");
        this->data_.ready = true;
    } else {
        this->mark_failed(); // Mark as failed if init fails
    }
}

void CSE7761Component::dump_config() {
    ESP_LOGCONFIG(TAG, "CSE7761: (Default Calibration: %s)", DefaultCalibration ? "TRUE" : "FALSE");
    uint32_t regVal = 0;
    for (uint8_t i = 0; i < 8; i++) {
        regVal = this->read_(CSE7761_REG_RMSIAC + i, 2);
        ESP_LOGCONFIG(TAG, "Reg%02hhX: %d - actual:%d", CSE7761_REG_RMSIAC + i, regVal, this->data_.coefficient[i]);
    }
    if (this->is_failed()) ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL); // Communication failure
    LOG_UPDATE_INTERVAL(this); // Log sensor update interval
    this->check_uart_settings(38400, 1, uart::UART_CONFIG_PARITY_EVEN, 8); // Verify UART: 38400 8E1
}

float CSE7761Component::get_setup_priority() const {
    return setup_priority::DATA; // Set setup priority to DATA
}

void CSE7761Component::update() {
    if (this->data_.ready) this->get_data_(); // Only read data if chip is ready
}

void CSE7761Component::write_(uint8_t reg, uint16_t data) {
  uint8_t buffer[5];

  buffer[0] = 0xA5;
  buffer[1] = reg;
  uint32_t len = 2;
  if (data) {
    if (data < 0xFF) {
      buffer[2] = data & 0xFF;
      len = 3;
    } else {
      buffer[2] = (data >> 8) & 0xFF;
      buffer[3] = data & 0xFF;
      len = 4;
    }
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
      crc += buffer[i];
    }
    buffer[len] = ~crc;
    len++;
  }

  this->write_array(buffer, len);
}

bool CSE7761Component::read_once_(uint8_t reg, uint8_t size, uint32_t *value) {
  while (this->available()) {
    this->read();
  }

  this->write_(reg, 0);

  uint8_t buffer[8] = {0};
  uint32_t rcvd = 0;

  for (uint32_t i = 0; i <= size; i++) {
    int value = this->read();
    if (value > -1 && rcvd < sizeof(buffer) - 1) {
      buffer[rcvd++] = value;
    }
  }

  if (!rcvd) {
    ESP_LOGD(TAG, "Received 0 bytes for register %hhu", reg);
    return false;
  }

  rcvd--;
  uint32_t result = 0;
  // CRC check
  uint8_t crc = 0xA5 + reg;
  for (uint32_t i = 0; i < rcvd; i++) {
    result = (result << 8) | buffer[i];
    crc += buffer[i];
  }
  crc = ~crc;
  if (crc != buffer[rcvd]) {
    return false;
  }

  *value = result;
  return true;
}

uint32_t CSE7761Component::read_(uint8_t reg, uint8_t size) {
  bool result = false;  // Start loop
  uint8_t retry = 3;    // Retry up to three times
  uint32_t value = 0;   // Default no value
  while (!result && retry > 0) {
    retry--;
    if (this->read_once_(reg, size, &value))
      return value;
  }
  ESP_LOGE(TAG, "Reading register %hhu failed!", reg);
  return value;
}

// Calculate coefficient for a given unit
uint32_t CSE7761Component::coefficient_by_unit_(uint32_t unit) {
    uint32_t coeff = 1;
    if (this->data_.model == CSE7761_MODEL_SONOFF_POWCT) coeff = 5; // Special multiplier for POW CT
    switch (unit) {
        case RMS_IAC:  return (0x800000 * 100 / (this->data_.coefficient[RMS_IAC] * coeff)) * 10; // RMS current A, scaled to 32-bit
        case RMS_IBC:  return (0x800000 * 100 / (this->data_.coefficient[RMS_IBC] * coeff)) * 10; // RMS current B, scaled to 32-bit
        case RMS_UC:   return 0x400000 * 100 / this->data_.coefficient[RMS_UC];            // RMS voltage
        case POWER_PAC:return 0x80000000 / (this->data_.coefficient[POWER_PAC] * coeff);  // Instantaneous active power A
        case POWER_PBC:return 0x80000000 / (this->data_.coefficient[POWER_PBC] * coeff);  // Instantaneous active power B
    }
    return 0; // Unsupported unit
}

bool CSE7761Component::chip_init_() {
    uint16_t calc_chksum = 0xFFFF; // Initialize checksum
    ESP_LOGD(TAG, "Chip Init");
    // Read 8 coefficient registers and accumulate checksum
    for (uint32_t i = 0; i < 8; i++) {
        this->data_.coefficient[i] = this->read_(CSE7761_REG_RMSIAC + i, 2);
        calc_chksum += this->data_.coefficient[i];
    }
    calc_chksum = ~calc_chksum; // Invert checksum
    uint16_t coeff_chksum = this->read_(CSE7761_REG_COEFFCHKSUM, 2); // Read stored checksum 
    // If checksum mismatch or zero, use default calibration
    if ((calc_chksum != coeff_chksum) || (!calc_chksum)) {
        DefaultCalibration = true;
        this->data_.coefficient[RMS_IAC] = CSE7761_IAC_REF;
        this->data_.coefficient[RMS_IBC] = CSE7761_IBC_REF;
        this->data_.coefficient[RMS_UC] = CSE7761_UC_REF;
        this->data_.coefficient[POWER_PAC] = CSE7761_PAC_REF;
        this->data_.coefficient[POWER_PBC] = CSE7761_PBC_REF;
        this->data_.coefficient[POWER_SC] = CSE7761_SC_REF;
        this->data_.coefficient[ENERGY_AC] = CSE7761_EAC_REF;
        this->data_.coefficient[ENERGY_BC] = CSE7761_EBC_REF;
    }
    this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_ENABLE_WRITE); // Enable write operations
    uint8_t sys_status = this->read_(CSE7761_REG_SYSSTATUS, 1);
    if (sys_status & 0x10) {  // Write enable to protected registers (WREN)
        
// SYSCON (0x00, default 0x0A04)
// [15:11] NC        : Reserved, read as 1
// [10]    ADC2ON    : 0 = Current Channel B disabled, 1 = enabled
// [9]     NC        : Reserved, read as 1
// [8:6]   PGAIB[2:0]: 000=1×, 001=2×, 010=4×, 011=8×, 1xx=16× (Current Channel B gain) - 1x for PowCt
// [5:3]   PGAU[2:0] : 000=1×, 001=2×, 010=4×, 011=8×, 1xx=16× (Voltage Channel gain)
// [2:0]   PGAIA[2:0]: 000=1×, 001=2×, 010=4×, 011=8×, 1xx=16× (Current Channel A gain) - 1x for PowCt
        
        if (this->data_.model == CSE7761_MODEL_SONOFF_POWCT) {
            this->write_(CSE7761_REG_SYSCON | 0x80, 0xFE00); // POW CT: enable Ch B
        }   
        else {
            this->write_(CSE7761_REG_SYSCON | 0x80, 0xFF04); // Dual R3 default
        }
            

// EMUCON (0x01, default 0x0000)
// [15:14] Tsensor_Step[1:0]: 00=Temp sensor step1 (OP1+/OP2+), 01=step2 (OP1+/OP2-), 10=step3 (OP1-/OP2+), 11=step4 (OP1-/OP2-)
// [13]    tensor_en        : 0=Temperature measurement disabled, 1=enabled
// [12]    comp_off         : 0=Comparator enabled, 1=Comparator disabled
// [11:10] Pmode[1:0]       : 00=Alg. sum (+/-) with REVQ, 01=Positive only, 10=Abs sum (+/-), 11=Reserved (same as 00)
// [9]     NC               : Reserved
// [8]     ZXD1             : 0=ZX toggles only at selected zero-crossing, 1=ZX toggles at both + and − zero-crossings
// [7]     ZXD0             : 0=Positive zero-crossing selected, 1=Negative zero-crossing selected
// [6]     HPFIBOFF         : 0=HPF of Current Channel B enabled, 1=disabled
// [5]     HPFIAOFF         : 0=HPF of Current Channel A enabled, 1=disabled
// [4]     HPFUOFF          : 0=HPF of Voltage (U) Channel enabled, 1=disabled
// [3]     NC               : Reserved
// [2]     PBRUN            : 0=PFB pulse & active energy accumulation disabled, 1=enabled
// [1]     PARUN            : 0=PFA pulse & active energy accumulation disabled, 1=enabled
// [0]     NC               : Reserved

        this->write_(CSE7761_REG_EMUCON | 0x80, 0x1183); // Dual R3: Ch B + ZX both pos & neg
    
// EMUCON2 (0x13, default 0x0001)
// [15:13] NC            : Reserved
// [12]    SDOCmos       : 0=SDO pin CMOS push-pull output, 1=SDO pin CMOS open-drain output
// [11]    EPB_CB        : 0=Energy_PB cleared after reading, 1=Energy_PB not cleared after reading (UART mode requires 1)
// [10]    EPA_CB        : 0=Energy_PA cleared after reading, 1=Energy_PA not cleared after reading (UART mode requires 1)
// [9:8]   DUPSEL[1:0]   : 00=Mean reg update 3.4Hz, 01=6.8Hz, 10=13.65Hz, 11=27.3Hz
// [7]     CHS_IB        : 0=Measure internal temperature, 1=Measure Current Channel B
// [6]     PfactorEN    : 0=Power factor output disabled, 1=enabled
// [5]     WaveEN       : 0=Waveform & instantaneous data output disabled, 1=enabled
// [4]     SAGEN        : 0=Voltage sag detection disabled, 1=enabled (WaveEN must be 1)
// [3]     OverEN       : 0=OV/OC/OL detection disabled, 1=enabled (WaveEN must be 1)
// [2]     ZxEN         : 0=Zero-cross/phase/voltage/frequency measurement disabled, 1=enabled
// [1]     PeakEN       : 0=Peak detection disabled, 1=enabled
// [0]     NC           : Reserved, read as 1 by default

        this->write_(CSE7761_REG_EMUCON2 | 0x80, 0x0FE5); // Dual R3/Pow CT + frequency measure enable

// Pulse1SEL (0x1D, default 0x3210)
// [15:12] NC        : Reserved, default = 0011
// [11:8]  NC        : Reserved, default = 0010
// [7:4]   P2Sel     : Pulse2 pin function select (see PxSel table below)
// [3:0]   P1Sel     : Pulse1 pin function select (see PxSel table below)
//
// PxSel function selection (applies to P1Sel and P2Sel):
// 0000 = Calibration pulse PFA (electric energy)
// 0001 = Calibration pulse PFB (electric energy)
// 0010 = Comparator indication signal (comp_sign)
// 0011 = Interrupt signal IRQ (default high; set low to use as interrupt)
// 0100 = Power overload indication (only PA or PB selectable)
// 0101 = Channel A negative power indication
// 0110 = Channel B negative power indication
// 0111 = Instantaneous value update interrupt
// 1000 = Mean value update interrupt
// 1001 = Voltage channel zero-crossing signal
// 1010 = Current Channel A zero-crossing signal
// 1011 = Current Channel B zero-crossing signal
// 1100 = Voltage channel overvoltage indication
// 1101 = Voltage channel undervoltage indication
// 1110 = Current Channel A overcurrent indication
// 1111 = Current Channel B overcurrent indication

// Example: enable zero-crossing signal output on function pin
// this->write_(CSE7761_REG_PULSE1SEL | 0x80, 0x3290);
    
  } else {
    ESP_LOGD(TAG, "Write failed at chip_init");
    return false;
  }
  return true;
}

void CSE7761Component::get_data_() {
    // Read RMS voltage (24-bit signed, highest bit = 0 for valid, 1 = invalid -> 0)
    uint32_t value = this->read_(CSE7761_REG_RMSU, 3);
    this->data_.voltage_rms = (value >= 0x800000) ? 0 : value;

    // Read voltage frequency (16-bit, highest bit = invalid)
    value = this->read_(CSE7761_REG_UFREQ, 2);
    this->data_.frequency = (value >= 0x8000) ? 0 : value;

    // Read RMS current channel A
    value = this->read_(CSE7761_REG_RMSIA, 3);
    this->data_.current_rms[0] = value;

    // Read Power channel A
    value = this->read_(CSE7761_REG_POWERPA, 4);
    this->data_.active_power[0] = (int32_t) value;

    // Read RMS current channel B
    value = this->read_(CSE7761_REG_RMSIB, 3);
    this->data_.current_rms[1] = value;

    // Read Power channel B
    value = this->read_(CSE7761_REG_POWERPB, 4);
    this->data_.active_power[1] = (int32_t) value;
    
    ESP_LOGD(TAG, "V:%d, F:%d, IA:%d, PowA:%d, IB:%d, PowB:%d", 
        this->data_.voltage_rms, this->data_.frequency, 
        this->data_.current_rms[0], this->data_.active_power[0], 
        this->data_.current_rms[1], this->data_.active_power[1]);
    // Convert RMS voltage and publish
    float voltage = (float) this->data_.voltage_rms / this->coefficient_by_unit_(RMS_UC);
    if (this->voltage_sensor_ != nullptr) this->voltage_sensor_->publish_state(voltage);

    // Convert frequency and publish
    if (this->frequency_sensor_ != nullptr) {
        float freq = this->data_.frequency ? ((float) CSE7761_FREF / 8 / this->data_.frequency) : 0; // Hz
        this->frequency_sensor_->publish_state(freq);
    }
    
    // Convert and publish current CH A and B (Pow CT has only CH A) (on DualR3 power is inverted)
    float active_power1 = 0, amps1 = 0, active_power2 = 0, amps2 = 0;
    if (this->data_.model != CSE7761_MODEL_SONOFF_POWCT) {
        // Convert and publish current CH A
        if (this->power_sensor_1_ != nullptr) {
            active_power1 = (float) this->data_.active_power[0] / this->coefficient_by_unit_(POWER_PAC) * (-1);
            active_power1 = (active_power1 < -1 || active_power1 > 1) ? active_power1 : 0;            // W
            this->power_sensor_1_->publish_state(active_power1);
        }

        if (this->current_sensor_1_ != nullptr) {
            amps1 = (float) this->data_.current_rms[0] / this->coefficient_by_unit_(RMS_IAC);
            amps1 = (amps1 < -0.005 || amps1 > 0.005) ? amps1 : 0.00001;                                  // A
            this->current_sensor_1_->publish_state(amps1);
        }

        if (this->pf_sensor_1_ != nullptr) {
            float pf = active_power1 / voltage / amps1;
            this->pf_sensor_1_->publish_state((pf <= 1 || pf >= -1) ? pf : 0);
        }

        if (this->power_sensor_2_ != nullptr) {
            active_power2 = (float) this->data_.active_power[1] / this->coefficient_by_unit_(POWER_PBC) * (-1); // W
            active_power2 = (active_power2 < -1 || active_power2 > 1) ? active_power2 : 0;
            this->power_sensor_2_->publish_state(active_power2);
        }

        if (this->current_sensor_2_ != nullptr) {
            amps2 = (float) this->data_.current_rms[1] / this->coefficient_by_unit_(RMS_IBC);           // A
            amps2 = (amps2 < -0.005 || amps2 > 0.005) ? amps2 : 0.00001;
            this->current_sensor_2_->publish_state(amps2);
        }

        if (this->pf_sensor_2_ != nullptr) {
            float pf = active_power2 / voltage / amps2;
            this->pf_sensor_2_->publish_state((pf <= 1 || pf >= -1) ? pf : 0);
        }
    } else {
        // Convert and publish CH A for POWCT
        if (this->power_sensor_1_ != nullptr) {
            active_power1 = (float) this->data_.active_power[0] / this->coefficient_by_unit_(POWER_PAC); // W
            active_power1 = (active_power1 < -1 || active_power1 > 1) ? active_power1 : 0;
            this->power_sensor_1_->publish_state(active_power1);
        }

        if (this->current_sensor_1_ != nullptr) {
            amps1 = (float) this->data_.current_rms[0] / this->coefficient_by_unit_(RMS_IAC);           // A
            amps1 = (amps1 < -0.005 || amps1 > 0.005) ? amps1 : 0.00001;                                 
            this->current_sensor_1_->publish_state(amps1);
        }

        if (this->pf_sensor_1_ != nullptr) {
            float pf = active_power1 / voltage / amps1;
            this->pf_sensor_1_->publish_state((pf <= 1 || pf >= -1) ? pf : 0);
        }
    }
}

}  // namespace cse7761
}  // namespace esphome
