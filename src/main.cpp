#include <Arduino.h>

#include <Wire.h>

#include <driver/i2s.h>

#include <cstring>

// ========== PIN CONFIGURATION ==========

#define I2C_SDA 5

#define I2C_SCL 6

#define TLV_RESET_PIN 19

// I2S Configuration

#define I2S_NUM         I2S_NUM_0

#define I2S_BCK_PIN     7    // Bit clock (BCLK) - connect to TLV BCLK

#define I2S_WS_PIN      14    // Word select (LRCLK/WCLK) - connect to TLV WCLK

#define I2S_DATA_IN_PIN 9    // Data in (DOUT from TLV) - connect to TLV DOUT

// ========== I2C CONFIGURATION ==========

#define I2C_FREQ 400000

#define ADC1_ADDRESS 0x1A

#define ADC2_ADDRESS 0x18

#define SAMPLE_RATE     48000

#define BITS_PER_SAMPLE 32

#define I2S_READ_LEN    (1024 * 2)

// ========== REGISTER DEFINITION STRUCTURE ==========

struct ADC3XXX_REG {
    uint8_t page;
    uint8_t reg;
    ADC3XXX_REG(uint8_t p = 0, uint8_t r = 0) : page(p), reg(r) {}
};

// ========== FILTER COEFFICIENTS CLASS ==========

class ADC3XXX_Filter_coefficients {
public:
    enum Mode { miniDSP, FIR, Bisquad };
    Mode mode;
    uint16_t register_pairs[64];

    ADC3XXX_Filter_coefficients(Mode m = miniDSP) : mode(m) {
        register_pairs[0] = 0b0000000100010111;
        register_pairs[1] = 0b0000000100010111;
        register_pairs[2] = 0b0111110111010011;
        register_pairs[3] = 0b0111111111111111;
        for (int i = 4; i < 64; i++) {
            if (i == 6 || i == 11 || i == 16 || i == 21 || i == 26 || i == 31) {
                register_pairs[i] = 0b0111111111111111;
            } else {
                register_pairs[i] = 0b0000000000000000;
            }
        }
    }
};

// ========== TLV320ADC3101 CLASS ==========

class TLV320ADC3101 {
private:
    uint8_t adc_i2c_address;
    uint8_t current_page;
    bool debug;
    bool verify_writes;
    int write_errors;

public:
    ADC3XXX_REG ADC3XXX_PAGE_SELECT;
    ADC3XXX_REG ADC3XXX_RESET;
    ADC3XXX_REG ADC3XXX_CLKGEN_MUX;
    ADC3XXX_REG ADC3XXX_PLL_PROG_PR;
    ADC3XXX_REG ADC3XXX_PLL_PROG_J;
    ADC3XXX_REG ADC3XXX_PLL_PROG_D_MSB;
    ADC3XXX_REG ADC3XXX_PLL_PROG_D_LSB;
    ADC3XXX_REG ADC3XXX_ADC_NADC;
    ADC3XXX_REG ADC3XXX_ADC_MADC;
    ADC3XXX_REG ADC3XXX_ADC_AOSR;
    ADC3XXX_REG ADC3XXX_ADC_IADC;
    ADC3XXX_REG ADC3XXX_CLKOUT_MUX;
    ADC3XXX_REG ADC3XXX_CLKOUT_M_DIV;
    ADC3XXX_REG ADC3XXX_INTERFACE_CTRL_1;
    ADC3XXX_REG ADC3XXX_CH_OFFSET_1;
    ADC3XXX_REG ADC3XXX_INTERFACE_CTRL_2;
    ADC3XXX_REG ADC3XXX_BCLK_N_DIV;
    ADC3XXX_REG ADC3XXX_INTERFACE_CTRL_3;
    ADC3XXX_REG ADC3XXX_INTERFACE_CTRL_4;
    ADC3XXX_REG ADC3XXX_INTERFACE_CTRL_5;
    ADC3XXX_REG ADC3XXX_I2S_SYNC;
    ADC3XXX_REG ADC3XXX_ADC_FLAG;
    ADC3XXX_REG ADC3XXX_CH_OFFSET_2;
    ADC3XXX_REG ADC3XXX_I2S_TDM_CTRL;
    ADC3XXX_REG ADC3XXX_INTR_FLAG_1;
    ADC3XXX_REG ADC3XXX_INTR_FLAG_2;
    ADC3XXX_REG ADC3XXX_INTR_FLAG_ADC1;
    ADC3XXX_REG ADC3XXX_INTR_FLAG_ADC2;
    ADC3XXX_REG ADC3XXX_INT1_CTRL;
    ADC3XXX_REG ADC3XXX_INT2_CTRL;
    ADC3XXX_REG ADC3XXX_GPIO2_CTRL;
    ADC3XXX_REG ADC3XXX_GPIO1_CTRL;
    ADC3XXX_REG ADC3XXX_DOUT_CTRL;
    ADC3XXX_REG ADC3XXX_SYNC_CTRL_1;
    ADC3XXX_REG ADC3XXX_SYNC_CTRL_2;
    ADC3XXX_REG ADC3XXX_CIC_GAIN_CTRL;
    ADC3XXX_REG ADC3XXX_PRB_SELECT;
    ADC3XXX_REG ADC3XXX_INST_MODE_CTRL;
    ADC3XXX_REG ADC3XXX_MIC_POLARITY_CTRL;
    ADC3XXX_REG ADC3XXX_ADC_DIGITAL;
    ADC3XXX_REG ADC3XXX_ADC_FINE_VOL_CTRL;
    ADC3XXX_REG ADC3XXX_LADC_VOL;
    ADC3XXX_REG ADC3XXX_RADC_VOL;
    ADC3XXX_REG ADC3XXX_ADC_PHASE_COMP;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_1;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_2;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_3;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_4;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_5;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_6;
    ADC3XXX_REG ADC3XXX_LEFT_CHN_AGC_7;
    ADC3XXX_REG ADC3XXX_LEFT_AGC_GAIN;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_1;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_2;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_3;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_4;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_5;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_6;
    ADC3XXX_REG ADC3XXX_RIGHT_CHN_AGC_7;
    ADC3XXX_REG ADC3XXX_RIGHT_AGC_GAIN;
    ADC3XXX_REG ADC3XXX_DITHER_CTRL;
    ADC3XXX_REG ADC3XXX_MICBIAS_CTRL;
    ADC3XXX_REG ADC3XXX_LEFT_PGA_SEL_1;
    ADC3XXX_REG ADC3XXX_LEFT_PGA_SEL_2;
    ADC3XXX_REG ADC3XXX_RIGHT_PGA_SEL_1;
    ADC3XXX_REG ADC3XXX_RIGHT_PGA_SEL_2;
    ADC3XXX_REG ADC3XXX_LEFT_APGA_CTRL;
    ADC3XXX_REG ADC3XXX_RIGHT_APGA_CTRL;
    ADC3XXX_REG ADC3XXX_LOW_CURRENT_MODES;
    ADC3XXX_REG ADC3XXX_ANALOG_PGA_FLAGS;

    TLV320ADC3101(uint8_t i2c_address)
        : adc_i2c_address(i2c_address), current_page(0xFF), debug(false), verify_writes(true), write_errors(0),
          ADC3XXX_PAGE_SELECT(0, 0),
          ADC3XXX_RESET(0, 1),
          ADC3XXX_CLKGEN_MUX(0, 4),
          ADC3XXX_PLL_PROG_PR(0, 5),
          ADC3XXX_PLL_PROG_J(0, 6),
          ADC3XXX_PLL_PROG_D_MSB(0, 7),
          ADC3XXX_PLL_PROG_D_LSB(0, 8),
          ADC3XXX_ADC_NADC(0, 18),
          ADC3XXX_ADC_MADC(0, 19),
          ADC3XXX_ADC_AOSR(0, 20),
          ADC3XXX_ADC_IADC(0, 21),
          ADC3XXX_CLKOUT_MUX(0, 25),
          ADC3XXX_CLKOUT_M_DIV(0, 26),
          ADC3XXX_INTERFACE_CTRL_1(0, 27),
          ADC3XXX_CH_OFFSET_1(0, 28),
          ADC3XXX_INTERFACE_CTRL_2(0, 29),
          ADC3XXX_BCLK_N_DIV(0, 30),
          ADC3XXX_INTERFACE_CTRL_3(0, 31),
          ADC3XXX_INTERFACE_CTRL_4(0, 32),
          ADC3XXX_INTERFACE_CTRL_5(0, 33),
          ADC3XXX_I2S_SYNC(0, 34),
          ADC3XXX_ADC_FLAG(0, 36),
          ADC3XXX_CH_OFFSET_2(0, 37),
          ADC3XXX_I2S_TDM_CTRL(0, 38),
          ADC3XXX_INTR_FLAG_1(0, 42),
          ADC3XXX_INTR_FLAG_2(0, 43),
          ADC3XXX_INTR_FLAG_ADC1(0, 45),
          ADC3XXX_INTR_FLAG_ADC2(0, 47),
          ADC3XXX_INT1_CTRL(0, 48),
          ADC3XXX_INT2_CTRL(0, 49),
          ADC3XXX_GPIO2_CTRL(0, 51),
          ADC3XXX_GPIO1_CTRL(0, 52),
          ADC3XXX_DOUT_CTRL(0, 53),
          ADC3XXX_SYNC_CTRL_1(0, 57),
          ADC3XXX_SYNC_CTRL_2(0, 58),
          ADC3XXX_CIC_GAIN_CTRL(0, 59),
          ADC3XXX_PRB_SELECT(0, 61),
          ADC3XXX_INST_MODE_CTRL(0, 62),
          ADC3XXX_MIC_POLARITY_CTRL(0, 80),
          ADC3XXX_ADC_DIGITAL(0, 81),
          ADC3XXX_ADC_FINE_VOL_CTRL(0, 82),
          ADC3XXX_LADC_VOL(0, 83),
          ADC3XXX_RADC_VOL(0, 84),
          ADC3XXX_ADC_PHASE_COMP(0, 85),
          ADC3XXX_LEFT_CHN_AGC_1(0, 86),
          ADC3XXX_LEFT_CHN_AGC_2(0, 87),
          ADC3XXX_LEFT_CHN_AGC_3(0, 88),
          ADC3XXX_LEFT_CHN_AGC_4(0, 89),
          ADC3XXX_LEFT_CHN_AGC_5(0, 90),
          ADC3XXX_LEFT_CHN_AGC_6(0, 91),
          ADC3XXX_LEFT_CHN_AGC_7(0, 92),
          ADC3XXX_LEFT_AGC_GAIN(0, 93),
          ADC3XXX_RIGHT_CHN_AGC_1(0, 94),
          ADC3XXX_RIGHT_CHN_AGC_2(0, 95),
          ADC3XXX_RIGHT_CHN_AGC_3(0, 96),
          ADC3XXX_RIGHT_CHN_AGC_4(0, 97),
          ADC3XXX_RIGHT_CHN_AGC_5(0, 98),
          ADC3XXX_RIGHT_CHN_AGC_6(0, 99),
          ADC3XXX_RIGHT_CHN_AGC_7(0, 100),
          ADC3XXX_RIGHT_AGC_GAIN(0, 101),
          ADC3XXX_DITHER_CTRL(1, 26),
          ADC3XXX_MICBIAS_CTRL(1, 51),
          ADC3XXX_LEFT_PGA_SEL_1(1, 52),
          ADC3XXX_LEFT_PGA_SEL_2(1, 54),
          ADC3XXX_RIGHT_PGA_SEL_1(1, 55),
          ADC3XXX_RIGHT_PGA_SEL_2(1, 57),
          ADC3XXX_LEFT_APGA_CTRL(1, 59),
          ADC3XXX_RIGHT_APGA_CTRL(1, 60),
          ADC3XXX_LOW_CURRENT_MODES(1, 61),
          ADC3XXX_ANALOG_PGA_FLAGS(1, 62) {}

    void enable_verification(bool enable) { verify_writes = enable; }

    int get_write_errors() { return write_errors; }

    void reset_error_count() { write_errors = 0; }

    void sw_reset() {
        i2c_write(ADC3XXX_RESET, 1);
    }

    uint8_t i2c_read(ADC3XXX_REG& reg) {
        if (current_page != reg.page) {
            current_page = reg.page;
            Wire.beginTransmission(adc_i2c_address);
            Wire.write(0);
            Wire.write(reg.page);
            Wire.endTransmission();
        }

        Wire.beginTransmission(adc_i2c_address);
        Wire.write(reg.reg);
        if (Wire.endTransmission() != 0) return 0xFF;

        Wire.requestFrom(adc_i2c_address, (uint8_t)1);
        if (Wire.available()) return Wire.read();

        return 0xFF;
    }

    void set_pll_config(bool pll_power_up, uint8_t pll_div_P, uint8_t pll_multiplier_R) {
        uint8_t config = apply_bits(0, 7, 1, pll_power_up);
        config = apply_bits(config, 6, 3, pll_div_P);
        config = apply_bits(config, 3, 4, pll_multiplier_R);
        i2c_write(ADC3XXX_PLL_PROG_PR, config);
    }

    void set_MADC_clock_div(bool MADC_clock_div_power_up, uint8_t div) {
        uint8_t config = apply_bits(0, 7, 1, MADC_clock_div_power_up);
        config = apply_bits(config, 6, 7, div);
        i2c_write(ADC3XXX_ADC_MADC, config);
    }

    void set_NADC_clock_div(bool NADC_clock_div_power_up, uint8_t div) {
        uint8_t config = apply_bits(0, 7, 1, NADC_clock_div_power_up);
        config = apply_bits(config, 6, 7, div);
        i2c_write(ADC3XXX_ADC_NADC, config);
    }

    void configure_clockgen(const char* PLL_CLK_source, const char* CODEC_CLK_source) {
        uint8_t PLL_src = 0, CODEC_src = 0;

        if (strcmp(PLL_CLK_source, "MCLK") == 0) PLL_src = 0b00;
        else if (strcmp(PLL_CLK_source, "BCLK") == 0) PLL_src = 0b01;
        else if (strcmp(PLL_CLK_source, "logic level 0") == 0) PLL_src = 0b11;

        if (strcmp(CODEC_CLK_source, "MCLK") == 0) CODEC_src = 0b00;
        else if (strcmp(CODEC_CLK_source, "BCLK") == 0) CODEC_src = 0b01;
        else if (strcmp(CODEC_CLK_source, "PLL_CLK") == 0) CODEC_src = 0b11;

        uint8_t config = apply_bits(0, 3, 2, PLL_src);
        config = apply_bits(config, 1, 2, CODEC_src);
        i2c_write(ADC3XXX_CLKGEN_MUX, config);
    }

    void set_AOSR(uint8_t AOSR) {
        uint8_t config = apply_bits(0, 7, 8, AOSR);
        i2c_write(ADC3XXX_ADC_AOSR, config);
    }

    void set_PLL_mult_J(uint8_t PLL_MULT_J) {
        if (PLL_MULT_J == 0) return;
        uint8_t config = apply_bits(0, 5, 6, PLL_MULT_J);
        i2c_write(ADC3XXX_PLL_PROG_J, config);
    }

    void set_PLL_frac_mult_D(uint16_t PLL_frac_mult_D) {
        uint16_t config = apply_bits_16(PLL_frac_mult_D, 15, 16, 600, 16);
        uint8_t lsb = config & 0xFF;
        uint8_t msb = (config >> 8) & 0xFF;

        bool prev_verify = verify_writes;
        verify_writes = false;
        i2c_write(ADC3XXX_PLL_PROG_D_LSB, lsb);
        i2c_write(ADC3XXX_PLL_PROG_D_MSB, msb);
        verify_writes = prev_verify;
    }

    void select_processing_block(const char* processing_block) {
        uint8_t config = 0;
        if (strcmp(processing_block, "programmable") == 0) {
            config = apply_bits(0, 4, 5, 0);
        } else if (strstr(processing_block, "PRB_R") != NULL) {
            int block_num = atoi(processing_block + 5);
            config = apply_bits(0, 4, 5, block_num);
        }
        i2c_write(ADC3XXX_PRB_SELECT, config);
    }

    void config_adc_audio_interface(const char* interface = "I2S", uint8_t word_length = 16,
                                   bool BCLK_is_output = false, bool WCLK_is_output = false,
                                   bool DOUT_3_stating = false) {
        uint8_t interface_val = 0, word_length_val = 0;

        if (strcmp(interface, "I2S") == 0) interface_val = 0b00;
        else if (strcmp(interface, "DSP") == 0) interface_val = 0b01;
        else if (strcmp(interface, "RJF") == 0) interface_val = 0b10;
        else if (strcmp(interface, "LJF") == 0) interface_val = 0b11;

        if (word_length == 16) word_length_val = 0b00;
        else if (word_length == 20) word_length_val = 0b01;
        else if (word_length == 24) word_length_val = 0b10;
        else if (word_length == 32) word_length_val = 0b11;

        uint8_t config = apply_bits(0, 7, 2, interface_val);
        config = apply_bits(config, 5, 2, word_length_val);
        config = apply_bits(config, 3, 1, BCLK_is_output);
        config = apply_bits(config, 2, 1, WCLK_is_output);
        config = apply_bits(config, 0, 1, DOUT_3_stating);

        i2c_write(ADC3XXX_INTERFACE_CTRL_1, config);

        config = apply_bits(0, 1, 2, 0b10);  // BDIV_CLK = ADC_CLK
        i2c_write(ADC3XXX_INTERFACE_CTRL_2, config);
    }

    void config_dout(bool disable_DOUT_bus_keeper = true, const char* DOUT_output_source = "primary") {
        uint8_t output_src = 0;

        if (strcmp(DOUT_output_source, "disabled") == 0) output_src = 0b000;
        else if (strcmp(DOUT_output_source, "primary") == 0) output_src = 0b001;
        else if (strcmp(DOUT_output_source, "general purpose") == 0) output_src = 0b010;
        else if (strcmp(DOUT_output_source, "CLKOUT") == 0) output_src = 0b011;
        else if (strcmp(DOUT_output_source, "INT1") == 0) output_src = 0b100;
        else if (strcmp(DOUT_output_source, "INT2") == 0) output_src = 0b101;
        else if (strcmp(DOUT_output_source, "BCLK") == 0) output_src = 0b110;
        else if (strcmp(DOUT_output_source, "WCLK") == 0) output_src = 0b111;

        uint8_t config = apply_bits(0, 4, 1, disable_DOUT_bus_keeper);
        config = apply_bits(config, 3, 3, output_src);
        i2c_write(ADC3XXX_DOUT_CTRL, config);
    }

    void configure_adc_digital(bool left_channel_powered = false, bool right_channel_powered = false) {
        uint8_t config = apply_bits(0, 7, 1, left_channel_powered);
        config = apply_bits(config, 6, 1, right_channel_powered);
        config = apply_bits(config, 1, 2, 0b10);  // Volume control disabled
        i2c_write(ADC3XXX_ADC_DIGITAL, config);
    }

    void i2c_write(ADC3XXX_REG& reg, uint8_t msg) {
        if (current_page != reg.page) {
            current_page = reg.page;
            Wire.beginTransmission(adc_i2c_address);
            Wire.write(0);
            Wire.write(reg.page);
            if (Wire.endTransmission() != 0) {
                write_errors++;
                return;
            }
        }

        Wire.beginTransmission(adc_i2c_address);
        Wire.write(reg.reg);
        Wire.write(msg);
        if (Wire.endTransmission() != 0) {
            write_errors++;
            return;
        }

        if (verify_writes && reg.reg != 1) {
            delay(1);
            uint8_t read_back = i2c_read(reg);
            if (read_back != msg) {
                write_errors++;
            }
        }
    }

    uint8_t apply_bits(uint8_t x, uint8_t msb, uint8_t field_size, uint8_t val) {
        uint8_t mask = 0xFF - ((1 << field_size) - 1) * (1 << (msb - field_size + 1));
        uint8_t shifted_val = val << (msb - field_size + 1);
        return (x & mask) | shifted_val;
    }

    uint16_t apply_bits_16(uint16_t x, uint8_t msb, uint8_t field_size, uint16_t val, uint8_t word_size) {
        uint16_t mask = (1 << word_size) - 1 - ((1 << field_size) - 1) * (1 << (msb - field_size + 1));
        uint16_t shifted_val = val << (msb - field_size + 1);
        return (x & mask) | shifted_val;
    }
};

// ========== GLOBAL INSTANCES ==========

TLV320ADC3101 ADC1(ADC1_ADDRESS);
TLV320ADC3101 ADC2(ADC2_ADDRESS);

// ========== ADC CONFIGURATION ==========
void config_ADC(TLV320ADC3101& ADC, uint8_t tdm, bool enable) {
    ADC.reset_error_count();
    ADC.sw_reset();
    delay(100);

    // Clock configuration
    ADC.set_pll_config(false, 1, 1);
    ADC.set_NADC_clock_div(false, 1);
    ADC.set_MADC_clock_div(false, 1);
    ADC.i2c_write(ADC.ADC3XXX_BCLK_N_DIV, 0b00000001);
    ADC.configure_clockgen("BCLK", "PLL_CLK");
    ADC.set_AOSR(128);
    ADC.set_pll_config(true, 1, 1);
    ADC.set_PLL_mult_J(32);
    ADC.set_PLL_frac_mult_D(0);
    ADC.set_NADC_clock_div(false, 8);
    ADC.set_MADC_clock_div(true, 2);
    ADC.select_processing_block("PRB_R2");

    // 32-bit I2S interface
    ADC.config_adc_audio_interface("I2S", 32, false, false, true);

    // Digital channels powered up
    ADC.configure_adc_digital(true, true);

    // ✅ Input routing with -6 dB attenuation to prevent clipping
    // Left channel: IN1LP + IN1RM differential pair with -6 dB attenuation
ADC.i2c_write(ADC.ADC3XXX_LEFT_PGA_SEL_1, 0b11111110);   // IN1LP bits[1:0] = 10 (-12dB)
ADC.i2c_write(ADC.ADC3XXX_LEFT_PGA_SEL_2, 0b00000000);   // NOT differential (single-ended)

// Right channel: IN2RP only (single-ended, -12 dB attenuation)
ADC.i2c_write(ADC.ADC3XXX_RIGHT_PGA_SEL_1, 0b11111011);  // IN2RP bits[3:2] = 10 (-12dB)
ADC.i2c_write(ADC.ADC3XXX_RIGHT_PGA_SEL_2, 0b00000000);  // NOT differential (single-ended)

    // Keep PGA gain at 0 dB (your working baseline)
    ADC.i2c_write(ADC.ADC3XXX_LEFT_APGA_CTRL, 0b00000000);   // 0 dB gain
    ADC.i2c_write(ADC.ADC3XXX_RIGHT_APGA_CTRL, 0b00000000);  // 0 dB gain

    // Dither and volume control
    ADC.i2c_write(ADC.ADC3XXX_DITHER_CTRL, 0b00000000);
    ADC.i2c_write(ADC.ADC3XXX_ADC_FINE_VOL_CTRL, 0b00000000);

    // Microphone bias
    ADC.i2c_write(ADC.ADC3XXX_MICBIAS_CTRL, 0b01010000);

    // Enable NADC clock
    ADC.set_NADC_clock_div(true, 8);

    // TDM configuration
    ADC.i2c_write(ADC.ADC3XXX_I2S_TDM_CTRL, 0b00000010);
    ADC.i2c_write(ADC.ADC3XXX_CH_OFFSET_1, tdm);

    // Enable DOUT if requested
    if (enable) {
        ADC.config_dout();
    }

    if (ADC.get_write_errors() > 0) {
        Serial.printf("⚠ Configuration had %d verification errors\n", ADC.get_write_errors());
    } else {
        Serial.println("✓ Configuration successful (0 errors)");
    }
}


// ========== I2S SETUP ==========

void setup_i2s() {
    Serial.println("\n=== I2S Configuration ===");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_IN_PIN
    };

    esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("❌ ERROR: i2s_driver_install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_NUM, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("❌ ERROR: i2s_set_pin failed: %d\n", err);
        return;
    }

    Serial.println("✓ I2S initialized successfully");
    Serial.printf("  Sample Rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("  BCK: GPIO%d, WS: GPIO%d, DIN: GPIO%d\n", I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_IN_PIN);
}

// ========== I2S AUDIO TEST ==========

void test_i2s_audio() {
    static uint8_t i2s_read_buff[I2S_READ_LEN];
    size_t bytes_read = 0;

    esp_err_t result = i2s_read(I2S_NUM, i2s_read_buff, I2S_READ_LEN, &bytes_read, 1000 / portTICK_PERIOD_MS);

    if (result == ESP_OK && bytes_read > 0) {
        int32_t* samples = (int32_t*)i2s_read_buff;
        int num_samples = bytes_read / 4;

        int32_t max_val = INT32_MIN;
        int32_t min_val = INT32_MAX;
        int64_t sum = 0;
        int non_zero = 0;

        for (int i = 0; i < num_samples; i++) {
            int32_t sample = samples[i];
            if (sample > max_val) max_val = sample;
            if (sample < min_val) min_val = sample;
            sum += abs(sample);
            if (sample != 0) non_zero++;
        }

        float avg = (float)sum / num_samples;

        Serial.println("\n=== I2S Audio Test ===");
        Serial.printf("Samples: %d | Non-zero: %d (%.1f%%)\n", num_samples, non_zero, (100.0 * non_zero) / num_samples);
        Serial.printf("Range: %d to %d\n", min_val, max_val);
        Serial.printf("Average: %.0f\n", avg);

        Serial.println("\nFirst 8 samples:");
        for (int i = 0; i < 8 && i < num_samples; i++) {
            Serial.printf("  [%d]: 0x%08X (%d)\n", i, samples[i], samples[i]);
        }

        if (non_zero == 0) {
            Serial.println("\n❌ ERROR: All samples ZERO - No I2S data!");
            Serial.println("Check:");
            Serial.println("  - I2S wiring (BCK/WS/DOUT)");
            Serial.println("  - TLV DOUT enabled");
            Serial.println("  - Audio input connected");
        } else if (avg < 100) {
            Serial.println("\n⚠ WARNING: Signal very weak - adjust gain");
        } else if (max_val == INT32_MAX || min_val == INT32_MIN) {
            Serial.println("\n⚠ WARNING: Clipping detected!");
        } else {
            Serial.println("\n✓ Audio signal detected!");
        }
    } else {
        Serial.printf("\n❌ I2S read failed: %d, bytes: %d\n", result, bytes_read);
    }
}

// ========== AUDIO RECORDING FUNCTION ==========

void record_audio_to_serial(int duration_seconds) {
    Serial.println("START_RECORDING");
    delay(100);

    size_t total_samples = SAMPLE_RATE * duration_seconds;
    size_t samples_sent = 0;
    uint8_t i2s_read_buff[I2S_READ_LEN];

    while (samples_sent < total_samples) {
        size_t bytes_read = 0;
        esp_err_t result = i2s_read(I2S_NUM, i2s_read_buff, I2S_READ_LEN,
                                    &bytes_read, portMAX_DELAY);

        if (result == ESP_OK && bytes_read > 0) {
            int32_t* samples = (int32_t*)i2s_read_buff;
            int num_samples = bytes_read / 4;

            for (int i = 0; i < num_samples && samples_sent < total_samples; i++) {
                int32_t sample_32 = samples[i];

                // Convert 32-bit to 16-bit: shift right by 16 to get upper 16 bits
                int16_t sample_16 = (int16_t)(sample_32 >> 16);  // Shift by 15 instead


                Serial.write((uint8_t*)&sample_16, sizeof(int16_t));
                samples_sent++;
            }
        }
    }

    Serial.println("\nEND_RECORDING");
}

// ========== SETUP ==========

void setup() {
    // ✅ Increased baud rate to 921600 for faster serial transfer
    Serial.begin(2000000);
    while (!Serial && millis() < 5000) delay(100);

    Serial.println("\n\n========================================");
    Serial.println("ESP32 TLV320ADC3101 + I2S Test");
    Serial.println("Improved Version with Register Fixes");
    Serial.println("========================================\n");

    // Hardware reset
    pinMode(TLV_RESET_PIN, OUTPUT);
    digitalWrite(TLV_RESET_PIN, HIGH);
    delay(100);
    digitalWrite(TLV_RESET_PIN, LOW);
    delay(100);
    digitalWrite(TLV_RESET_PIN, HIGH);
    delay(100);

    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(100);

    Serial.println("I2C Initialized");
    Serial.printf("  SDA: GPIO%d, SCL: GPIO%d, Freq: %d kHz\n\n", I2C_SDA, I2C_SCL, I2C_FREQ / 1000);

    // I2C scan
    Serial.println("Scanning I2C bus...");
    uint8_t found = 0;

    for (uint8_t i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X", i);
            if (i == ADC1_ADDRESS) Serial.print(" (ADC1)");
            if (i == ADC2_ADDRESS) Serial.print(" (ADC2)");
            Serial.println();
            found++;
        }
    }

    Serial.printf("Found %d devices\n", found);

    if (found == 0) {
        Serial.println("\n❌ ERROR: No I2C devices found!");
        return;
    }

    // Configure ADCs with improved settings
    Serial.println("\n--- Configuring ADC1 (0x1A) ---");
    config_ADC(ADC1, 0b00000000, true);

    Serial.println("\n--- Configuring ADC2 (0x18) (disabled for now) ---");
    // config_ADC(ADC2, 0b00100000, true);  // Disabled for single-channel testing

    // Setup I2S
    setup_i2s();

    Serial.println("\n✓ Setup complete! Waiting for ADCs to stabilize...");
    delay(2000);

    Serial.println("\nCommands:");
    Serial.println("  't' - Test audio (show samples)");
    Serial.println("  'r' - Record 5 seconds and stream to PC");
    Serial.println("\n▶ Ready for commands...\n");
}

// ========== LOOP ==========

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        if (command == 'r' || command == 'R') {
            Serial.println("Recording 5 seconds of audio...");
            delay(500);
            record_audio_to_serial(5);
        } else if (command == 't' || command == 'T') {
            test_i2s_audio();
        }
    }

    delay(10);
}
