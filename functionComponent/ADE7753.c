#include "ADE7753.h"

spi_device_handle_t spi;

#define TAG "ADE7753"

void ADE7753_transfer(uint8_t address, uint8_t *data, uint8_t length)
{
    // Cấu hình SPI giao dịch
    spi_transaction_t t = {
        .length = 8 * (length + 1),   // Độ dài giao dịch (address + data)
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    uint8_t buffer[length + 1];       // Tạo buffer tạm cho address + data
    buffer[0] = address;              // Ghi address vào buffer

    // Sao chép dữ liệu vào buffer
    for (uint8_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }

    // Tạo transaction cho đọc/ghi
    t.tx_buffer = buffer;
    t.rx_buffer = buffer;

    // Kéo CS xuống để bắt đầu giao dịch
    gpio_set_level(PIN_NUM_CS, 0);

    // Giao dịch SPI
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE("ADE7753", "SPI transmit failed: %s", esp_err_to_name(ret));
    }

    // Trả kết quả đọc vào mảng data
    for (uint8_t i = 0; i < length; i++) {
        data[i] = buffer[i + 1];  // Kết quả đọc từ ADE7753 bắt đầu từ vị trí thứ 2 trong buffer
    }

    // Kéo CS lên sau khi hoàn tất giao dịch
    gpio_set_level(PIN_NUM_CS, 1);

    // Delay nhỏ để ổn định ADE7753
    vTaskDelay(pdMS_TO_TICKS(ADE7753_TRA_DEL));
}


void ADE7753_writeRegister(uint8_t address, uint16_t value, uint8_t length)
{
    // Hold max 16-bit data (2 bytes)
    uint8_t data[2];
    // Break value for SPI transfer
    for (int8_t i=length-1; i>=0; i--) {
        data[i] = value;
        value = value >> 8;
    }
    ADE7753_transfer(address | 0x80, data, length);
}

uint32_t ADE7753_readRegister(uint8_t address, uint8_t length)
{
    // Hold max 24-bit data (3 bytes)
    uint8_t data[3];
    ADE7753_transfer(address, data, length);
    // Combine register value from SPI transfer data
    uint32_t value = 0;
    for (int8_t i=0; i<length; i++) {
        value = value << 8 | data[i];
    }
    // Convert 8, 16, 24 bit negative number to 32 bit negative number
    uint8_t nbits = length * 8;
    if (value & ((uint32_t) 1 << (nbits - 1))) {
        return value | (0xFFFFFFFF << nbits);
    }
    return value;
}

void ADE7753_writeMaskRegister(uint8_t address, uint16_t value, uint8_t length, uint16_t mask)
{
    uint16_t reg = ADE7753_readRegister(address, length);
    // Replace some bits in register with input value
    reg = (reg & ~mask) | (value & mask);
    ADE7753_writeRegister(address, reg, length);
}

void ADE7753_reset()
{
    ADE7753_writeMaskRegister(ADE7753_MODE, ADE7753_SWRST, 2, ADE7753_SWRST);
    vTaskDelay(pdMS_TO_TICKS(18));
    ADE7753_writeMaskRegister(ADE7753_MODE, 0x0000, 2, ADE7753_SWRST);
}

// Khởi tạo SPI
void init_spi(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,  // Tốc độ 4MHz
        .mode = 1,                      // SPI mode 1
        .spics_io_num = PIN_NUM_CS,     // Chân CS
        .queue_size = 7,
        .flags = 0                      // Không sử dụng SPI_DEVICE_HALFDUPLEX
    };


    // Khởi tạo SPI bus
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    ESP_LOGI("SPI", "SPI initialized successfully");
}

void ADE7753_begin()
{
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, HIGH);
    init_spi();
    ADE7753_reset();
    
}

void ADE7753_setCh1Gain(uint8_t gain)
{
    ADE7753_writeMaskRegister(ADE7753_GAIN, gain, 1, ADE7753_CH1_GAIN);
}

void ADE7753_setCh2Gain(uint8_t gain)
{
    ADE7753_writeMaskRegister(ADE7753_GAIN, gain, 1, ADE7753_CH2_GAIN);
}

void ADE7753_setCh1FullScale(uint8_t fs)
{
    ADE7753_writeMaskRegister(ADE7753_GAIN, fs, 1, ADE7753_CH1_FS);
}

int32_t ADE7753_readWaveForm()
{
    return ADE7753_readRegister(ADE7753_WAVEFORM, 3);
}

int32_t ADE7753_readAEnergy()
{
    return ADE7753_readRegister(ADE7753_AENERGY, 3);
}

int32_t ADE7753_readLineAEnergy()
{
    return ADE7753_readRegister(ADE7753_LAENERGY, 3);
}

uint32_t ADE7753_readVAEnergy()
{
    return ADE7753_readRegister(ADE7753_VAENERGY, 3);
}

uint32_t ADE7753_readLineVAEnergy()
{
    return ADE7753_readRegister(ADE7753_LVAENERGY, 3);
}

uint16_t ADE7753_readPeriod()
{
    return ADE7753_readRegister(ADE7753_PERIOD, 2);
}

uint32_t ADE7753_readCurrentRMS()
{
    return ADE7753_readRegister(ADE7753_IRMS, 3);
}

uint32_t ADE7753_readVoltageRMS()
{
    return ADE7753_readRegister(ADE7753_VRMS, 3);
}