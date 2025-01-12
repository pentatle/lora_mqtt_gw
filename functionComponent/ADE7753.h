#ifndef ADE7753_H
#define ADE7753_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

// Định nghĩa các chân kết nối
#define PIN_NUM_MISO 22
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

// Tốc độ SPI (4MHz)
#define SPI_FREQ_HZ 1 * 1000 * 1000

//Register addresses
//------Name----------------Address----No.Bits(Bytes)
#define ADE7753_WAVEFORM    0x01    //[24 (3)
#define ADE7753_AENERGY     0x02    // 24 (3)
#define ADE7753_RAENERGY    0x03    // 24 (3)
#define ADE7753_LAENERGY    0x04    // 24 (3)
#define ADE7753_VAENERGY    0x05    // 24 (3)
#define ADE7753_RVAENERGY   0x06    // 24 (3)
#define ADE7753_LVAENERGY   0x07    // 24 (3)
#define ADE7753_LVARENERGY  0x08    // 24 (3)
#define ADE7753_MODE        0x09    // 16 (2)
#define ADE7753_IRQEN       0x0A    // 16 (2)
#define ADE7753_STATUS      0x0B    // 16 (2)
#define ADE7753_RSTSTATUS   0x0C    // 16 (2)
#define ADE7753_CH1OS       0x0D    // 8  (1)
#define ADE7753_CH2OS       0x0E    // 8  (1)
#define ADE7753_GAIN        0x0F    // 8  (1)
#define ADE7753_PHCAL       0x10    // 6  (1)
#define ADE7753_APOS        0x11    // 16 (2)
#define ADE7753_WGAIN       0x12    // 12 (2)
#define ADE7753_WDIV        0x13    // 8  (1)
#define ADE7753_CFNUM       0x14    // 12 (2)
#define ADE7753_CFDEN       0x15    // 12 (2)
#define ADE7753_IRMS        0x16    // 24 (3)
#define ADE7753_VRMS        0x17    // 24 (3)
#define ADE7753_IRMSOS      0x18    // 12 (2)
#define ADE7753_VRMSOS      0x19    // 12 (2)
#define ADE7753_VAGAIN      0x1A    // 12 (2)
#define ADE7753_VADIV       0x1B    // 8  (1)
#define ADE7753_LINECYC     0x1C    // 16 (2)
#define ADE7753_ZXTOUT      0x1D    // 12 (2)
#define ADE7753_SAGCYC      0x1E    // 8  (1)
#define ADE7753_SAGLVL      0x1F    // 8  (1)
#define ADE7753_IPKLVL      0x20    // 8  (1)
#define ADE7753_VPKLVL      0x21    // 8  (1)
#define ADE7753_IPEAK       0x22    // 24 (3)
#define ADE7753_RSTIPEAK    0x23    // 24 (3)
#define ADE7753_VPEAK       0x24    // 24 (3)
#define ADE7753_RSTVPEAK    0x25    // 24 (3)
#define ADE7753_TEMP        0x26    // 8  (1)
#define ADE7753_PERIOD      0x27    // 16 (2)
#define ADE7753_TMODE       0x3D    // 8  (1)
#define ADE7753_CHKSUM      0x3E    // 6  (1)
#define ADE7753_DIEREV      0X3F    // 8  (1)

// Mode Register Mask
#define ADE7753_DISHPF      0x0001
#define ADE7753_DISLPF2     0x0002
#define ADE7753_DISCF       0x0004
#define ADE7753_DISSAG      0x0008
#define ADE7753_ASUSPEND    0x0010
#define ADE7753_TEMPSEL     0x0020
#define ADE7753_SWRST       0x0040
#define ADE7753_CYCMODE     0x0080
#define ADE7753_DISCH1      0x0100
#define ADE7753_DISCH2      0x0200
#define ADE7753_SWAP        0x0400
#define ADE7753_DTRT        0x1800
#define ADE7753_WAVSEL      0x6000
#define ADE7753_POAM        0x8000

// Interrupt Status Register Mask
#define ADE7753_AEHF        0x0001
#define ADE7753_SAG         0x0002
#define ADE7753_CYCEND      0x0004
#define ADE7753_WSMP        0x0008
#define ADE7753_ZX          0x0010
#define ADE7753_TEMPC       0x0020
#define ADE7753_RESET       0x0040
#define ADE7753_AEOF        0x0080
#define ADE7753_PKV         0x0100
#define ADE7753_PKI         0x0200
#define ADE7753_VAEHF       0x0400
#define ADE7753_VAEOF       0x0800
#define ADE7753_ZXTO        0x1000
#define ADE7753_PPOS        0x2000
#define ADE7753_PNEG        0x4000

// Channel 1 & 2 Offset Adjust and Gain Mask
#define ADE7753_OS_VAL      0x1F
#define ADE7753_OS_SIGN     0x20
#define ADE7753_ITGR_EN     0x80
#define ADE7753_CH1_GAIN    0x07
#define ADE7753_CH1_FS      0x18
#define ADE7753_CH2_GAIN    0xE0

// Configuration Constants
#define ADE7753_ITGR_OFF    0x00
#define ADE7753_ITGR_ON     0x80
#define ADE7753_GAIN_1      0x00
#define ADE7753_GAIN_2      0x21
#define ADE7753_GAIN_4      0x42
#define ADE7753_GAIN_8      0x63
#define ADE7753_GAIN_16     0x84
#define ADE7753_FS_0_5V     0x00
#define ADE7753_FS_0_25V    0x08
#define ADE7753_FS_0_125V   0x10
#define ADE7753_DR_1_128    0x0000
#define ADE7753_DR_1_256    0x0800
#define ADE7753_DR_1_512    0x1000
#define ADE7753_DR_1_1024   0x1800
#define ADE7753_WAV_PWR     0x0000
#define ADE7753_WAV_CH1     0x4000
#define ADE7753_WAV_CH2     0x6000

#define ADE7753_SPI_FREQ    10000000
#define ADE7753_TRA_DEL     4

#define LOW               0x0
#define HIGH              0x1


void ADE7753_transfer(uint8_t address, uint8_t *data, uint8_t length);
void ADE7753_writeRegister(uint8_t address, uint16_t value, uint8_t length);
uint32_t ADE7753_readRegister(uint8_t address, uint8_t length);
void ADE7753_writeMaskRegister(uint8_t address, uint16_t value, uint8_t length, uint16_t mask);
void ADE7753_reset();
void init_spi(void);
void ADE7753_begin();
void ADE7753_setCh1Gain(uint8_t gain);
void ADE7753_setCh2Gain(uint8_t gain);
void ADE7753_setCh1FullScale(uint8_t fs);
int32_t ADE7753_readWaveForm();
int32_t ADE7753_readAEnergy();
int32_t ADE7753_readLineAEnergy();
uint32_t ADE7753_readVAEnergy();
uint32_t ADE7753_readLineVAEnergy();
uint16_t ADE7753_readPeriod();
uint32_t ADE7753_readCurrentRMS();
uint32_t ADE7753_readVoltageRMS();

#endif