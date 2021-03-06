#pragma once

#include "main.h"

#define I3G_WHO_AM_I 0x0F
#define I3G_CTRL_REG1 0x20
#define I3G_CTRL_REG2 0x21
#define I3G_CTRL_REG3 0x22
#define I3G_CTRL_REG4 0x23
#define I3G_CTRL_REG5 0x24
#define I3G_REFERENCE 0x25
#define I3G_OUT_TEMP 0x26
#define I3G_STATUS_REG 0x27
#define I3G_OUT_X_L 0x28
#define I3G_OUT_X_H 0x29
#define I3G_OUT_Y_L 0x2A
#define I3G_OUT_Y_H 0x2B
#define I3G_OUT_Z_L 0x2C
#define I3G_OUT_Z_H 0x2D
#define I3G_FIFO_CTRL_REG 0x2E
#define I3G_FIFO_SRC_REG 0x2F
#define I3G_INT1_CFG 0x30
#define I3G_INT1_SRC 0x31
#define I3G_INT1_THS_XH 0x32
#define I3G_INT1_THS_XL 0x33
#define I3G_INT1_THS_YH 0x34
#define I3G_INT1_THS_YL 0x35
#define I3G_INT1_THS_ZH 0x36
#define I3G_INT1_THS_ZL 0x37
#define I3G_INT1_DURATION 0x38

#define I3G_245dsp 0b00
#define I3G_500dsp 0b01
#define I3G_2000dsp 0b10

#define I3G_speed I3G_2000dsp

#define I3G_245sp_coef 0.007477f
#define I3G_500sp_coef 0.015259f
#define I3G_2000sp_coef 0.061037f

#define I3G_X_bias 0.55f
#define I3G_Y_bias 2.0f
#define I3G_Z_bias -0.3f

typedef struct {
  SPI_HandleTypeDef *spi_handler;
  GPIO_TypeDef *CS_port;
  uint16_t CS_pin;
} I3G4250D_t;

typedef struct {
  float x, y, z;
} vang_t;

void GYRO_init(I3G4250D_t *);
uint8_t GYRO_read_reg(I3G4250D_t *, uint8_t);
void GYRO_write_reg(I3G4250D_t *, uint8_t, uint8_t);
int16_t GYRO_X(I3G4250D_t *);
int16_t GYRO_Y(I3G4250D_t *);
int16_t GYRO_Z(I3G4250D_t *);
void GYRO_ang(I3G4250D_t *, vang_t *);
void GYRO_set_dsp(I3G4250D_t *, uint8_t);