#include "I3G4250D.h"

void GYRO_init(I3G4250D_t *gyro) {
  GYRO_write_reg(gyro, I3G_CTRL_REG1, 0xFF);
}

uint8_t GYRO_read_reg(I3G4250D_t *gyro, uint8_t reg) {
  uint8_t spi_buffer[] = {0x80 | reg};
  HAL_GPIO_WritePin(gyro->CS_port, gyro->CS_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(gyro->spi_handler, spi_buffer, 1, 1000);
  HAL_SPI_Receive(gyro->spi_handler, spi_buffer, 1, 1000);
  HAL_GPIO_WritePin(gyro->CS_port, gyro->CS_pin, GPIO_PIN_SET);
  return spi_buffer[0];
}

void GYRO_write_reg(I3G4250D_t *gyro, uint8_t reg, uint8_t data) {
  uint8_t spi_buffer[] = {reg, data};
  HAL_GPIO_WritePin(gyro->CS_port, gyro->CS_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(gyro->spi_handler, spi_buffer, 2, 1000);
  HAL_GPIO_WritePin(gyro->CS_port, gyro->CS_pin, GPIO_PIN_SET);
  return;
}

int16_t GYRO_X(I3G4250D_t *gyro) {
  uint16_t data = 0;
  data = GYRO_read_reg(gyro, I3G_OUT_X_H) << 8;
  data |= GYRO_read_reg(gyro, I3G_OUT_X_L);
  return (int16_t)data;
}

int16_t GYRO_Y(I3G4250D_t *gyro) {
  uint16_t data = 0;
  data = GYRO_read_reg(gyro, I3G_OUT_Y_H) << 8;
  data |= GYRO_read_reg(gyro, I3G_OUT_Y_L);
  return (int16_t)data;
}

int16_t GYRO_Z(I3G4250D_t *gyro) {
  uint16_t data = 0;
  data = GYRO_read_reg(gyro, I3G_OUT_Z_H) << 8;
  data |= GYRO_read_reg(gyro, I3G_OUT_Z_L);
  return (int16_t)data;
}

void GYRO_ang(I3G4250D_t *gyro, vang_t *ang) {
#if I3G_speed == I3G_245dsp
  float coeff = I3G_245sp_coef;
#elif I3G_speed == I3G_500dsp
  float coeff = I3G_500sp_coef;
#elif I3G_speed == I3G_2000dsp
  float coeff = I3G_2000sp_coef;
#endif
  ang->x = (float)GYRO_X(gyro) * coeff + I3G_X_bias;
  ang->y = (float)GYRO_Y(gyro) * coeff + I3G_Y_bias;
  ang->z = (float)GYRO_Z(gyro) * coeff + I3G_Z_bias;
}

void GYRO_set_dsp(I3G4250D_t *gyro, uint8_t dsp) {
  uint8_t reg4 = GYRO_read_reg(gyro, I3G_CTRL_REG4);
  reg4 &= 0xCF;
  reg4 |= dsp << 4;
  GYRO_write_reg(gyro, I3G_CTRL_REG4, reg4);
  return;
}