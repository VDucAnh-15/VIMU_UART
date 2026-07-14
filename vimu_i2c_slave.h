#ifndef VIMU_I2C_SLAVE_H
#define VIMU_I2C_SLAVE_H

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "vimu_fifo.h"

/* I2C address: MMA8451 with SA0=1 */
#define VIMU_I2C_DEFAULT_ADDRESS        0x1DU

/* IRQ pin - active-high output to ESP32 */
#define VIMU_I2C_IRQ_GPIO_CLK           RCC_APB2Periph_GPIOA
#define VIMU_I2C_IRQ_GPIO_PORT          GPIOA
#define VIMU_I2C_IRQ_GPIO_PIN           GPIO_Pin_8

/* WHO_AM_I value returned to identify as MMA8451 */
#define MMA8451_WHO_AM_I_VALUE          0x1AU

/* ── MMA8451 Register Map ───────────────────────────────────────────── */
#define MMA8451_REG_F_STATUS            0x00U
#define MMA8451_REG_OUT_X_MSB           0x01U
#define MMA8451_REG_OUT_X_LSB           0x02U
#define MMA8451_REG_OUT_Y_MSB           0x03U
#define MMA8451_REG_OUT_Y_LSB           0x04U
#define MMA8451_REG_OUT_Z_MSB           0x05U
#define MMA8451_REG_OUT_Z_LSB           0x06U
#define MMA8451_REG_F_SETUP             0x09U
#define MMA8451_REG_WHO_AM_I            0x0DU
#define MMA8451_REG_XYZ_DATA_CFG        0x0EU
#define MMA8451_REG_CTRL_REG1           0x2AU
#define MMA8451_REG_CTRL_REG2           0x2BU
#define MMA8451_REG_CTRL_REG3           0x2CU
#define MMA8451_REG_CTRL_REG4           0x2DU
#define MMA8451_REG_CTRL_REG5           0x2EU

/* ── Register bit definitions ───────────────────────────────────────── */

/* CTRL_REG1 */
#define MMA8451_CTRL1_ACTIVE            (1U << 0)
#define MMA8451_CTRL1_F_READ            (1U << 1)
#define MMA8451_CTRL1_DR_SHIFT          3U
#define MMA8451_CTRL1_DR_MASK           0x38U

/* F_SETUP */
#define MMA8451_F_MODE_MASK             0xC0U
#define MMA8451_F_MODE_DISABLED         0x00U
#define MMA8451_F_MODE_CIRCULAR         0x40U
#define MMA8451_F_MODE_FILL             0x80U
#define MMA8451_F_MODE_TRIGGER          0xC0U
#define MMA8451_F_WMRK_MASK             0x3FU

/* F_STATUS */
#define MMA8451_F_OVF                   (1U << 7)
#define MMA8451_F_WMRK_FLAG             (1U << 6)
#define MMA8451_F_CNT_MASK              0x3FU

/* CTRL_REG3 */
#define MMA8451_CTRL3_IPOL              (1U << 1)  /* 0=active-low, 1=active-high */
#define MMA8451_CTRL3_PP_OD             (1U << 0)  /* 0=push-pull,  1=open-drain  */

/* CTRL_REG4 */
#define MMA8451_INT_EN_FIFO             (1U << 6)

/* CTRL_REG5 */
#define MMA8451_INT_CFG_FIFO_INT1       (1U << 6)  /* 0=INT2, 1=INT1 */

/* I2C commands queued to main loop */
#define VIMU_I2C_CMD_START              0x01U
#define VIMU_I2C_CMD_STOP               0x02U
#define VIMU_I2C_CMD_CLEAR              0x03U

/* ── Driver API ─────────────────────────────────────────────────────── */
void    I2C_Slave_Init(uint8_t address);
void    I2C_Slave_SetIrq(uint8_t active);
void    I2C_Slave_Service(void);

/* Callbacks implemented in vimu_app.c */
void    I2C_Slave_OnAddressed(uint8_t isTransmitter);
void    I2C_Slave_OnRxByte(uint8_t data);
uint8_t I2C_Slave_OnTxByte(void);
void    I2C_Slave_OnStop(void);
void    I2C_Slave_OnError(void);

#endif /* VIMU_I2C_SLAVE_H */
