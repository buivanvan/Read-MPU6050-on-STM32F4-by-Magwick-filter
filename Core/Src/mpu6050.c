#include "stm32f407xx.h"
#include "mpu6050.h"

/* Reset clock tree: HSI = HCLK = PCLK1 = 16 MHz. */
#define MPU_ADDRESS 0x68u
#define I2C_ERRORS (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)
#define TIMEOUT_US 5000u

void timer_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->PSC = 15;
    TIM2->ARR = 0xffffffffu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t micros(void) { return TIM2->CNT; }

void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while (micros() - start < us) {}
}

static void i2c_init(void)
{
    const uint32_t pins = (1u << 6) | (1u << 7);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;
    GPIOB->OTYPER |= pins;
    GPIOB->AFR[0] = (GPIOB->AFR[0] & 0x00ffffffu) | 0x44000000u;
    GPIOB->OSPEEDR |= 0xf000u;
    GPIOB->PUPDR &= ~0xf000u;
    GPIOB->MODER = (GPIOB->MODER & ~0xf000u) | 0xa000u;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;     /* Standard mode, 100 kHz. */
    I2C1->TRISE = 17;
    I2C1->OAR1 = 1u << 14;
    I2C1->CR1 = I2C_CR1_PE;
}

static bool wait_flag(uint32_t mask)
{
    uint32_t start = micros();
    for (;;) {
        uint32_t status = I2C1->SR1;
        if ((status & I2C_ERRORS) || micros() - start >= TIMEOUT_US) return false;
        if (status & mask) return true;
    }
}

static bool address(bool read)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_flag(I2C_SR1_SB)) return false;
    I2C1->DR = (MPU_ADDRESS << 1) | (read ? 1u : 0u);
    return wait_flag(I2C_SR1_ADDR);
}

static void clear_addr(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static bool select_register(uint8_t reg)
{
    uint32_t start = micros();
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (micros() - start >= TIMEOUT_US) return false;
    }
    if (!address(false)) return false;
    clear_addr();
    I2C1->DR = reg;
    return wait_flag(I2C_SR1_BTF);
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    if (!select_register(reg)) return false;
    I2C1->DR = value;
    if (!wait_flag(I2C_SR1_BTF)) return false;
    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

/* STM32F4 master receiver, used for single registers and a 14-byte burst. */
static bool read_reg(uint8_t reg, uint8_t *dst, unsigned count)
{
    if (!select_register(reg)) return false;
    I2C1->CR1 |= I2C_CR1_ACK;
    if (!address(true)) return false;
    if (count == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!wait_flag(I2C_SR1_RXNE)) return false;
        *dst = (uint8_t)I2C1->DR;
        return true;
    }
    clear_addr();
    while (count > 3) {
        if (!wait_flag(I2C_SR1_RXNE)) return false;
        *dst++ = (uint8_t)I2C1->DR;
        --count;
    }
    if (!wait_flag(I2C_SR1_BTF)) return false;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    *dst++ = (uint8_t)I2C1->DR;
    if (!wait_flag(I2C_SR1_BTF)) return false;
    I2C1->CR1 |= I2C_CR1_STOP;
    *dst++ = (uint8_t)I2C1->DR;
    *dst = (uint8_t)I2C1->DR;
    return true;
}

bool mpu6050_init(void)
{
    uint8_t id;
    i2c_init();
    if (!read_reg(0x75, &id, 1) || id != 0x68) return false;
    if (!write_reg(0x6b, 0x80)) return false; /* DEVICE_RESET */
    delay_us(100000);
    if (!write_reg(0x6b, 1)) return false;    /* Wake, X gyro PLL */
    delay_us(100000);
    return write_reg(0x1a, 3) && write_reg(0x19, 9) && /* DLPF, 1 kHz / 10 */
           write_reg(0x1b, 0) && write_reg(0x1c, 0) && /* 250 dps, 2 g */
           write_reg(0x38, 1);                       /* DATA_RDY_EN */
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

bool mpu6050_read(ImuRaw *sample)
{
    uint8_t status, data[14];
    unsigned axis;
    uint32_t start = micros();
    do {
        if (!read_reg(0x3a, &status, 1)) return false;
        if (status & 1u) break;
        if (micros() - start >= 50000u) return false;
        delay_us(500);
    } while (true);
    if (!read_reg(0x3b, data, sizeof(data))) return false;
    for (axis = 0; axis < 3; ++axis) {
        sample->accel[axis] = be16(data + 2 * axis);
        sample->gyro[axis] = be16(data + 8 + 2 * axis);
    }
    return true;
}
