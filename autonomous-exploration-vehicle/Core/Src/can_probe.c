#include "can_probe.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "spi.h"
#include "usart.h"

#define MCP2515_CMD_RESET 0xC0U
#define MCP2515_CMD_READ  0x03U
#define MCP2515_CMD_WRITE 0x02U

#define MCP2515_REG_CANSTAT 0x0EU
#define MCP2515_REG_CANCTRL 0x0FU
#define MCP2515_REG_CNF3    0x28U
#define MCP2515_REG_CNF2    0x29U
#define MCP2515_REG_CNF1    0x2AU
#define MCP2515_REG_CANINTF 0x2CU
#define MCP2515_REG_EFLG    0x2DU
#define MCP2515_REG_RXB0CTRL 0x60U
#define MCP2515_REG_RXB0SIDH 0x61U
#define MCP2515_REG_RXB0SIDL 0x62U
#define MCP2515_REG_RXB0DLC  0x65U
#define MCP2515_REG_RXB0D0   0x66U

#define MCP2515_MODE_CONFIG 0x80U
#define MCP2515_MODE_NORMAL 0x00U

volatile uint32_t g_can_probe_magic = 0U;
volatile uint32_t g_can_probe_count = 0U;
volatile uint32_t g_can_probe_status = 0U;
volatile uint32_t g_can_probe_regs = 0U;
volatile uint32_t g_can_probe_cnf3 = 0U;
volatile uint32_t g_can_probe_tx_count = 0U;
volatile uint32_t g_can_probe_txctrl = 0U;
volatile uint32_t g_can_probe_rx_count = 0U;
volatile uint32_t g_can_probe_rx_id = 0U;
volatile uint32_t g_can_probe_rx_info = 0U;
volatile uint32_t g_can_probe_rx_data0 = 0U;
volatile uint32_t g_can_probe_bus_flags = 0U;

static void CanProbe_Log(const char *message);
static void CanProbe_Reset(void);
static uint8_t CanProbe_ReadRegister(uint8_t address);
static void CanProbe_WriteRegister(uint8_t address, uint8_t value);
static uint8_t CanProbe_SetMode(uint8_t mode);
static uint8_t CanProbe_InitNormal500K(void);
static void CanProbe_PollReceive(void);

void CanProbe_Run(void)
{
  static uint8_t initialized = 0U;
  uint8_t canstat;
  uint8_t canctrl;
  uint8_t cnf1;
  uint8_t cnf2;
  uint8_t cnf3;
  char line[128];

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  if (initialized == 0U)
  {
    CanProbe_Log("\r\n[MCP2515] probe start\r\n");
    initialized = CanProbe_InitNormal500K();
  }

  canstat = CanProbe_ReadRegister(MCP2515_REG_CANSTAT);
  canctrl = CanProbe_ReadRegister(MCP2515_REG_CANCTRL);
  cnf1 = CanProbe_ReadRegister(MCP2515_REG_CNF1);
  cnf2 = CanProbe_ReadRegister(MCP2515_REG_CNF2);
  cnf3 = CanProbe_ReadRegister(MCP2515_REG_CNF3);
  CanProbe_PollReceive();

  g_can_probe_magic = 0x43414E50U;
  g_can_probe_count++;
  g_can_probe_regs = (uint32_t)canstat |
                     ((uint32_t)canctrl << 8) |
                     ((uint32_t)cnf1 << 16) |
                     ((uint32_t)cnf2 << 24);
  g_can_probe_cnf3 = cnf3;

  snprintf(line, sizeof(line),
           "[MCP2515] CANSTAT=0x%02X CANCTRL=0x%02X CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X\r\n",
           canstat, canctrl, cnf1, cnf2, cnf3);
  CanProbe_Log(line);

  if (((canstat & 0xE0U) == MCP2515_MODE_NORMAL) && ((canctrl & 0xE0U) == MCP2515_MODE_NORMAL))
  {
    g_can_probe_status = 4U;
    CanProbe_Log("[MCP2515] normal mode, RX polling active\r\n");
  }
  else if (((canstat & 0xE0U) == MCP2515_MODE_CONFIG) && ((canctrl & 0xE0U) == MCP2515_MODE_CONFIG))
  {
    g_can_probe_status = 1U;
    CanProbe_Log("[MCP2515] detected over SPI, still in config mode\r\n");
  }
  else if ((canstat == 0x00U && canctrl == 0x00U) || (canstat == 0xFFU && canctrl == 0xFFU))
  {
    g_can_probe_status = 2U;
    CanProbe_Log("[MCP2515] not detected: check VCC/GND/SCK/MISO/MOSI/CS\r\n");
  }
  else
  {
    g_can_probe_status = 3U;
    CanProbe_Log("[MCP2515] SPI responds, but values are unexpected\r\n");
  }
}

static void CanProbe_Log(const char *message)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)strlen(message), HAL_MAX_DELAY);
}

static void CanProbe_Reset(void)
{
  uint8_t command = MCP2515_CMD_RESET;

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

static uint8_t CanProbe_ReadRegister(uint8_t address)
{
  uint8_t tx[3] = {MCP2515_CMD_READ, address, 0x00U};
  uint8_t rx[3] = {0U, 0U, 0U};

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  return rx[2];
}

static void CanProbe_WriteRegister(uint8_t address, uint8_t value)
{
  uint8_t tx[3] = {MCP2515_CMD_WRITE, address, value};

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

static uint8_t CanProbe_SetMode(uint8_t mode)
{
  uint8_t canstat;

  CanProbe_WriteRegister(MCP2515_REG_CANCTRL, mode);

  for (uint32_t i = 0U; i < 50U; i++)
  {
    canstat = CanProbe_ReadRegister(MCP2515_REG_CANSTAT);
    if ((canstat & 0xE0U) == mode)
    {
      return 1U;
    }
    HAL_Delay(1);
  }

  return 0U;
}

static uint8_t CanProbe_InitNormal500K(void)
{
  CanProbe_Reset();
  HAL_Delay(10);

  if (CanProbe_SetMode(MCP2515_MODE_CONFIG) == 0U)
  {
    g_can_probe_status = 10U;
    return 0U;
  }

  CanProbe_WriteRegister(MCP2515_REG_CNF1, 0x00U);
  CanProbe_WriteRegister(MCP2515_REG_CNF2, 0x91U);
  CanProbe_WriteRegister(MCP2515_REG_CNF3, 0x01U);
  CanProbe_WriteRegister(MCP2515_REG_RXB0CTRL, 0x60U);
  CanProbe_WriteRegister(MCP2515_REG_CANINTF, 0x00U);

  if (CanProbe_SetMode(MCP2515_MODE_NORMAL) == 0U)
  {
    g_can_probe_status = 11U;
    return 0U;
  }

  g_can_probe_status = 4U;
  return 1U;
}

static void CanProbe_PollReceive(void)
{
  uint8_t canintf = CanProbe_ReadRegister(MCP2515_REG_CANINTF);
  uint8_t eflg = CanProbe_ReadRegister(MCP2515_REG_EFLG);

  g_can_probe_bus_flags = (uint32_t)canintf | ((uint32_t)eflg << 8);

  if ((canintf & 0x01U) == 0U)
  {
    return;
  }

  uint8_t sidh = CanProbe_ReadRegister(MCP2515_REG_RXB0SIDH);
  uint8_t sidl = CanProbe_ReadRegister(MCP2515_REG_RXB0SIDL);
  uint8_t dlc = CanProbe_ReadRegister(MCP2515_REG_RXB0DLC) & 0x0FU;
  uint8_t data[4] = {0U, 0U, 0U, 0U};

  for (uint8_t i = 0U; i < 4U && i < dlc; i++)
  {
    data[i] = CanProbe_ReadRegister((uint8_t)(MCP2515_REG_RXB0D0 + i));
  }

  g_can_probe_rx_count++;
  g_can_probe_rx_id = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
  g_can_probe_rx_info = dlc;
  g_can_probe_rx_data0 = (uint32_t)data[0] |
                         ((uint32_t)data[1] << 8) |
                         ((uint32_t)data[2] << 16) |
                         ((uint32_t)data[3] << 24);

  CanProbe_WriteRegister(MCP2515_REG_CANINTF, (uint8_t)(canintf & ~0x01U));
}
