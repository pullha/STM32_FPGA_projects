#include "can_probe.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "moter.h"
#include <stdlib.h>

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
#define MCP2515_REG_RXB1SIDH 0x71U
#define MCP2515_REG_RXB1SIDL 0x72U
#define MCP2515_REG_RXB1DLC  0x75U
#define MCP2515_REG_RXB1D0   0x76U

#define MCP2515_MODE_CONFIG 0x80U
#define MCP2515_MODE_NORMAL 0x00U
#define CAN_PROBE_SPI_TIMEOUT_MS 10U

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

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
  g_can_probe_count++;

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
  g_can_probe_regs = (uint32_t)canstat |
                     ((uint32_t)canctrl << 8) |
                     ((uint32_t)cnf1 << 16) |
                     ((uint32_t)cnf2 << 24);
  g_can_probe_cnf3 = cnf3;

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
    CanProbe_Log("[MCP2515] `SPI responds, but values are unexpected\r\n");
  }
}

uint8_t CanProbe_GetLastCommand(uint32_t *rx_count,
                                uint32_t *rx_id,
                                uint8_t *dlc,
                                uint8_t data[4])
{
  uint32_t packed_data;

  if ((rx_count == NULL) || (rx_id == NULL) || (dlc == NULL) || (data == NULL))
  {
    return 0U;
  }

  *rx_count = g_can_probe_rx_count;
  *rx_id = g_can_probe_rx_id;
  *dlc = (uint8_t)(g_can_probe_rx_info & 0x0FU);

  packed_data = g_can_probe_rx_data0;
  data[0] = (uint8_t)(packed_data & 0xFFU);
  data[1] = (uint8_t)((packed_data >> 8) & 0xFFU);
  data[2] = (uint8_t)((packed_data >> 16) & 0xFFU);
  data[3] = (uint8_t)((packed_data >> 24) & 0xFFU);

  return 1U;
}

static void CanProbe_Log(const char *message)
{
  (void)message;
}

static void CanProbe_Reset(void)
{
  uint8_t command = MCP2515_CMD_RESET;

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &command, 1, CAN_PROBE_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

static uint8_t CanProbe_ReadRegister(uint8_t address)
{
  uint8_t tx[3] = {MCP2515_CMD_READ, address, 0x00U};
  uint8_t rx[3] = {0U, 0U, 0U};

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, CAN_PROBE_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  return rx[2];
}

static void CanProbe_WriteRegister(uint8_t address, uint8_t value)
{
  uint8_t tx[3] = {MCP2515_CMD_WRITE, address, value};

  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 3, CAN_PROBE_SPI_TIMEOUT_MS);
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
    uint8_t canintf;
    uint8_t eflg;
    uint8_t sidh;
    uint8_t sidl;
    uint8_t dlc;
    uint8_t data[8] = {0U};
    uint8_t buffer_flag;
    uint8_t id_address;
    uint8_t dlc_address;
    uint8_t data_address;

    canintf = CanProbe_ReadRegister(MCP2515_REG_CANINTF);
    eflg = CanProbe_ReadRegister(MCP2515_REG_EFLG);
    g_can_probe_bus_flags = (uint32_t)canintf | ((uint32_t)eflg << 8);

    if ((canintf & 0x01U) != 0U)
    {
        buffer_flag = 0x01U;
        id_address = MCP2515_REG_RXB0SIDH;
        dlc_address = MCP2515_REG_RXB0DLC;
        data_address = MCP2515_REG_RXB0D0;
    }
    else if ((canintf & 0x02U) != 0U)
    {
        buffer_flag = 0x02U;
        id_address = MCP2515_REG_RXB1SIDH;
        dlc_address = MCP2515_REG_RXB1DLC;
        data_address = MCP2515_REG_RXB1D0;
    }
    else
    {
        return;
    }

    sidh = CanProbe_ReadRegister(id_address);
    sidl = CanProbe_ReadRegister((uint8_t)(id_address + 1U));
    dlc = CanProbe_ReadRegister(dlc_address) & 0x0FU;
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    // CAN 수신 여부 확인


    // RXB0 데이터 읽기
    for (uint8_t i = 0; i < dlc; i++)
    {
        data[i] = CanProbe_ReadRegister(
            (uint8_t)(data_address + i));
    }

    // 수신 정보 저장
    g_can_probe_rx_count++;
    g_can_probe_rx_id = ((uint32_t)sidh << 3) |
                        ((uint32_t)sidl >> 5);
    g_can_probe_rx_info = dlc;
    g_can_probe_rx_data0 = (uint32_t)data[0];

    // 여기에 추가
    if ((g_can_probe_rx_id == 0x210U) && (dlc >= 4U))
    {
        int8_t left = (int8_t)data[0];
    int8_t right = (int8_t)data[1];
    uint16_t duration_ms = (uint16_t)data[2] |
                           ((uint16_t)data[3] << 8);

    if ((left == 0) && (right == 0))
    {
        Moter_Stop();
    }
    else if (((left >= 0) && (right >= 0)) ||
             ((left <= 0) && (right <= 0)))
    {
        /* Preserve the two wheel values so straight PID can use their trim. */
        Moter_StraightCommandStart((int16_t)left,
                                   (int16_t)right,
                                   duration_ms);
    }
    else
    {
        Moter_DirectCommandStart(left, right, duration_ms);
    }
    }

    // 마지막에 수신 플래그 삭제
    CanProbe_WriteRegister(
        MCP2515_REG_CANINTF,
        (uint8_t)(canintf & (uint8_t)~buffer_flag));
}
