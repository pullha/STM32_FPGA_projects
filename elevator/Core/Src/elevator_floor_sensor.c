#include "elevator_floor_sensor.h"

#include "main.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} FloorSensorPin;

/* Board-friendly NUCLEO-F411RE floor sensor GPIO map.
 * Car A sensors are on the left CN7 Morpho header.
 * Car B sensors are on the right CN10 Morpho header.
 */
static const FloorSensorPin FloorSensorPins[ELEVATOR_CAR_COUNT][ELEVATOR_FLOOR_MAX] =
{
  {
    { PI_A_F1_GPIO_Port, PI_A_F1_Pin },
    { PI_A_F2_GPIO_Port, PI_A_F2_Pin },
    { PI_A_F3_GPIO_Port, PI_A_F3_Pin },
    { PI_A_F4_GPIO_Port, PI_A_F4_Pin },
    { PI_A_F5_GPIO_Port, PI_A_F5_Pin }
  },
  {
    { PI_B_F1_GPIO_Port, PI_B_F1_Pin },
    { PI_B_F2_GPIO_Port, PI_B_F2_Pin },
    { PI_B_F3_GPIO_Port, PI_B_F3_Pin },
    { PI_B_F4_GPIO_Port, PI_B_F4_Pin },
    { PI_B_F5_GPIO_Port, PI_B_F5_Pin }
  }
};

/* EXTI can select only one GPIO port for each line number.
 * With the current pin map, PC8/PB8 and PC9/PB9 conflict, so PB8 and PB9
 * remain polled inputs until those sensors are moved to unique EXTI lines.
 */
static const bool FloorSensorExtiEnabled[ELEVATOR_CAR_COUNT][ELEVATOR_FLOOR_MAX] =
{
  { true, true, true, true, true },
  { true, true, false, true, false }
};

static volatile uint16_t FloorSensorInterruptMask[ELEVATOR_CAR_COUNT] = { 0U, 0U };
static volatile uint32_t FloorSensorInterruptCount[ELEVATOR_CAR_COUNT] = { 0U, 0U };

/* Guard helper so invalid enum values do not index the sensor table. */
static bool CarIsValid(ElevatorCarId car)
{
  return (car == ELEVATOR_CAR_A) || (car == ELEVATOR_CAR_B);
}

/* Convert a 1-based floor number into the 0-based table index. */
static uint8_t FloorToIndex(uint8_t floor)
{
  return (uint8_t)(floor - ELEVATOR_FLOOR_MIN);
}

static uint16_t FloorToMask(uint8_t floor)
{
  return (uint16_t)(1U << FloorToIndex(floor));
}

/* Configure one group of pins as digital inputs.
 * Measured photo interrupter outputs are active-high:
 * clear beam = LOW, blocked beam = HIGH.
 */
static void InitPortPins(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = pins;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/* Configure one group of pins as edge-triggered EXTI inputs. */
static void InitExtiPins(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = pins;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void EnableExtiIrqs(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(PI_A_F4_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(PI_B_F4_Pin | PI_A_F5_Pin | PI_B_F2_Pin | PI_B_F1_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(PI_A_F1_Pin | PI_A_F2_Pin | PI_A_F3_Pin);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Configure every floor sensor input. */
void ElevatorFloorSensor_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  InitExtiPins(GPIOB, PI_A_F5_Pin);
  InitPortPins(GPIOB, PI_B_F3_Pin | PI_B_F5_Pin);
  InitExtiPins(GPIOC, PI_A_F1_Pin | PI_A_F2_Pin | PI_A_F3_Pin |
                      PI_B_F1_Pin | PI_B_F2_Pin | PI_B_F4_Pin);
  InitExtiPins(GPIOD, PI_A_F4_Pin);

  EnableExtiIrqs();
}

void ElevatorFloorSensor_HandleExti(uint16_t gpio_pin)
{
  uint8_t car_index;
  uint8_t floor;

  for (car_index = 0U; car_index < ELEVATOR_CAR_COUNT; car_index++)
  {
    for (floor = ELEVATOR_FLOOR_MIN; floor <= ELEVATOR_FLOOR_MAX; floor++)
    {
      uint8_t floor_index = FloorToIndex(floor);

      if (FloorSensorExtiEnabled[car_index][floor_index] &&
          (FloorSensorPins[car_index][floor_index].pin == gpio_pin))
      {
        FloorSensorInterruptMask[car_index] |= FloorToMask(floor);
        FloorSensorInterruptCount[car_index]++;
      }
    }
  }
}

/* Read the raw active-high photo interrupter state. */
bool ElevatorFloorSensor_IsBlocked(ElevatorCarId car, uint8_t floor)
{
  const FloorSensorPin *sensor_pin;

  if (!CarIsValid(car) || !ElevatorFloorIsValid(floor))
  {
    return false;
  }

  sensor_pin = &FloorSensorPins[car][FloorToIndex(floor)];

  return HAL_GPIO_ReadPin(sensor_pin->port, sensor_pin->pin) == GPIO_PIN_SET;
}

/* Debounced arrival check.
 * The sensor must still be blocked after a short delay before the floor is accepted.
 */
bool ElevatorFloorSensor_IsArrived(ElevatorCarId car, uint8_t floor)
{
  if (!ElevatorFloorSensor_IsBlocked(car, floor))
  {
    return false;
  }

  HAL_Delay(ELEVATOR_FLOOR_SENSOR_DEBOUNCE_MS);

  return ElevatorFloorSensor_IsBlocked(car, floor);
}

/* Return all currently blocked floor sensors for one car. */
uint16_t ElevatorFloorSensor_ReadMask(ElevatorCarId car)
{
  uint16_t mask = 0U;
  uint8_t floor;

  if (!CarIsValid(car))
  {
    return 0U;
  }

  for (floor = ELEVATOR_FLOOR_MIN; floor <= ELEVATOR_FLOOR_MAX; floor++)
  {
    if (ElevatorFloorSensor_IsBlocked(car, floor))
    {
      mask |= (uint16_t)(1U << FloorToIndex(floor));
    }
  }

  return mask;
}

/* Report a single active floor only.
 * Returning 0 for multiple active bits makes wiring or alignment problems easy to notice.
 */
uint8_t ElevatorFloorSensor_GetActiveFloor(ElevatorCarId car)
{
  uint16_t mask = ElevatorFloorSensor_ReadMask(car);
  uint8_t floor;
  uint8_t active_floor = 0U;
  uint8_t active_count = 0U;

  for (floor = ELEVATOR_FLOOR_MIN; floor <= ELEVATOR_FLOOR_MAX; floor++)
  {
    if ((mask & (uint16_t)(1U << FloorToIndex(floor))) != 0U)
    {
      active_floor = floor;
      active_count++;
    }
  }

  return (active_count == 1U) ? active_floor : 0U;
}

uint16_t ElevatorFloorSensor_GetInterruptMask(ElevatorCarId car)
{
  if (!CarIsValid(car))
  {
    return 0U;
  }

  return FloorSensorInterruptMask[car];
}

uint32_t ElevatorFloorSensor_GetInterruptCount(ElevatorCarId car)
{
  if (!CarIsValid(car))
  {
    return 0U;
  }

  return FloorSensorInterruptCount[car];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  ElevatorFloorSensor_HandleExti(GPIO_Pin);
}
