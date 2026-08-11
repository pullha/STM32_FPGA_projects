/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "algorithm_a.h"
#include "algorithm_b.h"
#include "algorithm_c.h"
#include "algorithm_d.h"
#include <stdio.h>
#include <string.h>

#include "elevator_floor_sensor.h"
#include "elevator_led.h"
#include "elevator_motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART command line length, including the final null terminator. */
#define UART_COMMAND_BUFFER_SIZE  64U

/* Small helper for static scenario arrays. The project arrays are all below 255 items. */
#define ARRAY_COUNT(array)        ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/* Temporary sensor-based auto-run values.
 * Max steps are only a safety limit; normal stopping is done by photo interrupters.
 */
#define SENSOR_RUN_MAX_STEPS_PER_FLOOR  25000U
#define SENSOR_HOME_MAX_STEPS_TO_F1     30000U
#define SENSOR_RUN_STEP_DELAY_US        1000U
#define SENSOR_RUN_STOP_DWELL_MS        1000U
#define SENSOR_ROUND_TRIP_DWELL_MS      10000U
#define SENSOR_ROUND_TRIP_MAX_STEPS     (SENSOR_RUN_MAX_STEPS_PER_FLOOR * 5U)
#define MOTOR_AB_TRAVEL_TEST_STEPS      61440U
#define AUTO_AB_B_START_DELAY_MS        30000U
#define MOTOR_B_F5_CLEAR_STEPS          10000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Shared UART buffers used for command input and formatted status output. */
static char uart_tx_buffer[160];
static char uart_rx_buffer[UART_COMMAND_BUFFER_SIZE];
static uint8_t uart_rx_index = 0U;

/* Scenario selection changed by "set ab" and "set cd" UART commands. */
static uint8_t selected_ab_scenario = 1U;
static uint8_t selected_cd_scenario = 1U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Send a null-terminated string over USART2. */
static void UartPrint(const char *message)
{
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)strlen(message), 100U);
}

/* Initial car positions for AB comparison.
 * A starts idle on 1F, and B starts idle on 4F.
 */
static const ElevatorState AbInitialElevators[ELEVATOR_CAR_COUNT] =
{
  { ELEVATOR_CAR_A, 1U, ELEVATOR_DIR_IDLE, 0U, { 0U } },
  { ELEVATOR_CAR_B, 4U, ELEVATOR_DIR_IDLE, 0U, { 0U } }
};

/* AB scenario 1 compares fixed odd/even zones against fixed low/high zones. */
static const ElevatorRequest AbScenario1[] =
{
  { 1U, ELEVATOR_DIR_UP, 3U, ELEVATOR_CAR_A },
  { 3U, ELEVATOR_DIR_UP, 5U, ELEVATOR_CAR_NONE },
  { 3U, ELEVATOR_DIR_DOWN, 1U, ELEVATOR_CAR_NONE }
};

/* Initial car positions for CD scenario 1.
 * A is already moving upward and has 3F and 5F reserved.
 * B is idle on 5F with no reserved stops.
 */
static const ElevatorState CdInitialElevators[ELEVATOR_CAR_COUNT] =
{
  { ELEVATOR_CAR_A, 2U, ELEVATOR_DIR_UP, 2U, { 3U, 5U } },
  { ELEVATOR_CAR_B, 5U, ELEVATOR_DIR_IDLE, 0U, { 0U } }
};

/* Initial car positions for CD scenario 2.
 * This setup is meant to show a case where algorithm D can prefer B because
 * A has several reserved stops and therefore a worse ETA.
 */
static const ElevatorState Cd2InitialElevators[ELEVATOR_CAR_COUNT] =
{
  { ELEVATOR_CAR_A, 1U, ELEVATOR_DIR_UP, 3U, { 2U, 3U, 5U } },
  { ELEVATOR_CAR_B, 5U, ELEVATOR_DIR_IDLE, 0U, { 0U } }
};

/* CD scenario 1: compare direction-priority dispatch with ETA dispatch. */
static const ElevatorRequest CdScenario1[] =
{
  { 4U, ELEVATOR_DIR_UP, 5U, ELEVATOR_CAR_NONE },
  { 1U, ELEVATOR_DIR_UP, 3U, ELEVATOR_CAR_NONE }
};

/* CD scenario 2: one 4F UP request used to make C and D choose differently. */
static const ElevatorRequest CdScenario2[] =
{
  { 4U, ELEVATOR_DIR_UP, 5U, ELEVATOR_CAR_NONE }
};

/* ETA timing used by algorithm D: 5 seconds per floor and 30 seconds dwell. */
static const ElevatorTiming CdTiming1 =
{
  5U,
  30U
};

/* Convert car enum values into short strings for UART output. */
static const char *CarName(ElevatorCarId car)
{
  if (car == ELEVATOR_CAR_A)
  {
    return "A";
  }

  if (car == ELEVATOR_CAR_B)
  {
    return "B";
  }

  return "NONE";
}

/* Convert direction enum values into short strings for UART output. */
static const char *DirectionName(ElevatorDirection direction)
{
  if (direction == ELEVATOR_DIR_UP)
  {
    return "UP";
  }

  if (direction == ELEVATOR_DIR_DOWN)
  {
    return "DOWN";
  }

  return "IDLE";
}

/* Copy a scenario's initial elevator state before running an algorithm.
 * This keeps repeated UART runs from modifying the original scenario data.
 */
static void CopyElevators(ElevatorState destination[ELEVATOR_CAR_COUNT],
                          const ElevatorState source[ELEVATOR_CAR_COUNT])
{
  uint8_t index;

  for (index = 0U; index < ELEVATOR_CAR_COUNT; index++)
  {
    destination[index] = source[index];
  }
}

/* Print every supported UART command. */
static void PrintHelp(void)
{
  UartPrint("\r\nCommands:\r\n");
  UartPrint("  run ab a  - run selected AB scenario with algorithm A\r\n");
  UartPrint("  run ab b  - run selected AB scenario with algorithm B\r\n");
  UartPrint("  run cd c  - run selected CD scenario with algorithm C\r\n");
  UartPrint("  run cd d  - run selected CD scenario with algorithm D\r\n");
  UartPrint("  set ab 1  - select AB scenario 1\r\n");
  UartPrint("  set cd 1  - select CD scenario 1\r\n");
  UartPrint("  set cd 2  - select CD scenario 2\r\n");
  UartPrint("  status    - print selected scenarios and floor sensors\r\n");
  UartPrint("  help      - print this command list\r\n\r\n");
}

/* Read photo interrupters and show the currently detected floor with LEDs.
 * A car LED is turned on only when exactly one floor sensor is active.
 */
static void UpdateFloorArrivalLeds(void)
{
  uint8_t floor_a = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_A);
  uint8_t floor_b = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_B);

  if (floor_a != 0U)
  {
    (void)ElevatorLed_ShowFloor(ELEVATOR_CAR_A, floor_a);
  }
  else
  {
    ElevatorLed_CarOff(ELEVATOR_CAR_A);
  }

  if (floor_b != 0U)
  {
    (void)ElevatorLed_ShowFloor(ELEVATOR_CAR_B, floor_b);
  }
  else
  {
    ElevatorLed_CarOff(ELEVATOR_CAR_B);
  }
}

/* Print raw sensor masks and decoded active floor for both cars. */
static void PrintFloorSensorState(void)
{
  uint16_t mask_a = ElevatorFloorSensor_ReadMask(ELEVATOR_CAR_A);
  uint16_t mask_b = ElevatorFloorSensor_ReadMask(ELEVATOR_CAR_B);
  uint16_t irq_mask_a = ElevatorFloorSensor_GetInterruptMask(ELEVATOR_CAR_A);
  uint16_t irq_mask_b = ElevatorFloorSensor_GetInterruptMask(ELEVATOR_CAR_B);
  uint32_t irq_count_a = ElevatorFloorSensor_GetInterruptCount(ELEVATOR_CAR_A);
  uint32_t irq_count_b = ElevatorFloorSensor_GetInterruptCount(ELEVATOR_CAR_B);
  uint8_t floor_a = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_A);
  uint8_t floor_b = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_B);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "PI A:F%u mask=0x%02X irq=0x%02X cnt=%lu B:F%u mask=0x%02X irq=0x%02X cnt=%lu\r\n",
                 (unsigned int)floor_a,
                 (unsigned int)mask_a,
                 (unsigned int)irq_mask_a,
                 (unsigned long)irq_count_a,
                 (unsigned int)floor_b,
                 (unsigned int)mask_b,
                 (unsigned int)irq_mask_b,
                 (unsigned long)irq_count_b);
  UartPrint(uart_tx_buffer);
}

/* Print the currently selected scenario numbers and live floor sensor state. */
static void PrintStatus(void)
{
  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Selected scenarios: AB%u CD%u\r\n",
                 (unsigned int)selected_ab_scenario,
                 (unsigned int)selected_cd_scenario);
  UartPrint(uart_tx_buffer);
  PrintFloorSensorState();
}

/* Print one request assignment line after an algorithm finishes. */
static void PrintAssignmentLine(const char *group,
                                uint8_t scenario_number,
                                char algorithm,
                                uint8_t request_index,
                                const ElevatorRequest *request,
                                const ElevatorAssignment *assignment)
{
  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "%s%u alg %c req%u F%u->F%u %s reqCar=%s => %s %s %s\r\n",
                 group,
                 (unsigned int)scenario_number,
                 algorithm,
                 (unsigned int)(request_index + 1U),
                 (unsigned int)request->from_floor,
                 (unsigned int)request->target_floor,
                 DirectionName(request->direction),
                 CarName(request->requested_car),
                 assignment->accepted ? "OK" : "REJECT",
                 CarName(assignment->assigned_car),
                 assignment->reason);
  UartPrint(uart_tx_buffer);
}

/* Run the selected AB scenario with algorithm A or B.
 * This currently compares dispatch decisions over UART; motor movement is not
 * started from this function yet.
 */
static void RunAbScenario(char algorithm)
{
  ElevatorState elevators[ELEVATOR_CAR_COUNT];
  ElevatorAssignment assignments[ARRAY_COUNT(AbScenario1)];
  uint8_t index;

  if (selected_ab_scenario != 1U)
  {
    UartPrint("Only AB scenario 1 is available.\r\n");
    return;
  }

  CopyElevators(elevators, AbInitialElevators);

  if (algorithm == 'a')
  {
    ElevatorAlgorithmA_AssignBatch(elevators, AbScenario1, ARRAY_COUNT(AbScenario1), assignments);
  }
  else if (algorithm == 'b')
  {
    ElevatorAlgorithmB_AssignBatch(elevators, AbScenario1, ARRAY_COUNT(AbScenario1), assignments);
  }
  else
  {
    UartPrint("Use: run ab a or run ab b\r\n");
    return;
  }

  UartPrint("Running AB scenario...\r\n");

  for (index = 0U; index < ARRAY_COUNT(AbScenario1); index++)
  {
    PrintAssignmentLine("AB", selected_ab_scenario, algorithm, index, &AbScenario1[index], &assignments[index]);
  }
}

/* Run the selected CD scenario with algorithm C or D.
 * Scenario 1 and 2 share the same UART commands but have different initial
 * elevator states and request lists.
 */
static void RunCdScenario(char algorithm)
{
  ElevatorState elevators[ELEVATOR_CAR_COUNT];
  ElevatorAssignment assignments[ELEVATOR_MAX_STOPS];
  const ElevatorRequest *requests = CdScenario1;
  uint8_t request_count = ARRAY_COUNT(CdScenario1);
  uint8_t index;

  if (selected_cd_scenario == 1U)
  {
    CopyElevators(elevators, CdInitialElevators);
    requests = CdScenario1;
    request_count = ARRAY_COUNT(CdScenario1);
  }
  else if (selected_cd_scenario == 2U)
  {
    CopyElevators(elevators, Cd2InitialElevators);
    requests = CdScenario2;
    request_count = ARRAY_COUNT(CdScenario2);
  }
  else
  {
    UartPrint("Use: set cd 1 or set cd 2\r\n");
    return;
  }

  if (algorithm == 'c')
  {
    ElevatorAlgorithmC_AssignBatch(elevators, requests, request_count, assignments);
  }
  else if (algorithm == 'd')
  {
    ElevatorAlgorithmD_AssignBatch(elevators, requests, request_count, &CdTiming1, assignments);
  }
  else
  {
    UartPrint("Use: run cd c or run cd d\r\n");
    return;
  }

  UartPrint("Running CD scenario...\r\n");

  for (index = 0U; index < request_count; index++)
  {
    PrintAssignmentLine("CD", selected_cd_scenario, algorithm, index, &requests[index], &assignments[index]);
  }
}

/* Make command parsing case-insensitive for ASCII commands. */
static void ToLowerAscii(char *text)
{
  while (*text != '\0')
  {
    if ((*text >= 'A') && (*text <= 'Z'))
    {
      *text = (char)(*text + ('a' - 'A'));
    }

    text++;
  }
}

/* Split one command line into space/tab-separated tokens.
 * The input buffer is modified in place by replacing separators with '\0'.
 */
static uint8_t TokenizeCommand(char *command, char *tokens[], uint8_t max_tokens)
{
  uint8_t count = 0U;
  char *cursor = command;

  while ((*cursor != '\0') && (count < max_tokens))
  {
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    if (*cursor == '\0')
    {
      break;
    }

    tokens[count] = cursor;
    count++;

    while ((*cursor != '\0') && (*cursor != ' ') && (*cursor != '\t'))
    {
      cursor++;
    }

    if (*cursor != '\0')
    {
      *cursor = '\0';
      cursor++;
    }
  }

  return count;
}

/* Handle "set" commands that select which scenario number will be run next. */
static void ProcessSetCommand(char *tokens[], uint8_t token_count)
{
  if (token_count != 3U)
  {
    UartPrint("Use: set ab 1, set cd 1, or set cd 2\r\n");
    return;
  }

  if (strcmp(tokens[1], "ab") == 0)
  {
    if (strcmp(tokens[2], "1") != 0)
    {
      UartPrint("Only AB scenario 1 is available now.\r\n");
      return;
    }

    selected_ab_scenario = 1U;
    UartPrint("Selected AB scenario 1\r\n");
    return;
  }

  if (strcmp(tokens[1], "cd") == 0)
  {
    if (strcmp(tokens[2], "1") == 0)
    {
      selected_cd_scenario = 1U;
      UartPrint("Selected CD scenario 1\r\n");
      return;
    }

    if (strcmp(tokens[2], "2") == 0)
    {
      selected_cd_scenario = 2U;
      UartPrint("Selected CD scenario 2\r\n");
      return;
    }

    UartPrint("Use: set cd 1 or set cd 2\r\n");
    return;
  }

  UartPrint("Use: set ab 1, set cd 1, or set cd 2\r\n");
}

/* Handle "run" commands that choose a scenario group and algorithm letter. */
static void ProcessRunCommand(char *tokens[], uint8_t token_count)
{
  if (token_count != 3U)
  {
    UartPrint("Use: run ab a, run ab b, run cd c, or run cd d\r\n");
    return;
  }

  if (tokens[2][1] != '\0')
  {
    UartPrint("Algorithm must be one letter: a, b, c, or d\r\n");
    return;
  }

  if (strcmp(tokens[1], "ab") == 0)
  {
    RunAbScenario(tokens[2][0]);
    return;
  }

  if (strcmp(tokens[1], "cd") == 0)
  {
    RunCdScenario(tokens[2][0]);
    return;
  }

  UartPrint("Use scenario group ab or cd.\r\n");
}

/* Top-level UART command dispatcher. */
static void ProcessCommand(char *command)
{
  char *tokens[4];
  uint8_t token_count;

  ToLowerAscii(command);
  token_count = TokenizeCommand(command, tokens, ARRAY_COUNT(tokens));

  if (token_count == 0U)
  {
    return;
  }

  if (strcmp(tokens[0], "help") == 0)
  {
    PrintHelp();
    return;
  }

  if (strcmp(tokens[0], "status") == 0)
  {
    PrintStatus();
    return;
  }

  if (strcmp(tokens[0], "set") == 0)
  {
    ProcessSetCommand(tokens, token_count);
    return;
  }

  if (strcmp(tokens[0], "run") == 0)
  {
    ProcessRunCommand(tokens, token_count);
    return;
  }

  UartPrint("Unknown command. Type help.\r\n");
}

/* Poll USART2 without blocking the main loop.
 * A command is processed only after a line-feed '\n' is received.
 */
static void PollUartCommand(void)
{
  uint8_t rx_byte;

  while (HAL_UART_Receive(&huart2, &rx_byte, 1U, 0U) == HAL_OK)
  {
    if (rx_byte == '\r')
    {
      continue;
    }

    if (rx_byte == '\n')
    {
      uart_rx_buffer[uart_rx_index] = '\0';
      ProcessCommand(uart_rx_buffer);
      uart_rx_index = 0U;
      return;
    }

    if ((rx_byte == 0x08U) || (rx_byte == 0x7FU))
    {
      if (uart_rx_index > 0U)
      {
        uart_rx_index--;
      }

      continue;
    }

    if (uart_rx_index < (UART_COMMAND_BUFFER_SIZE - 1U))
    {
      uart_rx_buffer[uart_rx_index] = (char)rx_byte;
      uart_rx_index++;
    }
    else
    {
      uart_rx_index = 0U;
      UartPrint("Command too long. Type help.\r\n");
    }
  }
}

/* Move one car until the target floor photo interrupter is detected.
 * The max-step value is only a safety timeout in case wiring or alignment fails.
 */
static bool MoveCarToFloorBySensor(ElevatorCarId car, uint8_t *current_floor, uint8_t target_floor)
{
  ElevatorDirection direction;
  uint8_t floor_distance;
  uint32_t max_steps;
  ElevatorMotorStatus status;

  if ((current_floor == 0) ||
      !ElevatorFloorIsValid(*current_floor) ||
      !ElevatorFloorIsValid(target_floor))
  {
    return false;
  }

  if (*current_floor == target_floor)
  {
    (void)ElevatorLed_ShowFloor(car, *current_floor);
    HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);
    return true;
  }

  direction = ElevatorDirectionBetween(*current_floor, target_floor);
  floor_distance = ElevatorFloorDistance(*current_floor, target_floor);
  max_steps = (uint32_t)floor_distance * SENSOR_RUN_MAX_STEPS_PER_FLOOR;

  ElevatorLed_CarOff(car);
  status = ElevatorMotor_MoveUntilFloorSensor(car,
                                              direction,
                                              target_floor,
                                              max_steps,
                                              SENSOR_RUN_STEP_DELAY_US);

  if (status == ELEVATOR_MOTOR_STATUS_TARGET_REACHED)
  {
    *current_floor = target_floor;
    (void)ElevatorLed_ShowFloor(car, *current_floor);
    HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);
    return true;
  }

  ElevatorMotor_CarOff(car);
  ElevatorLed_CarOff(car);
  return false;
}

/* Move upward until the first-floor photo interrupter is reached.
 * This is used when the car starts below 1F, where no floor sensor is active yet.
 */
static bool HomeCarToFirstFloorBySensor(ElevatorCarId car)
{
  ElevatorMotorStatus status;

  if (ElevatorFloorSensor_IsArrived(car, 1U))
  {
    (void)ElevatorLed_ShowFloor(car, 1U);
    UartPrint("Home already at F1.\r\n");
    return true;
  }

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Home %s upward to F1...\r\n",
                 CarName(car));
  UartPrint(uart_tx_buffer);

  ElevatorLed_CarOff(car);
  status = ElevatorMotor_MoveUntilFloorSensor(car,
                                              ELEVATOR_DIR_UP,
                                              1U,
                                              SENSOR_HOME_MAX_STEPS_TO_F1,
                                              SENSOR_RUN_STEP_DELAY_US);

  if (status == ELEVATOR_MOTOR_STATUS_TARGET_REACHED)
  {
    (void)ElevatorLed_ShowFloor(car, 1U);
    UartPrint("Home reached F1.\r\n");
    PrintFloorSensorState();
    HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);
    return true;
  }

  ElevatorMotor_CarOff(car);
  ElevatorLed_CarOff(car);
  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Home failed status=%u.\r\n",
                 (unsigned int)status);
  UartPrint(uart_tx_buffer);
  PrintFloorSensorState();
  return false;
}

static void PrintCarReachedFloor(ElevatorCarId car, uint8_t floor)
{
  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "%s reached F%u.\r\n",
                 CarName(car),
                 (unsigned int)floor);
  UartPrint(uart_tx_buffer);
}

static bool MoveBothCarsUntilFloor(ElevatorDirection direction,
                                   uint8_t target_floor,
                                   uint32_t max_steps)
{
  bool reached[ELEVATOR_CAR_COUNT];
  uint32_t step_index;
  uint32_t next_report_step = 0U;

  reached[ELEVATOR_CAR_A] = ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_A, target_floor);
  reached[ELEVATOR_CAR_B] = ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_B, target_floor);

  if (reached[ELEVATOR_CAR_A])
  {
    PrintCarReachedFloor(ELEVATOR_CAR_A, target_floor);
  }

  if (reached[ELEVATOR_CAR_B])
  {
    PrintCarReachedFloor(ELEVATOR_CAR_B, target_floor);
  }

  for (step_index = 0U;
       (step_index < max_steps) && (!reached[ELEVATOR_CAR_A] || !reached[ELEVATOR_CAR_B]);
       step_index++)
  {
    if (!reached[ELEVATOR_CAR_A])
    {
      ElevatorMotor_Step(ELEVATOR_CAR_A, direction);
    }

    if (!reached[ELEVATOR_CAR_B])
    {
      ElevatorMotor_Step(ELEVATOR_CAR_B, direction);
    }

    HAL_Delay(SENSOR_RUN_STEP_DELAY_US / 1000U);

    if (!reached[ELEVATOR_CAR_A] &&
        ElevatorFloorSensor_IsBlocked(ELEVATOR_CAR_A, target_floor) &&
        ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_A, target_floor))
    {
      ElevatorMotor_CarOff(ELEVATOR_CAR_A);
      reached[ELEVATOR_CAR_A] = true;
      PrintCarReachedFloor(ELEVATOR_CAR_A, target_floor);
      PrintFloorSensorState();
    }

    if (!reached[ELEVATOR_CAR_B] &&
        ElevatorFloorSensor_IsBlocked(ELEVATOR_CAR_B, target_floor) &&
        ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_B, target_floor))
    {
      ElevatorMotor_CarOff(ELEVATOR_CAR_B);
      reached[ELEVATOR_CAR_B] = true;
      PrintCarReachedFloor(ELEVATOR_CAR_B, target_floor);
      PrintFloorSensorState();
    }

    if (step_index >= next_report_step)
    {
      next_report_step += 5000U;
      PrintFloorSensorState();
    }
  }

  ElevatorMotor_AllOff();

  if (!reached[ELEVATOR_CAR_A])
  {
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "A failed to reach F%u.\r\n",
                   (unsigned int)target_floor);
    UartPrint(uart_tx_buffer);
  }

  if (!reached[ELEVATOR_CAR_B])
  {
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "B failed to reach F%u.\r\n",
                   (unsigned int)target_floor);
    UartPrint(uart_tx_buffer);
  }

  PrintFloorSensorState();
  return reached[ELEVATOR_CAR_A] && reached[ELEVATOR_CAR_B];
}

static void AutoRunBothCarsFloorSensorRoundTrip(void)
{
  UartPrint("Auto AB floor sensor round-trip start.\r\n");
  UartPrint("Move A/B upward until F5 sensors.\r\n");

  if (!MoveBothCarsUntilFloor(ELEVATOR_DIR_UP, 5U, SENSOR_ROUND_TRIP_MAX_STEPS))
  {
    UartPrint("Round-trip aborted during upward move.\r\n");
    return;
  }

  UartPrint("Both cars reached F5. Dwell 10s.\r\n");
  HAL_Delay(SENSOR_ROUND_TRIP_DWELL_MS);
  UartPrint("Move A/B downward until F1 sensors.\r\n");

  if (!MoveBothCarsUntilFloor(ELEVATOR_DIR_DOWN, 1U, SENSOR_ROUND_TRIP_MAX_STEPS))
  {
    UartPrint("Round-trip aborted during downward move.\r\n");
    return;
  }

  UartPrint("Auto AB floor sensor round-trip done.\r\n");
}

/* Execute the stop list created by an algorithm for one elevator car. */
static void ExecuteCarStopsBySensor(ElevatorState *elevator)
{
  uint8_t active_floor;
  uint8_t stop_index;
  uint8_t current_floor;

  active_floor = ElevatorFloorSensor_GetActiveFloor(elevator->id);
  current_floor = (active_floor != 0U) ? active_floor : elevator->current_floor;

  if (ElevatorFloorIsValid(current_floor))
  {
    (void)ElevatorLed_ShowFloor(elevator->id, current_floor);
    HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);
  }

  for (stop_index = 0U; stop_index < elevator->stop_count; stop_index++)
  {
    if (!MoveCarToFloorBySensor(elevator->id, &current_floor, elevator->stops[stop_index]))
    {
      break;
    }
  }

  elevator->current_floor = current_floor;
  elevator->direction = ELEVATOR_DIR_IDLE;
}

/* UART-free AB scenario 1 test using algorithm A.
 * The algorithm produces the stop list, then the motor moves by floor sensors.
 */
static void AutoRunAb1AlgorithmABySensor(void)
{
  ElevatorState elevators[ELEVATOR_CAR_COUNT];
  ElevatorAssignment assignments[ARRAY_COUNT(AbScenario1)];

  UartPrint("Auto AB1 algorithm A start.\r\n");

  if (!HomeCarToFirstFloorBySensor(ELEVATOR_CAR_A))
  {
    UartPrint("Auto AB1 algorithm A aborted.\r\n");
    return;
  }

  CopyElevators(elevators, AbInitialElevators);
  elevators[ELEVATOR_CAR_A].current_floor = 1U;
  ElevatorAlgorithmA_AssignBatch(elevators, AbScenario1, ARRAY_COUNT(AbScenario1), assignments);

  UartPrint("Auto AB1 algorithm A run stops.\r\n");
  ExecuteCarStopsBySensor(&elevators[ELEVATOR_CAR_A]);
  ExecuteCarStopsBySensor(&elevators[ELEVATOR_CAR_B]);
  UartPrint("Auto AB1 algorithm A done.\r\n");
  PrintFloorSensorState();
}

/* UART-free AB scenario 1 test using algorithm B.
 * The system waits 30 seconds before running so the start state can be checked.
 */
static void AutoRunAb1AlgorithmBBySensor(void)
{
  ElevatorState elevators[ELEVATOR_CAR_COUNT];
  ElevatorAssignment assignments[ARRAY_COUNT(AbScenario1)];
  uint8_t index;
  uint8_t active_floor_a;
  uint8_t active_floor_b;

  selected_ab_scenario = 1U;
  UartPrint("Selected AB scenario 1.\r\n");
  PrintFloorSensorState();

  UartPrint("Wait 30 seconds before algorithm B.\r\n");
  HAL_Delay(AUTO_AB_B_START_DELAY_MS);

  CopyElevators(elevators, AbInitialElevators);
  active_floor_a = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_A);
  active_floor_b = ElevatorFloorSensor_GetActiveFloor(ELEVATOR_CAR_B);

  if (active_floor_a != 0U)
  {
    elevators[ELEVATOR_CAR_A].current_floor = active_floor_a;
  }

  if (active_floor_b != 0U)
  {
    elevators[ELEVATOR_CAR_B].current_floor = active_floor_b;
  }

  UartPrint("Auto AB1 algorithm B start.\r\n");
  ElevatorAlgorithmB_AssignBatch(elevators, AbScenario1, ARRAY_COUNT(AbScenario1), assignments);

  for (index = 0U; index < ARRAY_COUNT(AbScenario1); index++)
  {
    PrintAssignmentLine("AB", selected_ab_scenario, 'b', index, &AbScenario1[index], &assignments[index]);
  }

  ExecuteCarStopsBySensor(&elevators[ELEVATOR_CAR_A]);
  ExecuteCarStopsBySensor(&elevators[ELEVATOR_CAR_B]);
  UartPrint("Auto AB1 algorithm B done.\r\n");
  PrintFloorSensorState();
}

/* UART-free hardware test: move only car B upward until the 5F sensor is HIGH.
 * If B already starts on the 5F sensor, move down a little first so the rising
 * movement can prove that the 5F sensor stops the motor.
 */
static void AutoRunMotorBToF5SensorTest(void)
{
  ElevatorMotorStatus status;

  UartPrint("Motor B F5 sensor stop test start.\r\n");
  PrintFloorSensorState();

  if (ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_B, 5U))
  {
    UartPrint("B F5 is already active. Move B down first.\r\n");
    ElevatorMotor_MoveStepsUs(ELEVATOR_CAR_B,
                              ELEVATOR_DIR_DOWN,
                              MOTOR_B_F5_CLEAR_STEPS,
                              SENSOR_RUN_STEP_DELAY_US);
    ElevatorMotor_CarOff(ELEVATOR_CAR_B);
    PrintFloorSensorState();
    HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);
  }

  UartPrint("Move B upward until F5 sensor is active.\r\n");
  ElevatorLed_CarOff(ELEVATOR_CAR_B);
  status = ElevatorMotor_MoveUntilFloorSensor(ELEVATOR_CAR_B,
                                              ELEVATOR_DIR_UP,
                                              5U,
                                              SENSOR_ROUND_TRIP_MAX_STEPS,
                                              SENSOR_RUN_STEP_DELAY_US);

  if (status == ELEVATOR_MOTOR_STATUS_TARGET_REACHED)
  {
    (void)ElevatorLed_ShowFloor(ELEVATOR_CAR_B, 5U);
    UartPrint("Motor B stopped at F5 sensor.\r\n");
  }
  else
  {
    ElevatorLed_CarOff(ELEVATOR_CAR_B);
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "Motor B F5 test failed status=%u.\r\n",
                   (unsigned int)status);
    UartPrint(uart_tx_buffer);
  }

  ElevatorMotor_CarOff(ELEVATOR_CAR_B);
  PrintFloorSensorState();
}

/* UART-free hardware test: move only car B downward until the 1F sensor is HIGH.
 * During the descent, print every B sensor-mask change so F5/F4/F3/F2/F1 can be checked.
 */
static void AutoRunMotorBDownToF1TraceTest(void)
{
  uint32_t step_index;
  uint32_t next_report_step = 0U;
  uint16_t current_mask;
  uint16_t previous_mask;
  uint16_t seen_mask = 0U;
  bool reached_f1 = false;

  UartPrint("Motor B down-to-F1 sensor trace start.\r\n");
  PrintFloorSensorState();

  previous_mask = ElevatorFloorSensor_ReadMask(ELEVATOR_CAR_B);
  seen_mask |= previous_mask;

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "B initial mask=0x%02X\r\n",
                 (unsigned int)previous_mask);
  UartPrint(uart_tx_buffer);

  if (ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_B, 1U))
  {
    (void)ElevatorLed_ShowFloor(ELEVATOR_CAR_B, 1U);
    UartPrint("B already at F1 sensor.\r\n");
    PrintFloorSensorState();
    return;
  }

  ElevatorLed_CarOff(ELEVATOR_CAR_B);

  for (step_index = 0U; step_index < SENSOR_ROUND_TRIP_MAX_STEPS; step_index++)
  {
    ElevatorMotor_Step(ELEVATOR_CAR_B, ELEVATOR_DIR_DOWN);
    HAL_Delay(SENSOR_RUN_STEP_DELAY_US / 1000U);

    current_mask = ElevatorFloorSensor_ReadMask(ELEVATOR_CAR_B);
    seen_mask |= current_mask;

    if (current_mask != previous_mask)
    {
      (void)snprintf(uart_tx_buffer,
                     sizeof(uart_tx_buffer),
                     "B step=%lu mask=0x%02X\r\n",
                     (unsigned long)step_index,
                     (unsigned int)current_mask);
      UartPrint(uart_tx_buffer);
      previous_mask = current_mask;
    }

    if ((current_mask & 0x01U) != 0U)
    {
      if (ElevatorFloorSensor_IsArrived(ELEVATOR_CAR_B, 1U))
      {
        reached_f1 = true;
        break;
      }
    }

    if (step_index >= next_report_step)
    {
      next_report_step += 5000U;
      (void)snprintf(uart_tx_buffer,
                     sizeof(uart_tx_buffer),
                     "B progress step=%lu mask=0x%02X seen=0x%02X\r\n",
                     (unsigned long)step_index,
                     (unsigned int)current_mask,
                     (unsigned int)seen_mask);
      UartPrint(uart_tx_buffer);
    }
  }

  ElevatorMotor_CarOff(ELEVATOR_CAR_B);

  if (reached_f1)
  {
    (void)ElevatorLed_ShowFloor(ELEVATOR_CAR_B, 1U);
    UartPrint("Motor B stopped at F1 sensor.\r\n");
  }
  else
  {
    ElevatorLed_CarOff(ELEVATOR_CAR_B);
    UartPrint("Motor B did not reach F1 before max steps.\r\n");
  }

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "B seen mask during descent=0x%02X\r\n",
                 (unsigned int)seen_mask);
  UartPrint(uart_tx_buffer);
  PrintFloorSensorState();
}

/* Move one car in one direction while tracing every floor-sensor mask change.
 * The motor stops when the target floor sensor stays HIGH through debounce.
 */
static bool TraceCarUntilFloorSensor(ElevatorCarId car,
                                     ElevatorDirection direction,
                                     uint8_t target_floor,
                                     const char *phase_name)
{
  uint32_t step_index;
  uint32_t next_report_step = 0U;
  uint16_t current_mask;
  uint16_t previous_mask;
  uint16_t seen_mask = 0U;
  uint16_t target_mask = (uint16_t)(1U << (target_floor - 1U));
  bool reached_target = false;

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "%s %s to F%u trace start.\r\n",
                 CarName(car),
                 phase_name,
                 (unsigned int)target_floor);
  UartPrint(uart_tx_buffer);
  PrintFloorSensorState();

  previous_mask = ElevatorFloorSensor_ReadMask(car);
  seen_mask |= previous_mask;

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "%s initial mask=0x%02X\r\n",
                 CarName(car),
                 (unsigned int)previous_mask);
  UartPrint(uart_tx_buffer);

  if (ElevatorFloorSensor_IsArrived(car, target_floor))
  {
    (void)ElevatorLed_ShowFloor(car, target_floor);
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "%s already at F%u sensor.\r\n",
                   CarName(car),
                   (unsigned int)target_floor);
    UartPrint(uart_tx_buffer);
    PrintFloorSensorState();
    return true;
  }

  ElevatorLed_CarOff(car);

  for (step_index = 0U; step_index < SENSOR_ROUND_TRIP_MAX_STEPS; step_index++)
  {
    ElevatorMotor_Step(car, direction);
    HAL_Delay(SENSOR_RUN_STEP_DELAY_US / 1000U);

    current_mask = ElevatorFloorSensor_ReadMask(car);
    seen_mask |= current_mask;

    if (current_mask != previous_mask)
    {
      (void)snprintf(uart_tx_buffer,
                     sizeof(uart_tx_buffer),
                     "%s %s step=%lu mask=0x%02X\r\n",
                     CarName(car),
                     phase_name,
                     (unsigned long)step_index,
                     (unsigned int)current_mask);
      UartPrint(uart_tx_buffer);
      previous_mask = current_mask;
    }

    if ((current_mask & target_mask) != 0U)
    {
      if (ElevatorFloorSensor_IsArrived(car, target_floor))
      {
        reached_target = true;
        break;
      }
    }

    if (step_index >= next_report_step)
    {
      next_report_step += 5000U;
      (void)snprintf(uart_tx_buffer,
                     sizeof(uart_tx_buffer),
                     "%s %s progress step=%lu mask=0x%02X seen=0x%02X\r\n",
                     CarName(car),
                     phase_name,
                     (unsigned long)step_index,
                     (unsigned int)current_mask,
                     (unsigned int)seen_mask);
      UartPrint(uart_tx_buffer);
    }
  }

  ElevatorMotor_CarOff(car);

  if (reached_target)
  {
    (void)ElevatorLed_ShowFloor(car, target_floor);
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "%s stopped at F%u sensor.\r\n",
                   CarName(car),
                   (unsigned int)target_floor);
    UartPrint(uart_tx_buffer);
  }
  else
  {
    ElevatorLed_CarOff(car);
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "%s did not reach F%u before max steps.\r\n",
                   CarName(car),
                   (unsigned int)target_floor);
    UartPrint(uart_tx_buffer);
  }

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "%s %s seen mask=0x%02X\r\n",
                 CarName(car),
                 phase_name,
                 (unsigned int)seen_mask);
  UartPrint(uart_tx_buffer);
  PrintFloorSensorState();

  return reached_target;
}

/* UART-free hardware test: car A starts near 5F, descends to 1F while tracing
 * all sensors, then ascends back to 5F while tracing sensors again.
 */
static void AutoRunMotorADownUpTraceTest(void)
{
  UartPrint("Motor A down-up sensor trace start.\r\n");

  if (!TraceCarUntilFloorSensor(ELEVATOR_CAR_A, ELEVATOR_DIR_DOWN, 1U, "DOWN"))
  {
    UartPrint("Motor A down-up trace aborted during DOWN phase.\r\n");
    return;
  }

  HAL_Delay(SENSOR_RUN_STOP_DWELL_MS);

  if (!TraceCarUntilFloorSensor(ELEVATOR_CAR_A, ELEVATOR_DIR_UP, 5U, "UP"))
  {
    UartPrint("Motor A down-up trace aborted during UP phase.\r\n");
    return;
  }

  UartPrint("Motor A down-up sensor trace done.\r\n");
}

/* UART-free hardware test: car A starts near 1F and ascends to 5F while tracing
 * every floor-sensor mask change. The motor stops when the 5F sensor is HIGH.
 */
static void AutoRunMotorAUpToF5TraceTest(void)
{
  UartPrint("Motor A up-to-F5 sensor trace start.\r\n");

  if (!TraceCarUntilFloorSensor(ELEVATOR_CAR_A, ELEVATOR_DIR_UP, 5U, "UP"))
  {
    UartPrint("Motor A up-to-F5 trace failed before F5.\r\n");
    return;
  }

  UartPrint("Motor A up-to-F5 sensor trace done.\r\n");
}

/* UART-free hardware test: car A starts above/near 5F and descends to 1F while
 * tracing every floor-sensor mask change. The motor stops when the 1F sensor is HIGH.
 */
static void AutoRunMotorADownToF1TraceTest(void)
{
  UartPrint("Motor A down-to-F1 sensor trace start.\r\n");

  if (!TraceCarUntilFloorSensor(ELEVATOR_CAR_A, ELEVATOR_DIR_DOWN, 1U, "DOWN"))
  {
    UartPrint("Motor A down-to-F1 trace failed before F1.\r\n");
    return;
  }

  UartPrint("Motor A down-to-F1 sensor trace done.\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Project hardware startup.
   * - Photo interrupter inputs are initialized by ElevatorFloorSensor_Init().
   * - LEDs start off and then follow active floor sensors in the main loop.
   * - UART starts in command mode so scenarios can be selected without editing code.
   */
  ElevatorFloorSensor_Init();
  ElevatorLed_AllOff();
  UartPrint("Elevator UART command mode ready.\r\n");
  PrintHelp();

  /* Temporary UART-free motor A downward sensor trace test.
   * Disable this call again when returning to command-based scenario control.
   */
  AutoRunMotorADownToF1TraceTest();
  if (0)
  {
    UpdateFloorArrivalLeds();
    AutoRunBothCarsFloorSensorRoundTrip();
    AutoRunAb1AlgorithmABySensor();
  }

  /* Manual motor test block.
   * Keep disabled during normal scenario comparison.
   * Example: move motor B upward by a fixed number of half-steps, then release coils.
   */
  /*
  ElevatorMotor_MoveStepsUs(ELEVATOR_CAR_B, ELEVATOR_DIR_UP, 40960U, 1000U);
  ElevatorMotor_CarOff(ELEVATOR_CAR_B);
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Motor-only test mode: do not read floor sensors automatically. */
    PollUartCommand();

    HAL_Delay(10U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
