#ifndef __MOTER_H__
#define __MOTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

typedef enum
{
  MOTER_LEFT = 0,
  MOTER_RIGHT = 1
} MoterSide;

/* Command IDs are intended to match future CAN payload values from Raspberry Pi. */
typedef enum
{
  MOTER_COMMAND_STOP = 0,
  MOTER_COMMAND_FORWARD = 1,
  MOTER_COMMAND_BACKWARD = 2,
  MOTER_COMMAND_TURN_LEFT = 3,
  MOTER_COMMAND_TURN_RIGHT = 4,
  MOTER_COMMAND_DIRECT_WHEEL = 100
} MoterCommand;

/* Low-level motor control. Percent range is -100 to 100. */
void Moter_Init(void);
void Moter_Stop(void);
void Moter_SetSpeed(MoterSide side, int16_t percent);
void Moter_SetBothSpeed(int16_t left_percent, int16_t right_percent);

/* High-level movement helpers. Forward/backward use encoder PID correction. */
void Moter_MoveForward(int16_t percent);
void Moter_MoveBackward(int16_t percent);
void Moter_TurnLeft(int16_t percent);
void Moter_TurnRight(int16_t percent);
void Moter_MoveUpdate(void);

/* Timed command runner: call Start once, then Update periodically. */
void Moter_CommandStart(MoterCommand command, int16_t percent, uint32_t duration_ms);
void Moter_StraightCommandStart(int16_t left_percent, int16_t right_percent, uint32_t duration_ms);
void Moter_DirectCommandStart(int16_t left_percent, int16_t right_percent, uint32_t duration_ms);
void Moter_CommandUpdate(void);
uint8_t Moter_CommandIsRunning(void);
MoterCommand Moter_GetActiveCommand(void);
uint32_t Moter_GetCommandRemainingMs(void);

/* Encoder counts are normalized so forward motion is positive on both sides. */
int32_t Moter_GetLeftEncoder(void);
int32_t Moter_GetRightEncoder(void);
void Moter_ResetEncoders(void);

/* Straight-line PID internals exposed for tests and live tuning. */
void Moter_StraightPidStart(int16_t base_left_percent, int16_t base_right_percent);
void Moter_StraightPidUpdate(void);
int32_t Moter_GetStraightPidError(void);
int32_t Moter_GetStraightPidLeftDelta(void);
int32_t Moter_GetStraightPidRightDelta(void);
int16_t Moter_GetLeftCommandPercentX10(void);
int16_t Moter_GetRightCommandPercentX10(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTER_H__ */
