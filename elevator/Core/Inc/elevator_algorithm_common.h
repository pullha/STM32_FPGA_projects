#ifndef ELEVATOR_ALGORITHM_COMMON_H
#define ELEVATOR_ALGORITHM_COMMON_H

#include <stdbool.h>
#include <stdint.h>

/* Project constants for the 5-floor, 2-car elevator demo. */
#define ELEVATOR_FLOOR_MIN       1U
#define ELEVATOR_FLOOR_MAX       5U
#define ELEVATOR_CAR_COUNT       2U
#define ELEVATOR_MAX_STOPS       8U

/* Elevator car IDs are fixed to match array indexes: A=0, B=1.
 * ELEVATOR_CAR_NONE is used when no car is selected or a request is rejected.
 */
typedef enum
{
  ELEVATOR_CAR_A = 0,
  ELEVATOR_CAR_B = 1,
  ELEVATOR_CAR_NONE = 255
} ElevatorCarId;

/* Shared direction type for both elevator movement and hall-call direction. */
typedef enum
{
  ELEVATOR_DIR_DOWN = -1,
  ELEVATOR_DIR_IDLE = 0,
  ELEVATOR_DIR_UP = 1
} ElevatorDirection;

/* One scenario order created by main.c.
 * from_floor is the call floor, direction is the passenger direction,
 * target_floor is where the passenger wants to go, and requested_car is used
 * only when the scenario must force A or B, such as algorithm A on floor 1.
 */
typedef struct
{
  uint8_t from_floor;
  ElevatorDirection direction;
  uint8_t target_floor;
  ElevatorCarId requested_car;
} ElevatorRequest;

/* Elevator state used by the dispatch algorithms.
 * current_floor and direction describe the starting condition.
 * stops[] is the planned stop list that gets updated during batch assignment.
 */
typedef struct
{
  ElevatorCarId id;
  uint8_t current_floor;
  ElevatorDirection direction;
  uint8_t stop_count;
  uint8_t stops[ELEVATOR_MAX_STOPS];
} ElevatorState;

/* Dispatch result returned for each request.
 * accepted=false means the algorithm could not handle that request.
 * reason is a short UART/debug string explaining the decision.
 */
typedef struct
{
  ElevatorCarId assigned_car;
  bool accepted;
  const char *reason;
} ElevatorAssignment;

/* Timing constants used by algorithm D's ETA calculation.
 * The project currently compares algorithms with 5 seconds per floor and
 * 30 seconds of stop/dwell time.
 */
typedef struct
{
  uint16_t seconds_per_floor;
  uint16_t dwell_seconds;
} ElevatorTiming;

/* Return true when a floor number is inside the 1F to 5F model range. */
static inline bool ElevatorFloorIsValid(uint8_t floor)
{
  return (floor >= ELEVATOR_FLOOR_MIN) && (floor <= ELEVATOR_FLOOR_MAX);
}

/* Validate optional caller-side car selection.
 * NONE means the algorithm may choose normally.
 */
static inline bool ElevatorRequestedCarIsValid(ElevatorCarId car)
{
  return (car == ELEVATOR_CAR_A) || (car == ELEVATOR_CAR_B) || (car == ELEVATOR_CAR_NONE);
}

/* Absolute floor distance. Example: floor 2 to floor 5 is distance 3. */
static inline uint8_t ElevatorFloorDistance(uint8_t a, uint8_t b)
{
  return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

/* Calculate movement direction from one floor to another. */
static inline ElevatorDirection ElevatorDirectionBetween(uint8_t from_floor, uint8_t to_floor)
{
  if (to_floor > from_floor)
  {
    return ELEVATOR_DIR_UP;
  }

  if (to_floor < from_floor)
  {
    return ELEVATOR_DIR_DOWN;
  }

  return ELEVATOR_DIR_IDLE;
}

/* Return true when floor is on the inclusive path between a and b. */
static inline bool ElevatorFloorBetween(uint8_t floor, uint8_t a, uint8_t b)
{
  if (a <= b)
  {
    return (floor >= a) && (floor <= b);
  }

  return (floor <= a) && (floor >= b);
}

/* Validate one request before an algorithm scores or assigns it.
 * The direction must match the direction from from_floor to target_floor.
 */
static inline bool ElevatorRequestIsValid(const ElevatorRequest *request)
{
  return (request != 0) &&
         ElevatorFloorIsValid(request->from_floor) &&
         ElevatorFloorIsValid(request->target_floor) &&
         (request->from_floor != request->target_floor) &&
         (request->direction == ElevatorDirectionBetween(request->from_floor, request->target_floor)) &&
         ElevatorRequestedCarIsValid(request->requested_car);
}

#endif /* ELEVATOR_ALGORITHM_COMMON_H */
