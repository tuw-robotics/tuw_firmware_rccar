
#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <builtin_interfaces/msg/time.h>
#include <stdint.h>
#include <time.h>

// Synchronized time
typedef struct timeval wallclock_timestamp_t;

// Un-synchronized time
typedef uint64_t monotonic_timestamp_t;

#define NO_WAIT 0
#define WAIT_FOREVER portMAX_DELAY
#define ONE_TICK 1

#define S_TO_MS(x) ((x) * 1000LL)
#define S_TO_US(x) ((x) * 1000000LL)
#define MS_TO_US(x) ((x) * 1000LL)
#define US_TO_MS(x) ((x) / 1000LL)
#define US_TO_NS(x) ((x) * 1000LL)
#define US_TO_TICKS(x) (US_TO_MS(x) / portTICK_PERIOD_MS)

#define NS_TO_S(ns) ((ns) / 1000000000LL)
#define NS_SUBS_TO_USEC(ns) (((ns) % 1000000000LL) / 1000LL)

/**
 * @brief Calculates the time difference between two Time messages
 *
 * @param start Pointer to the start Time message
 * @param end Pointer to the end Time message
 * @return builtin_interfaces__msg__Time The time difference as a Time message
 */
builtin_interfaces__msg__Time time_delta(const builtin_interfaces__msg__Time *start, const builtin_interfaces__msg__Time *end);

/**
 * @brief Gets the current wallclock time as a Time message
 *
 * @return builtin_interfaces__msg__Time The current wallclock time
 */
builtin_interfaces__msg__Time time_now();

#endif // TIMING_UTILS_H
